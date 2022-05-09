/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: lmks_seq_preprocess.cc
 * @Brief: implementation of GesturePreProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu May 20 2021 05:52:09
 */

#include "model_inference/preprocess/gesture_preprocess.h"
#include <cstring>
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"
#include "model_inference/inference_engine.h"
#include "model_inference/inferencer.h"

namespace inference {

void InputFloat2Int(int8_t* feat_buffer1, int input_quanti_factor,
                    float* BPU_input_data, int N, int H, int W, int C,
                    int frame_input_size) {
  int32_t tmp = 0;
  int index = 0;
  for (int n = 0; n < N; ++n) {
    for (int h = 0; h < H; ++h) {
      for (int w = 0; w < W; ++w) {
        for (int c = 0; c < C; ++c) {
          int offset = n * H * W * C + h * W * C + w * C + c;
          tmp = floor(BPU_input_data[offset] * input_quanti_factor);
          if (tmp > 127) {
            tmp = 127;
          } else if (tmp < -128) {
            tmp = -128;
          }
          feat_buffer1[index] = static_cast<int8_t>(tmp);
          index++;
        }
      }
    }
  }
  HOBOT_CHECK(index == frame_input_size);
}

int GesturePreProcess::Init(const std::string& json_str) {
  LOGD << "GesturePreProcess Init";
  Json::Value cfg_jv;
  Json::CharReaderBuilder reader_builder;
  JSONCPP_STRING errs;
  std::unique_ptr<Json::CharReader> const json_reader(
      reader_builder.newCharReader());
  bool ret = json_reader->parse(
      json_str.c_str(), json_str.c_str() + json_str.size(), &cfg_jv, &errs);
  if (!ret || !errs.empty()) {
    LOGE << "Failed to parse config string, " << errs;
    return -1;
  }

  if (cfg_jv["seq_len"].isInt()) {
    seq_len_ = cfg_jv["seq_len"].asInt();
  }
  if (cfg_jv["kps_len"].isInt()) {
    kps_len_ = cfg_jv["kps_len"].asInt();
  }
  if (cfg_jv["stride"].isDouble()) {
    stride_ = cfg_jv["stride"].asFloat();
  }
  if (cfg_jv["max_gap"].isDouble()) {
    max_gap_ = cfg_jv["max_gap"].asFloat();
  }
  if (cfg_jv["buf_len"].isInt()) {
    buf_len_ = cfg_jv["buf_len"].asInt();
  }
  if (cfg_jv["norm_kps_conf"].isInt()) {
    norm_kps_conf_ = cfg_jv["norm_kps_conf"].asInt() ? true : false;
  }
  if (cfg_jv["kps_norm_scale"].isDouble()) {
    kps_norm_scale_ = cfg_jv["kps_norm_scale"].asFloat();
  }
  if (cfg_jv["input_shift"].isInt()) {
    input_shift_ = cfg_jv["input_shift"].asInt();
  }
  LOGD << "seq_len: " << seq_len_ << ", kps_len: " << kps_len_
       << ", stride: " << stride_ << ", max_gap: " << max_gap_
       << ", buf_len: " << buf_len_ << ", kps_norm_scale: " << kps_norm_scale_
       << ", input_shift: " << input_shift_;

  lmks_proc_.Init(kps_len_, seq_len_, stride_, max_gap_, kps_norm_scale_,
                  norm_kps_conf_, buf_len_);

  return 0;
}

int GesturePreProcess::Execute(
    const std::vector<xstream::BaseDataPtr>& input,
    std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
  LOGD << "GesturePreProcess Execute";
  auto xstream_image = std::static_pointer_cast<xstream::ImageFrame>(input[0]);
  auto rois = std::static_pointer_cast<BaseDataVector>(input[1]);
  auto lmkses = std::static_pointer_cast<BaseDataVector>(input[2]);
  auto disappeared_ids = std::static_pointer_cast<BaseDataVector>(input[3]);

  uint64_t timestamp = xstream_image->time_stamp_;

  LOGD << "roi size: " << rois->datas_.size();
  for (size_t roi_idx = 0; roi_idx < rois->datas_.size(); ++roi_idx) {
    auto tensor_task = std::make_shared<TensorInferenceEngineTask>();
    tasks.push_back(tensor_task);
    tensor_task->sequence_id_ = xstream_image->frame_id_;

    auto roi = std::static_pointer_cast<xstream::BBox>(rois->datas_[roi_idx]);
    auto lmks =
        std::static_pointer_cast<xstream::Landmarks>(lmkses->datas_[roi_idx]);

    if (roi->state_ != xstream::DataState::VALID ||
        lmks->values_.size() != static_cast<uint32_t>(kps_len_)) {
      LOGD << "invalid continue";
      continue;
    } else {
      tensor_task->timestamp_ = timestamp;
      tensor_task->track_id_ = roi->id_;

      int8_t* feature_buf = nullptr;
      static int input_size = 1;

      // preprocess
      auto cached_kpses = std::make_shared<BaseDataVector>();
      // currently only support gesture detection
      lmks_proc_.Execute(roi, lmks, cached_kpses, timestamp);
      if (cached_kpses->datas_.size() < static_cast<uint32_t>(seq_len_)) {
        continue;
      } else {
        // prepare tensor properties
        InferenceEngine::GetInstance()->PrepareModelTensorProperties(
            infer_->input_model_info_, tensor_task->input_tensors_);
        InferenceEngine::GetInstance()->PrepareModelTensorProperties(
            infer_->output_model_info_, tensor_task->output_tensors_);
        static std::vector<int> input_valid_shape;
        static std::vector<int> input_aligned_shape;
        static std::once_flag flag;
        std::call_once(flag, [&tensor_task] {
          for (int i = 0; i < tensor_task->input_tensors_[0]
                                  .properties.valid_shape.num_dimensions;
               ++i) {
            input_valid_shape.push_back(
                tensor_task->input_tensors_[0]
                    .properties.valid_shape.dimension_size[i]);
          }
          for (int i = 0; i < tensor_task->input_tensors_[0]
                                  .properties.aligned_shape.num_dimensions;
               ++i) {
            input_aligned_shape.push_back(
                tensor_task->input_tensors_[0]
                    .properties.aligned_shape.dimension_size[i]);
            input_size *= input_aligned_shape[i];
          }
        });

        // tensor with aligned shape, padding 0
        LmksProcessTensor tensor(input_aligned_shape[0], input_aligned_shape[1],
                                 input_aligned_shape[2],
                                 input_aligned_shape[3]);
        for (int nn = 0; nn < input_valid_shape[0]; ++nn) {
          for (int hh = 0; hh < input_valid_shape[1]; ++hh) {
            for (int ww = 0; ww < input_valid_shape[2]; ++ww) {
              int cached_data_idx = input_valid_shape[2] == seq_len_ ? ww : hh;
              int kps_idx = input_valid_shape[1] == kps_len_ ? hh : ww;
              auto cur_kps = std::static_pointer_cast<xstream::Landmarks>(
                  cached_kpses->datas_[cached_data_idx]);
              tensor.Set(nn, hh, ww, 0, cur_kps->values_[kps_idx].x_);
              tensor.Set(nn, hh, ww, 1, cur_kps->values_[kps_idx].y_);
              tensor.Set(nn, hh, ww, 2, cur_kps->values_[kps_idx].score_);
            }
          }
        }

        float* BPU_input_data = tensor.data.data();
        feature_buf = static_cast<int8_t*>(malloc(input_size));
        int input_quanti_factor = 1 << input_shift_;
        InputFloat2Int(feature_buf, input_quanti_factor, BPU_input_data,
                       input_aligned_shape[0], input_aligned_shape[1],
                       input_aligned_shape[2], input_aligned_shape[3],
                       input_size);
      }

      // 申请Tensor
      InferenceEngine::GetInstance()->AllocModelTensor(
          tensor_task->input_tensors_);
      InferenceEngine::GetInstance()->AllocModelTensor(
          tensor_task->output_tensors_);
      HOBOT_CHECK(tensor_task->input_tensors_.size() == 1)
          << "input tensor size: " << tensor_task->input_tensors_.size()
          << ", need 1";
      auto& input_tensor = tensor_task->input_tensors_[0];

      if (input_tensor.properties.tensor_type == TENSOR_TYPE_S8) {
        HOBOT_CHECK(input_tensor.sys_mem[0].mem_size == input_size)
            << "allocated input size: " << input_tensor.sys_mem[0].mem_size
            << ", need: " << input_size;
        memcpy(input_tensor.sys_mem[0].vir_addr,
               reinterpret_cast<uint8_t*>(feature_buf),
               input_tensor.sys_mem[0].mem_size);
      } else {
        HOBOT_CHECK(0) << "not support data type: "
                       << input_tensor.properties.tensor_type;
      }

      free(feature_buf);
      feature_buf = nullptr;
    }
  }

  lmks_proc_.Clean(disappeared_ids);

  return 0;
}

}  // namespace inference
