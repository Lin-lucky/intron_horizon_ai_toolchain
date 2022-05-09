/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: lmks_106pts_postprocess.cc
 * @Brief: implementation of Lmks4PostProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Tue May 18 2021 02:19:54
 */

#include "model_inference/postprocess/lmks4_postprocess.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

namespace inference {

int Lmks4PostProcess::Init(const std::string &json_str) {
  Json::Value cfg_jv;
  JSONCPP_STRING errs;
  Json::CharReaderBuilder reader_builder;
  std::unique_ptr<Json::CharReader> const json_reader(
      reader_builder.newCharReader());
  bool res = json_reader->parse(
      json_str.c_str(), json_str.c_str() + json_str.size(), &cfg_jv, &errs);
  if (!res || !errs.empty()) {
    LOGE << "Failed to parse config str: " << errs;
    return -1;
  }
  if (cfg_jv["config"]["i_o_stride"].isInt()) {
    i_o_stride_ = cfg_jv["config"]["i_o_stride"].asInt();
  }
  if (cfg_jv["config"]["lmks_num"].isInt()) {
    lmks_num_ = cfg_jv["config"]["lmks_num"].asInt();
  }
  if (cfg_jv["config"]["vector_size"].isInt()) {
    vector_size_ = cfg_jv["config"]["vector_size"].asInt();
  }
  LOGD << "io_stride: " << i_o_stride_ << ", lmks_num: " << lmks_num_
       << ", vector_size: " << vector_size_;
  return 0;
}

int Lmks4PostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "Lmks4PostProcess Execute";
  HOBOT_CHECK(tasks.size() == 1);
  auto task = tasks[0];

  auto lmks_result = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(lmks_result);

  auto roi_task = std::dynamic_pointer_cast<RoiInferenceEngineTask>(task);
  HOBOT_CHECK(roi_task != nullptr);

  int result_num = roi_task->roi_box_.size();
  LOGD << "result num: " << result_num;

  if (roi_task->float_tensors_.empty()) {
    for (int box_idx = 0; box_idx < result_num; box_idx++) {
      auto fake_result = std::make_shared<xstream::Landmarks>();
      fake_result->state_ = xstream::DataState::INVALID;
      lmks_result->datas_.push_back(fake_result);
    }
    return 0;
  }

  int output_size = 1;
  int valid_result_idx = 0;

  HOBOT_CHECK(roi_task->float_tensors_.size() == 2);  // output_layer = 2

  // 对应输出层每个box_result的有效大小
  static std::vector<int> valid_offset{1, 1};
  static std::once_flag flag;
  std::call_once(flag, [&roi_task]() {
    for (int dim_idx = 0; dim_idx < 4; dim_idx++) {
      valid_offset[0] *= roi_task->float_tensors_[0].dim[dim_idx];
      valid_offset[1] *= roi_task->float_tensors_[1].dim[dim_idx];
    }
  });

  for (int box_idx = 0; box_idx < result_num; box_idx++) {
    std::vector<xstream::BaseDataPtr> output;      // 一个box的结果
    if (!roi_task->roi_box_[box_idx].resizable) {  // 无推理结果
      LOGD << "no infer result";
      auto lmks = std::make_shared<xstream::Landmarks>();
      lmks->state_ = xstream::DataState::INVALID;
      output.push_back(lmks);
    } else {
      // 取对应的float_tensor解析
      HandleLmks(roi_task->float_tensors_, valid_offset, valid_result_idx,
                 roi_task->roi_box_[box_idx], &output);
      valid_result_idx++;
    }

    for (int i = 0; i < output_size; i++) {
      auto base_data_vector = std::static_pointer_cast<xstream::BaseDataVector>(
          frame_result->at(i));
      base_data_vector->datas_.push_back(output[i]);
    }
  }

// debug
#if 0
  auto lmks_vector =
      std::static_pointer_cast<xstream::BaseDataVector>(frame_result->at(0));
  for (int box_idx = 0; box_idx < lmks_vector->datas_.size(); box_idx++) {
    auto lmks_ret = std::static_pointer_cast<xstream::Landmarks>(
        lmks_vector->datas_[box_idx]);
    LOGD << *lmks_ret.get();
  }
#endif

  return 0;
}

void Lmks4PostProcess::HandleLmks(const std::vector<FloatTensor> &float_tensors,
                                  const std::vector<int> &valid_offset,
                                  const int valid_result_idx,
                                  const InferBBox roi,
                                  std::vector<xstream::BaseDataPtr> *output) {
  LOGD << "Lmks4PostProcess HandleLmks";
  auto lmks = std::make_shared<xstream::Landmarks>();
  lmks->values_.resize(lmks_num_);
  // x
  LmksPostPro(
      float_tensors[0].value.data() + valid_result_idx * valid_offset[0], roi,
      1, lmks);
  // y
  LmksPostPro(
      float_tensors[1].value.data() + valid_result_idx * valid_offset[1], roi,
      0, lmks);
  output->push_back(lmks);
}

void Lmks4PostProcess::LmksPostPro(const float *mxnet_out, const InferBBox roi,
                                   const int axis,
                                   std::shared_ptr<xstream::Landmarks> lmks) {
  LOGD << "Lmks4PostProcess LmksPostPro";
  float roi_width = roi.x2 - roi.x1;
  float roi_height = roi.y2 - roi.y1;
  for (size_t point_idx = 0; point_idx < lmks_num_; ++point_idx) {
    float max_value = 0;
    int max_index = 0;
    for (int i = 0; i < vector_size_; ++i) {
      int index = i * lmks_num_ + point_idx;
      float value = mxnet_out[index];
      if (value > max_value) {
        max_value = value;
        max_index = i;
      }
    }

    float index = max_index;
    float diff = 0;
    if (index > 0 && index < vector_size_ - 1) {
      int left = (index - 1) * lmks_num_ + point_idx;
      int right = (index + 1) * lmks_num_ + point_idx;
      diff = mxnet_out[right] - mxnet_out[left];
    }

    if (diff > 0) {
      diff = 0.25;
    } else if (diff < 0) {
      diff = -0.25;
    } else {
      diff = 0;
    }

    index = index + (diff * 1.0 + 0.5);
    index = index * i_o_stride_;

    auto cal_score = [&max_value](float score) -> float {
      float new_score = 0.0;
      float val = max_value / 2.0;
      if (val > 1.0) {
        val = 1.0;
      }
      if (score > 0) {
        new_score = std::min(score, val);
      } else {
        new_score = val;
      }
      return new_score;
    };

    if (axis == 0) {
      // vertical vector
      lmks->values_.at(point_idx).y_ =
          index * roi_height / (vector_size_ * i_o_stride_) + roi.y1;
      lmks->values_.at(point_idx).score_ =
          cal_score(lmks->values_.at(point_idx).score_);
    } else {
      // horizontal vector
      lmks->values_.at(point_idx).x_ =
          index * roi_width / (vector_size_ * i_o_stride_) + roi.x1;
      lmks->values_.at(point_idx).score_ =
          cal_score(lmks->values_.at(point_idx).score_);
    }
  }
}

}  // namespace inference
