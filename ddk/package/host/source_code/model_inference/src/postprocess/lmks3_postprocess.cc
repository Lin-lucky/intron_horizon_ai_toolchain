/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: lmks3_postprocess.cc
 * @Brief: implementation of Lmks3PostProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Wed May 19 2021 02:51:43
 */

#include "model_inference/postprocess/lmks3_postprocess.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

namespace inference {

int Lmks3PostProcess::Init(const std::string &json_str) {
  LOGD << json_str;
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
  if (cfg_jv["i_o_stride"].isInt()) {
    i_o_stride_ = cfg_jv["i_o_stride"].asInt();
  }
  LOGD << "io_stride: " << i_o_stride_;
  return 0;
}

int Lmks3PostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "Lmks3PostProcess Execute";
  HOBOT_CHECK(tasks.size() == 1);
  auto task = tasks[0];

  auto landmarks_result = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(landmarks_result);

  auto roi_task = std::dynamic_pointer_cast<RoiInferenceEngineTask>(task);
  HOBOT_CHECK(roi_task != nullptr);

  int result_num = roi_task->roi_box_.size();
  LOGD << "result num: " << result_num;

  if (roi_task->float_tensors_.empty()) {
    for (int box_idx = 0; box_idx < result_num; box_idx++) {
      auto fake_result = std::make_shared<xstream::Landmarks>();
      fake_result->state_ = xstream::DataState::INVALID;
      landmarks_result->datas_.push_back(fake_result);
    }
    return 0;
  }

  int output_size = 1;
  int valid_result_idx = 0;

  HOBOT_CHECK(roi_task->float_tensors_.size() == 1)
      << roi_task->float_tensors_.size();  // output_layer = 1

  // 对应输出层每个box_result的有效大小
  static int valid_offset = 1;
  static std::once_flag flag;
  std::call_once(flag, [&roi_task]() {
    for (int dim_idx = 0; dim_idx < 4; dim_idx++) {
      valid_offset *= roi_task->float_tensors_[0].dim[dim_idx];
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
      LmksPostPro(roi_task->float_tensors_, valid_offset, valid_result_idx,
                  roi_task->roi_box_[box_idx], &output);
      valid_result_idx++;
    }

    for (int i = 0; i < output_size; i++) {
      auto base_data_vector = std::static_pointer_cast<xstream::BaseDataVector>(
          frame_result->at(i));
      base_data_vector->datas_.push_back(output[i]);
    }
  }
}

void Lmks3PostProcess::LmksPostPro(
    const std::vector<FloatTensor> &float_tensors, const int valid_offset,
    const int valid_result_idx, const InferBBox roi,
    std::vector<xstream::BaseDataPtr> *output) {
  LOGD << "Lmks3PostProcess LmksPostPro";
  auto mxnet_output =
      float_tensors[0].value.data() + valid_result_idx * valid_offset;
  int box_height = floor(roi.y2 - roi.y1);
  int box_width = floor(roi.x2 - roi.x1);
  auto ratio_h = float_tensors[0].dim[1] * i_o_stride_;
  auto ratio_w = float_tensors[0].dim[2] * i_o_stride_;

  int step = 4;  // 4 for performance optimization, use 1 for normal cases
  auto landmarks = std::make_shared<xstream::Landmarks>();
  landmarks->values_.resize(float_tensors[0].dim[3]);
  for (size_t c = 0; c < float_tensors[0].dim[3]; ++c) {  // c
    float max_value = 0;
    int max_index[2] = {0, 0};
    for (auto h = 0; h < float_tensors[0].dim[1]; h += step) {    // h
      for (auto w = 0; w < float_tensors[0].dim[2]; w += step) {  // w
        int index = h * float_tensors[0].dim[2] * float_tensors[0].dim[3] +
                    w * float_tensors[0].dim[3] + c;
        float value = mxnet_output[index];
        if (value > max_value) {
          max_value = value;
          max_index[0] = h;
          max_index[1] = w;
        }
      }  // w
    }    // h
    // performance optimization
    auto is_max = false;
    auto campare_func = [&float_tensors, &max_index, &max_value, &is_max,
                         mxnet_output, this](int h, int w, int c) {
      int index = h * float_tensors[0].dim[2] * float_tensors[0].dim[3] +
                  w * float_tensors[0].dim[3] + c;
      if (max_value < mxnet_output[index]) {
        max_value = mxnet_output[index];
        max_index[0] = h;
        max_index[1] = w;
        is_max = false;
      }
    };
    while (false == is_max) {
      is_max = true;
      int h = max_index[0];
      int w = max_index[1];
      if (h > 0) {
        campare_func(h - 1, w, c);
      }
      if (h < float_tensors[0].dim[1] - 1) {
        campare_func(h + 1, w, c);
      }
      if (w > 0) {
        campare_func(h, w - 1, c);
      }
      if (w < float_tensors[0].dim[2] - 1) {
        campare_func(h, w + 1, c);
      }
    }
    // end performance optimization

    float y = max_index[0];
    float x = max_index[1];

    float diff_x = 0, diff_y = 0;
    if (y > 0 && y < float_tensors[0].dim[1] - 1) {
      int top = (y - 1) * float_tensors[0].dim[2] * float_tensors[0].dim[3] +
                x * float_tensors[0].dim[3] + c;
      int down = (y + 1) * float_tensors[0].dim[2] * float_tensors[0].dim[3] +
                 x * float_tensors[0].dim[3] + c;
      diff_y = mxnet_output[down] - mxnet_output[top];
    }
    if (x > 0 && x < float_tensors[0].dim[2] - 1) {
      int left = y * float_tensors[0].dim[2] * float_tensors[0].dim[3] +
                 (x - 1) * float_tensors[0].dim[3] + c;
      int right = y * float_tensors[0].dim[2] * float_tensors[0].dim[3] +
                  (x + 1) * float_tensors[0].dim[3] + c;
      diff_x = mxnet_output[right] - mxnet_output[left];
    }

    // y = y + (diff_y * diff_coeff + 0.5); // for float32 of model output
    // x = x + (diff_x * diff_coeff + 0.5); // for float32 of model output
    y = y + (diff_y > 0 ? 0.25 : -0.25) + 0.5;
    x = x + (diff_x > 0 ? 0.25 : -0.25) + 0.5;
    y = y * i_o_stride_;
    x = x * i_o_stride_;
    y = y * box_height / ratio_h + roi.y1;
    x = x * box_width / ratio_w + roi.x1;

    auto &point = landmarks->values_[c];
    point.x_ = x;
    point.y_ = y;
    point.score_ = max_value;
    LOGD << "hand_lmk" << c << " x y score:( " << x << " " << y << " "
         << point.score_ << ")";
  }  // c

  output->push_back(landmarks);
}

}  // namespace inference
