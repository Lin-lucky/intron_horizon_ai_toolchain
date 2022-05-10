/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: result_util.cpp
 * @Brief: implementation of result_util
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu Dec 17 2020 11:06:23
 */

#include "multitask_postprocess_method/result_util.h"
#include "hobotlog/hobotlog.hpp"

namespace xstream {

void CoordinateTransfrom(DnnAsyncData &dnn_result, OutMsg &det_results,
                         int model_input_width, int model_input_height) {
  int src_image_width = dnn_result.src_image_width;
  int src_image_height = dnn_result.src_image_height;
  LOGD << "src image size: " << src_image_width << " x " << src_image_height;
  LOGD << "model input size: " << model_input_width << " x "
       << model_input_height;
  for (auto &boxes : det_results.boxes) {
    for (auto &box : boxes.second) {
      box.x1_ = box.x1_ * src_image_width / model_input_width;
      box.y1_ = box.y1_ * src_image_height / model_input_height;
      box.x2_ = box.x2_ * src_image_width / model_input_width;
      box.y2_ = box.y2_ * src_image_height / model_input_height;
    }
  }

  for (auto &landmarks : det_results.landmarks) {
    for (auto &landmark : landmarks.second) {
      for (auto &point : landmark.values_) {
        point.x_ = point.x_ * src_image_width / model_input_width;
        point.y_ = point.y_ * src_image_height / model_input_height;
      }
    }
  }
}

int GetOutputInfo(BPU_MODEL_S bpu_model, std::vector<ModelOutputInfo> &info) {
  if (static_cast<uint32_t>(bpu_model.output_num) != info.size()) {
    LOGE << "model output num: " << bpu_model.output_num
         << " is different from output info size: " << info.size();
    return -1;
  }
  for (size_t i = 0; i < info.size(); ++i) {
    ModelOutputInfo &out_info = info[i];
    const uint8_t *shifts = bpu_model.outputs[i].shifts;
    int aligned_shape_dim = bpu_model.outputs[i].aligned_shape.ndim;
    HOBOT_CHECK(aligned_shape_dim == 4) << "aligned_shape_dim: "
                                        << aligned_shape_dim;
    out_info.shift = shifts[0];
    for (int dim = 0; dim < aligned_shape_dim; ++dim) {
      out_info.aligned_dims.push_back(
          bpu_model.outputs[i].aligned_shape.d[dim]);
    }
  }
  return 0;
}

}  // namespace xstream
