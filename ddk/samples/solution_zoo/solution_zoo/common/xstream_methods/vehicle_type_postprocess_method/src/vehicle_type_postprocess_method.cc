/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: MultitaskPostProcessMethod.cpp
 * @Brief: definition of the MultitaskPostProcessMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-12-03 20:00:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-12-03 21:23:08
 */

#include "vehicle_type_postprocess_method/vehicle_type_postprocess_method.h"
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "dnn_async_data.h"
#include "dnn_util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"
#include "xstream/vision_type.h"

namespace xstream {

int VehicleTypePostProcessMethod::Init(const std::string &cfg_path) {
  LOGD << "VehicleTypePostProcessMethod Init";
  DnnPostProcessMethod::Init(cfg_path);

  output_size_ = config_.GetIntValue("output_size", 0);
  HOBOT_CHECK(output_size_ > 0) << "error output size: " << output_size_;

  return 0;
}

int VehicleTypePostProcessMethod::ParseDnnResult(
    DnnAsyncData &dnn_result, std::vector<BaseDataPtr> &frame_result) {
  LOGD << "VehicleTypePostProcessMethod ParseDnnResult";

  if (dnn_result.output_tensors.empty()) {
    LOGD << "failed to run model in predict method";
    frame_result.resize(output_size_);

    for (uint32_t i = 0; i < output_size_; ++i) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      frame_result[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }

    return 0;
  }

  frame_result.resize(output_size_);
  auto &output_tensors = dnn_result.output_tensors[0];
  int output_num = dnn_result.dnn_model->bpu_model.output_num;
  LOGD << "output num: " << output_num;
  // multiple results for different boxes in one output tensor
  for (uint32_t out_idx = 0; out_idx < output_size_; ++out_idx) {
    auto base_data_vector = std::make_shared<BaseDataVector>();
    for (int i = 0; i < output_num; ++i) {
      if (HB_SYS_isMemCachable(&(output_tensors[i].data))) {
        HB_SYS_flushMemCache(&(output_tensors[i].data),
                             HB_SYS_MEM_CACHE_INVALIDATE);
      }
      auto output_tensor = output_tensors[i];
      ParseVehicleType(dnn_result.dnn_model->bpu_model, dnn_result.valid_box,
                       output_tensor, base_data_vector);
    }
    frame_result[out_idx] =
        std::static_pointer_cast<BaseData>(base_data_vector);
  }

  return 0;
}

void VehicleTypePostProcessMethod::ParseVehicleType(
    BPU_MODEL_S bpu_model, std::vector<int> &valid_box,
    BPU_TENSOR_S output_tensor, std::shared_ptr<BaseDataVector> result) {
  // compute feature size, for vehicle_type bpu output size equals to 1
  int aligned_shape_dim = bpu_model.outputs[0].aligned_shape.ndim;
  HOBOT_CHECK(aligned_shape_dim == 4) << "error aligned shape dim: "
                                      << aligned_shape_dim;
  auto aligned_shape = bpu_model.outputs[0].aligned_shape.d;

  int real_shape_dim = bpu_model.outputs[0].shape.ndim;
  HOBOT_CHECK(real_shape_dim == 4) << "error aligned shape dim: "
                                   << real_shape_dim;
  auto real_shape = bpu_model.outputs[0].shape.d;

  auto shifts = bpu_model.outputs[0].shifts;
  int feature_size = aligned_shape[1] * aligned_shape[2] * aligned_shape[3];

  auto features = reinterpret_cast<int32_t *>(output_tensor.data.virAddr);

  for (size_t roi_idx = 0, feature_idx = 0; roi_idx < valid_box.size();
       ++roi_idx) {
    auto vehicle_type =
        std::make_shared<xstream::Attribute_<int>>();
    if (valid_box[roi_idx] != 1) {
      vehicle_type->value_ = -1;
    } else {
      auto data = features + feature_size * feature_idx;

      int max_index = 0;
      float max_score = -999.0f;
      // TODO(shiyu.fu): will layout of output always be NHWC?
      for (int c = 0; c < real_shape[3]; ++c) {
        float cur_score = GetFloatByInt(data[c], shifts[c]);
        if (cur_score > max_score) {
          max_index = c;
          max_score = cur_score;
        }
      }

      vehicle_type->value_ = max_index;
      vehicle_type->score_ = max_score;
      LOGD << "vehicle type: " << max_index << ", score: " << max_score;

      feature_idx++;
    }
    result->datas_.push_back(vehicle_type);
  }
  LOGD << "result->datas_.size(): " << result->datas_.size();
}

}  // namespace xstream
