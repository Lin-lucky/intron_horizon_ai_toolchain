/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: VehicleColorPostProcessMethod.cpp
 * @Brief: implementation of VehicleColorPostProcessMethod
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Mon Jan 04 2021 10:58:06
 */

#include "vehicle_color_postprocess_method/vehicle_color_postprocess_method.h"
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "dnn_async_data.h"
#include "dnn_util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

namespace xstream {

int VehicleColorPostProcessMethod::Init(const std::string &cfg_path) {
  LOGD << "VehicleColorPostProcessMethod Init";
  DnnPostProcessMethod::Init(cfg_path);

  output_size_ = config_.GetIntValue("output_size", 0);
  HOBOT_CHECK(output_size_ > 0) << "error output size: " << output_size_;

  return 0;
}

int VehicleColorPostProcessMethod::ParseDnnResult(
    DnnAsyncData &dnn_result, std::vector<BaseDataPtr> &frame_result) {
  LOGD << "VehicleColorPostProcessMethod ParseDnnResult";

  {
    RUN_PROCESS_TIME_PROFILER("VehicleColorPostProcess");
    RUN_FPS_PROFILER("VehicleColorPostProcess");
    frame_result.resize(1);
    auto vehicle_color_rets = std::make_shared<BaseDataVector>();
    frame_result[0] = vehicle_color_rets;

    for (size_t i = 0; i < dnn_result.output_tensors.size(); ++i) {
      auto output_tensor = dnn_result.output_tensors[i];
      if (output_tensor.empty()) {
        auto fake_vehicle_color = std::make_shared<Attribute_<int>>();
        fake_vehicle_color->state_ = DataState::INVALID;
        vehicle_color_rets->datas_.push_back(fake_vehicle_color);
        continue;
      }

      auto tensor = output_tensor[0];
      if (HB_SYS_isMemCachable(&tensor.data)) {
        HB_SYS_flushMemCache(&tensor.data, HB_SYS_MEM_CACHE_INVALIDATE);
      }
      auto vehicle_color = std::make_shared<Attribute_<int>>();
      ParseVehicleColor(dnn_result.dnn_model->bpu_model, tensor, vehicle_color);
      vehicle_color_rets->datas_.push_back(vehicle_color);
    }
  }

  return 0;
}

void VehicleColorPostProcessMethod::ParseVehicleColor(
    BPU_MODEL_S bpu_model,
    BPU_TENSOR_S output_tensor, std::shared_ptr<Attribute_<int>> result) {
  // compute feature size, for vehicle color bpu output size equals to 1
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

  auto feature = reinterpret_cast<int32_t *>(output_tensor.data.virAddr);

  int max_index = 12;
  float max_score = -999.0f;
  for (int c = 0; c < real_shape[3]; ++c) {
    float cur_score = GetFloatByInt(feature[c], shifts[c]);
    if (cur_score > max_score) {
      max_index = c;
      max_score = cur_score;
    }
  }

  result->value_ = max_index;
  result->score_ = max_score;
  LOGD << "vehicle color: " << max_index << ", score: " << max_score;
}

}  // namespace xstream
