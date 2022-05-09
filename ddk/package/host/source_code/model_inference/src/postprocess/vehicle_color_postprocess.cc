/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of vehicle_color_postprocess
 * @file   vehicle_color_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.20
 */

#include "model_inference/postprocess/vehicle_color_postprocess.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
#include "hobotlog/hobotlog.hpp"
#include "model_inference/inference_engine.h"

namespace inference {

int VehicleColorPostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "VehicleColorPostProcess Execute";

  auto vehicle_color_rets = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(vehicle_color_rets);

  for (size_t i = 0; i < tasks.size(); ++i) {
      auto task = tasks[i];
    if (task->float_tensors_.size() == 0 ||
        task->float_tensors_[0].value.size() == 0) {
      auto fake_vehicle_color = std::make_shared<xstream::Attribute_<int>>();
      fake_vehicle_color->state_ = xstream::DataState::INVALID;
      vehicle_color_rets->datas_.push_back(fake_vehicle_color);
      continue;
    }
    auto vehicle_color = std::make_shared<xstream::Attribute_<int>>();
    int real_c_dim = task->float_tensors_[0].dim[3];
    ParseVehicleColor(
        task->float_tensors_[0].value.data(), real_c_dim, vehicle_color);
    vehicle_color_rets->datas_.push_back(vehicle_color);
  }

  return 0;
}

void VehicleColorPostProcess::ParseVehicleColor(
    const float* feature, const int real_c_dim,
    std::shared_ptr<xstream::Attribute_<int>> result) {
  // compute feature size, for vehicle color bpu output size equals to 1
  int max_index = 12;
  float max_score = -999.0f;
  for (int c = 0; c < real_c_dim; ++c) {
    float cur_score = feature[c];
    if (cur_score > max_score) {
      max_index = c;
      max_score = cur_score;
    }
  }

  result->value_ = max_index;
  result->score_ = max_score;
  LOGD << "vehicle color: " << max_index << ", score: " << max_score;
}

}  // namespace inference
