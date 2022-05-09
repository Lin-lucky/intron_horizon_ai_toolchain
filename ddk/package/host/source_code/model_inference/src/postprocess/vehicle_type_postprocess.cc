/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of vehicle_type_postprocess
 * @file   vehicle_type_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.20
 */

#include "model_inference/postprocess/vehicle_type_postprocess.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
#include "hobotlog/hobotlog.hpp"
#include "model_inference/inference_engine.h"

namespace inference {

int VehicleTypePostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "VehicleTypePostProcess Execute";
  HOBOT_CHECK(tasks.size() == 1);
  auto task = tasks[0];
  auto vehicle_type_result = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(vehicle_type_result);  // output: vehicle_type

  auto roi_task =
      std::dynamic_pointer_cast<RoiInferenceEngineTask>(task);
  HOBOT_CHECK(roi_task != nullptr);

  int result_num = roi_task->roi_box_.size();
  int valid_result_idx = 0;

  // task failed
  if (roi_task->float_tensors_.size() <= 0) {
    for (int box_idx = 0; box_idx < result_num; box_idx++) {
      auto vehicle_type = std::make_shared<xstream::Attribute_<int>>();
      vehicle_type->state_ = xstream::DataState::INVALID;
      vehicle_type_result->datas_.push_back(vehicle_type);
    }
    return 0;
  }
  HOBOT_CHECK(roi_task->float_tensors_.size() == 1);  // output_layer = 1

  // 对应输出层每个box_result的有效大小
  static int valid_offset = 1;
  static std::once_flag flag;
  std::call_once(flag, [&roi_task]() {
    for (int dim_idx = 0; dim_idx < 4; dim_idx++) {
      valid_offset *= roi_task->float_tensors_[0].dim[dim_idx];
    }
  });

  for (int box_idx = 0; box_idx < result_num; box_idx++) {
    // 一个box的结果
    auto vehicle_type = std::make_shared<xstream::Attribute_<int>>();
    if (!roi_task->roi_box_[box_idx].resizable) {  // 无推理结果
      vehicle_type->state_ = xstream::DataState::INVALID;
    } else {
      // 取对应的float_tensor解析
      ParseVehicleType(
          roi_task->float_tensors_[0].value.data() +
              valid_offset * valid_result_idx,
          roi_task->float_tensors_[0].dim[3], vehicle_type);
      valid_result_idx++;
    }

    vehicle_type_result->datas_.push_back(vehicle_type);
  }

  return 0;
}

void VehicleTypePostProcess::ParseVehicleType(
    const float* float_tensor, const int real_c_dim,
    std::shared_ptr<xstream::Attribute_<int>> vehicle_type) {
  int max_index = 0;
  float max_score = -999.0f;
  // TODO(shiyu.fu): will layout of output always be NHWC?
  for (int c = 0; c < real_c_dim; ++c) {
    float cur_score = float_tensor[c];
    if (cur_score > max_score) {
      max_index = c;
      max_score = cur_score;
    }
  }

  vehicle_type->value_ = max_index;
  vehicle_type->score_ = max_score;
  LOGD << "vehicle type: " << max_index << ", score: " << max_score;
}

}  // namespace inference
