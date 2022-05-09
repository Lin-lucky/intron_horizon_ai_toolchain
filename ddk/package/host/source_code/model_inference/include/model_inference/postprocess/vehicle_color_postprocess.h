

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of vehicle_color_postprocess
 * @file   vehicle_color_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.20
 */
#ifndef VEHICLE_COLOR_POSTPROCESS_H_
#define VEHICLE_COLOR_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "xstream/vision_type.h"
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"

namespace inference {

class VehicleColorPostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str) {
    return 0;
  }

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  void ParseVehicleColor(
    const float* feature, const int real_c_dim,
    std::shared_ptr<xstream::Attribute_<int>> result);
};

}  // namespace inference

#endif  // VEHICLE_COLOR_POSTPROCESS_H_
