

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of vehicle_type_postprocess
 * @file   vehicle_type_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.20
 */
#ifndef VEHICLE_TYPE_POSTPROCESS_H_
#define VEHICLE_TYPE_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "xstream/vision_type.h"
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"

namespace inference {

class VehicleTypePostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str) {
    return 0;
  }

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  void ParseVehicleType(
    const float* float_tensor, const int real_c_dim,
    std::shared_ptr<xstream::Attribute_<int>> vehicle_type);
};

}  // namespace inference

#endif  // VEHICLE_TYPE_POSTPROCESS_H_
