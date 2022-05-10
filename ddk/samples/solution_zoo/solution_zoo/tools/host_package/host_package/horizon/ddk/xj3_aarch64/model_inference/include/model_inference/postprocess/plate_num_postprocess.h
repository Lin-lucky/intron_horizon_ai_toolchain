

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of plate_num_postprocess
 * @file   plate_num_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.20
 */
#ifndef PLATE_NUM_POSTPROCESS_H_
#define PLATE_NUM_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "xstream/vision_type.h"
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"

namespace inference {

class PlateNumPostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str) {
    return 0;
  }

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  void ParsePlateNum(
    const FloatTensor* float_tensor,
    const int valid_idx, const int valid_offset,
    std::shared_ptr<xstream::DataArray_<int>> plate_num);

  std::string Convert2String(
    std::vector<int> numerical_plate_num);
};

}  // namespace inference

#endif  // PLATE_NUM_POSTPROCESS_H_
