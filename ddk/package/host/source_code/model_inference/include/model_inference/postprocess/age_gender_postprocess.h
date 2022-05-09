

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of age_gender_postprocess
 * @file   age_gender_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef AGE_GENDER_POSTPROCESS_H_
#define AGE_GENDER_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

class AgeGenderPostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str) {
    return 0;
  }

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

  void HandleAgeGender(const std::vector<FloatTensor> &float_tensors,
                       const std::vector<int> &valid_offset,
                       const int batch_idx,
                       std::vector<xstream::BaseDataPtr> *output);

  xstream::BaseDataPtr AgePostPro(const float* mxnet_out);
  xstream::BaseDataPtr GenderPostPro(const float* mxnet_out);

  const std::vector<int> g_age_range_ = {1,  6,  7,  12, 13, 18, 19, 28,
                                        29, 35, 36, 45, 46, 55, 56, 100};
};

}  // namespace inference

#endif  // AGE_GENDER_POSTPROCESS_H_
