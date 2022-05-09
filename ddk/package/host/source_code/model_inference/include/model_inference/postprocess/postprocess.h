/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of postprocess
 * @file   postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef POSTPROCESS_H_
#define POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "model_inference/inference_task.h"
#include "xstream/xstream_world.h"

namespace inference {

class PostProcess {
 public:
  static std::shared_ptr<PostProcess> GetInstance(std::string class_name);


  virtual int Init(const std::string &json_str) = 0;

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result) = 0;
};

}  // namespace inference

#endif  // POSTPROCESS_H_
