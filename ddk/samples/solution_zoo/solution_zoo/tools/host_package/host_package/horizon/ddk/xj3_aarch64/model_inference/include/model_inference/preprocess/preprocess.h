/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_task
 * @file   inference_task.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "json/json.h"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "model_inference/inference_task.h"

namespace inference {

class Inferencer;

class PreProcess {
 public:
  static std::shared_ptr<PreProcess> GetInstance(
      std::string class_name, Inferencer* infer);

  // 构造PreProcess对象需要传入Inferencer对象
  explicit PreProcess(Inferencer* infer) : infer_(infer) {}

  virtual ~PreProcess() {}
  virtual int Init(const std::string &json_str) = 0;

  // Execute函数需要构建task(Tensor or ROI)
  // 需要派生类自行实现
  virtual int Execute(
      const std::vector<xstream::BaseDataPtr> &input,
      std::vector<std::shared_ptr<InferenceEngineTask>>& tasks) {
    return -1;
  }

  Inferencer* infer_;  // Execute需要infer_模型信息
};

}  // namespace inference

#endif  // PREPROCESS_H_
