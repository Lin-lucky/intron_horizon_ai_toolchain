

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of mobilenetv2_postprocess
 * @file   mobilenetv2_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef MOBILENETV2_POSTPROCESS_H_
#define MOBILENETV2_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

class MobileNetV2PostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  void GetMaxResult(const float* scores, int valid_shape,
                    xstream::Classification &cls);

  std::vector<std::string> class_names_;  // 分类标签，需要从文件读取
};

}  // namespace inference

#endif  // MOBILENETV2_POSTPROCESS_H_
