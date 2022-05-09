

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of face_quality_postprocess
 * @file   face_quality_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.18
 */
#ifndef FACE_QUALITY_POSTPROCESS_H_
#define FACE_QUALITY_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

class FaceQualityPostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

  void FaceQualityPostPro(
    const std::vector<FloatTensor> &float_tensors,
    std::vector<xstream::BaseDataPtr> *output);

  float threshold_ = 0.0f;
  int output_slot_size_ = 14;  // 14个输出
};

}  // namespace inference

#endif  // FACE_QUALITY_POSTPROCESS_H_
