

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of faceid_postprocess
 * @file   faceid_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.18
 */
#ifndef FACEID_POSTPROCESS_H_
#define FACEID_POSTPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "xstream/vision_type.h"
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"

namespace inference {

class FaceIDPostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str) {
    return 0;
  }

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  xstream::BaseDataPtr FaceFeaturePostPro(
    const float* mxnet_outs, std::shared_ptr<xstream::FloatFeature> feature);

  void L2Norm(std::vector<float> &input, int length);
};

}  // namespace inference

#endif  // FACEID_POSTPROCESS_H_
