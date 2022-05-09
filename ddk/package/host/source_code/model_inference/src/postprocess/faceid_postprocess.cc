/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of faceid_postprocess
 * @file   faceid_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/postprocess/faceid_postprocess.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

int FaceIDPostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "FaceIDPostProcess Execute";
  auto base_data_vector = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(base_data_vector);

  int task_num = tasks.size();

  for (int dim_idx = 0; dim_idx < task_num; dim_idx++) {
    auto task = tasks[dim_idx];
    auto feature = std::make_shared<xstream::FloatFeature>();
    if (task->float_tensors_.size() == 0) {  // 任务失败
      feature->state_ = xstream::DataState::INVALID;
    } else {
      FaceFeaturePostPro(task->float_tensors_[0].value.data(), feature);
    }
    base_data_vector->datas_.push_back(feature);
  }
  return 0;
}

xstream::BaseDataPtr FaceIDPostProcess::FaceFeaturePostPro(
    const float* mxnet_outs,
    std::shared_ptr<xstream::FloatFeature> feature) {
  static const int kFeatureCnt = 128;
  feature->values_.resize(kFeatureCnt);
  for (int i = 0; i < kFeatureCnt; i++) {
    feature->values_[i] = mxnet_outs[i];
    LOGD << mxnet_outs[i];
  }
  L2Norm(feature->values_, kFeatureCnt);
  LOGD << "feature: [";
  for (int i = 0; i < kFeatureCnt; i++) {
    LOGD << feature->values_[i];
  }
  LOGD << "]";
  return feature;
}

void FaceIDPostProcess::L2Norm(std::vector<float> &input, int length) {
  float sum = 0.0;
  float eps = 1e-10;
  for (int i = 0; i < length; ++i) {
    sum += input[i] * input[i];
  }
  sum = sqrt(sum) + eps;
  for (int i = 0; i < length; ++i) {
    input[i] = input[i] / sum;
  }
}

}  // namespace inference
