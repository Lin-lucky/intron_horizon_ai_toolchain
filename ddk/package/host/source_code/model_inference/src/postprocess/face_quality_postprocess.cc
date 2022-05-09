/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of face_quality_postprocess
 * @file   face_quality_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/postprocess/face_quality_postprocess.h"
#include <algorithm>
#include "json/json.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
#include "model_inference/inference_engine.h"

namespace inference {

inline float SigMoid(const float input) {
  float ret = 1 / (1 + std::exp(-1 * input));
  return ret;
}

inline void SoftMax(std::vector<float> &inputs) {
  float sum = 0;
  for (auto &input : inputs) {
    sum += std::exp(input);
  }
  for (auto &input : inputs) {
    input = std::exp(input) / sum;
  }
}

int FaceQualityPostProcess::Init(const std::string &json_str)  {
  // string转json
  Json::Reader Reader;
  Json::Value config;
  Reader.parse(json_str, config);

  threshold_ = config["threshold"].isNumeric() ?
               config["threshold"].asFloat() : threshold_;
  return 0;
}

int FaceQualityPostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "FaceQualityPostProcess Execute";
  frame_result->resize(output_slot_size_);
  for (int i = 0; i < output_slot_size_; i++) {
    auto base_data_vector = std::make_shared<xstream::BaseDataVector>();
    frame_result->at(i) = base_data_vector;
  }

  int box_num = tasks.size();
  for (int dim_idx = 0; dim_idx < box_num; dim_idx++) {
    std::vector<xstream::BaseDataPtr> output;  // 一个box的结果
    auto task = tasks[dim_idx];

    FaceQualityPostPro(task->float_tensors_, &output);
    for (int slot_idx = 0; slot_idx < output_slot_size_; slot_idx++) {
      auto base_data_vector = std::static_pointer_cast<
          xstream::BaseDataVector>(frame_result->at(slot_idx));
      base_data_vector->datas_.push_back(output[slot_idx]);
    }
  }

  return 0;
}

void FaceQualityPostProcess::FaceQualityPostPro(
    const std::vector<FloatTensor> &float_tensors,
    std::vector<xstream::BaseDataPtr> *output) {
  output->resize(output_slot_size_);
  // layer 1
  for (int slot_idx = 0; slot_idx < output_slot_size_ - 1; slot_idx++) {
    auto attribute =
        std::make_shared<xstream::Attribute_<int>>();
    if (float_tensors.size() == 0) {
      // 推理失败
      attribute->value_ = -1;
      attribute->score_ = 1.0f;
      attribute->state_ = xstream::DataState::INVALID;
    } else {
      const float *mxnet_out = float_tensors[0].value.data();
      attribute->score_ = SigMoid(mxnet_out[slot_idx]);
      attribute->value_ = attribute->score_ < threshold_ ? 0 : 1;
    }
    int out_idx = slot_idx == 0 ? slot_idx : slot_idx + 1;
    (*output)[out_idx] = std::static_pointer_cast<xstream::BaseData>(attribute);
  }

  // layer 2
  auto attribute = std::make_shared<xstream::Attribute_<int>>();
  if (float_tensors.size() == 0) {
    attribute->value_ = -1;
    attribute->score_ = 1.0f;
    attribute->state_ = xstream::DataState::INVALID;
  } else {
    const float *mxnet_out = float_tensors[1].value.data();
    std::vector<float> attrs = {mxnet_out[0], mxnet_out[1], mxnet_out[2],
                                mxnet_out[3]};
    SoftMax(attrs);
    auto largest = std::max_element(std::begin(attrs), std::end(attrs));
    auto pos = std::distance(std::begin(attrs), largest);
    attribute->value_ = pos;
    attribute->score_ = attrs[pos];
  }
  (*output)[1] = std::static_pointer_cast<xstream::BaseData>(attribute);
}

}  // namespace inference
