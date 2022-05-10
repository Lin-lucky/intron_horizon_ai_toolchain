/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: BinaryClassifyPostPredictor.cpp
 * @Brief: definition of the BinaryClassifyPostPredictor
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-05-16 14:27:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-05-16 14:27:05
 */

#include <memory>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/post_predictor/binary_classify_post_predictor.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

namespace xstream {

void BinaryClassifyPostPredictor::Do(CNNMethodRunData *run_data) {
  {
    int dim_size = run_data->input_dim_size;
    auto &mxnet_output = run_data->mxnet_output;
    std::vector<BaseDataPtr> &batch_output = run_data->output;
    batch_output.resize(output_slot_size_);
    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }
    {
      RUN_FPS_PROFILER(model_name_ + "_post");
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");

      auto boxes = std::static_pointer_cast<BaseDataVector>(
          (*(run_data->input))[0]);

      for (int dim_idx = 0; dim_idx < dim_size; dim_idx++) {
        std::vector<BaseDataPtr> output;
        HandleResult(mxnet_output[dim_idx], &output);

        for (int i = 0; i < output_slot_size_; i++) {
          auto base_data_vector =
              std::static_pointer_cast<BaseDataVector>(batch_output[i]);
          base_data_vector->datas_.push_back(output[i]);
        }
      }
    }
  }
}

void BinaryClassifyPostPredictor::HandleResult(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    std::vector<BaseDataPtr> *output) {
  if (mxnet_outs.size()) {
    auto attribute = PostPro(mxnet_outs[0]);
    output->push_back(attribute);
  } else {
    auto attribute = std::make_shared<xstream::Attribute_<int32_t>>();
    attribute->state_ = DataState::INVALID;
    output->push_back(std::static_pointer_cast<BaseData>(attribute));
  }
}

BaseDataPtr BinaryClassifyPostPredictor::PostPro(
    const std::vector<int8_t> &mxnet_outs) {
  auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs.data());
  auto attribute_result = std::make_shared<xstream::Attribute_<int32_t>>();
  attribute_result->value_ = mxnet_out[0] > 0.0f ? 1 : 0;
  attribute_result->score_ =
      mxnet_out[0] > 0.0f ? mxnet_out[0] : -1 * mxnet_out[0];
  LOGD << "attribute: " << attribute_result->value_;
  LOGD << "score: " << attribute_result->score_;
  return std::static_pointer_cast<BaseData>(attribute_result);
}

}  // namespace xstream
