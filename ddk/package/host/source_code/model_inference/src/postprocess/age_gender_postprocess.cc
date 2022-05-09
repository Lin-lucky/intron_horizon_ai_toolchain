/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of age_gender_postprocess
 * @file   age_gender_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/postprocess/age_gender_postprocess.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
#include "model_inference/inference_engine.h"

namespace inference {

int AgeGenderPostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "AgeGenderPostProcess Execute";
  HOBOT_CHECK(tasks.size() == 1);
  auto task = tasks[0];
  auto age_result = std::make_shared<xstream::BaseDataVector>();
  auto gender_result = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(age_result);
  frame_result->push_back(gender_result);  // output: age, gender

  auto roi_task =
      std::dynamic_pointer_cast<RoiInferenceEngineTask>(task);
  HOBOT_CHECK(roi_task != nullptr);

  int result_num = roi_task->roi_box_.size();

  int output_size = 2;
  int valid_result_idx = 0;

  // task failed
  if (roi_task->float_tensors_.size() <= 0) {
    for (int box_idx = 0; box_idx < result_num; box_idx++) {
      auto age = std::make_shared<xstream::Age>();
      age->state_ = xstream::DataState::INVALID;
      age_result->datas_.push_back(age);

      auto gender = std::make_shared<xstream::Gender>();
      gender->state_ = xstream::DataState::INVALID;
      gender_result->datas_.push_back(gender);
    }
    return 0;
  }

  HOBOT_CHECK(roi_task->float_tensors_.size() == 2);  // output_layer = 2

  // 对应输出层每个box_result的有效大小
  static std::vector<int> valid_offset{1, 1};
  static std::once_flag flag;
  std::call_once(flag, [&roi_task]() {
    for (int dim_idx = 0; dim_idx < 4; dim_idx++) {
      valid_offset[0] *= roi_task->float_tensors_[0].dim[dim_idx];
      valid_offset[1] *= roi_task->float_tensors_[1].dim[dim_idx];
    }
  });

  for (int box_idx = 0; box_idx < result_num; box_idx++) {
    std::vector<xstream::BaseDataPtr> output;  // 一个box的结果
    if (!roi_task->roi_box_[box_idx].resizable) {  // 无推理结果
      auto age = std::make_shared<xstream::Age>();
      age->state_ = xstream::DataState::INVALID;
      output.push_back(age);

      auto gender = std::make_shared<xstream::Gender>();
      gender->state_ = xstream::DataState::INVALID;
      output.push_back(gender);
    } else {
      // 取对应的float_tensor解析
      HandleAgeGender(
          roi_task->float_tensors_, valid_offset, valid_result_idx, &output);
      valid_result_idx++;
    }

    for (int i = 0; i < output_size; i++) {
      auto base_data_vector = std::static_pointer_cast<
          xstream::BaseDataVector>(frame_result->at(i));
      base_data_vector->datas_.push_back(output[i]);
    }
  }

  // debug
  #if 0
    auto age_vector = std::static_pointer_cast<
        xstream::BaseDataVector>(frame_result->at(0));
    for (int box_idx = 0; box_idx < age_vector->datas_.size(); box_idx++) {
      auto age = std::static_pointer_cast<xstream::Age>(
          age_vector->datas_[box_idx]);
      LOGD << *age.get();
    }
    auto gender_vector = std::static_pointer_cast<
        xstream::BaseDataVector>(frame_result->at(1));
    for (int box_idx = 0; box_idx < gender_vector->datas_.size(); box_idx++) {
      auto gender = std::static_pointer_cast<xstream::Gender>(
          gender_vector->datas_[box_idx]);
      LOGD << *gender.get();
    }
  #endif

  return 0;
}

void AgeGenderPostProcess::HandleAgeGender(
    const std::vector<FloatTensor> &float_tensors,
    const std::vector<int> &valid_offset,
    const int batch_idx,
    std::vector<xstream::BaseDataPtr> *output) {
  auto age = AgePostPro(float_tensors[0].value.data() +
      batch_idx * valid_offset[0]);
  output->push_back(age);
  auto gender = GenderPostPro(float_tensors[1].value.data() +
      batch_idx * valid_offset[1]);
  output->push_back(gender);
}

xstream::BaseDataPtr AgeGenderPostProcess::AgePostPro(
    const float* mxnet_out) {
  auto age_result = std::make_shared<xstream::Age>();
  auto &age_class = age_result->value_;
  age_class = 0;
  for (int age_i = 0; age_i < 14; age_i += 2) {
    if (mxnet_out[age_i] < mxnet_out[age_i + 1]) {
      age_class += 1;
    }
  }
  age_result->min_ = g_age_range_[age_class * 2];
  age_result->max_ = g_age_range_[age_class * 2 + 1];
  return std::static_pointer_cast<xstream::BaseData>(age_result);
}

xstream::BaseDataPtr AgeGenderPostProcess::GenderPostPro(
    const float* mxnet_out) {
  auto gender_result = std::make_shared<xstream::Gender>();
  gender_result->value_ = mxnet_out[0] > 0.0f ? 1 : -1;
  gender_result->score_ = mxnet_out[0];
  return std::static_pointer_cast<xstream::BaseData>(gender_result);
}

}  // namespace inference
