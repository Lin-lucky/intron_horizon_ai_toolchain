/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of plate_num_postprocess
 * @file   plate_num_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.20
 */

#include "model_inference/postprocess/plate_num_postprocess.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"
#include "hobotlog/hobotlog.hpp"
#include "model_inference/inference_engine.h"

namespace inference {

int PlateNumPostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "PlateNumPostProcess Execute";
  // currently only support single row
  HOBOT_CHECK(tasks.size() == 1);
  auto task = tasks[0];
  auto plate_num_result = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(plate_num_result);  // output: plate_num

  auto roi_task =
      std::dynamic_pointer_cast<RoiInferenceEngineTask>(task);
  HOBOT_CHECK(roi_task != nullptr);

  int result_num = roi_task->roi_box_.size();
  int valid_result_idx = 0;

  // task failed
  if (roi_task->float_tensors_.size() <= 0) {
    for (int box_idx = 0; box_idx < result_num; box_idx++) {
      auto plate_num = std::make_shared<xstream::DataArray_<int>>();
      plate_num->state_ = xstream::DataState::INVALID;
      plate_num_result->datas_.push_back(plate_num);
    }
    return 0;
  }

  HOBOT_CHECK(roi_task->float_tensors_.size() == 1);  // output_layer = 1

  // 对应输出层每个box_result的有效大小
  static int valid_offset = 1;
  static std::once_flag flag;
  std::call_once(flag, [&roi_task]() {
    for (int dim_idx = 0; dim_idx < 4; dim_idx++) {
      valid_offset *= roi_task->float_tensors_[0].dim[dim_idx];
    }
  });

  for (int box_idx = 0; box_idx < result_num; box_idx++) {
    // 一个box的结果
    auto plate_num = std::make_shared<xstream::DataArray_<int>>();

    if (!roi_task->roi_box_[box_idx].resizable) {  // 无推理结果
      plate_num->state_ = xstream::DataState::INVALID;
    } else {
      // 取对应的float_tensor解析
      ParsePlateNum(&roi_task->float_tensors_[0],
                    valid_result_idx, valid_offset, plate_num);
      valid_result_idx++;
    }

    plate_num_result->datas_.push_back(plate_num);
  }

  return 0;
}

void PlateNumPostProcess::ParsePlateNum(
    const FloatTensor* float_tensor,
    const int valid_idx, const int valid_offset,
    std::shared_ptr<xstream::DataArray_<int>> plate_num) {
  int slice_num = float_tensor->dim[2];
  int score_num = float_tensor->dim[3];

  std::vector<int> nums;

  // get max index for each slice
  for (int slice_idx = 0; slice_idx < slice_num; ++slice_idx) {
    int max_index = -1;
    float max_score = -999.0f;

    for (int score_idx = 0; score_idx < score_num; ++score_idx) {
      int idx = slice_idx * score_num + score_idx;
      float cur_score = (float_tensor->value.data() +
                         valid_offset * valid_idx)[idx];
      if (cur_score > max_score) {
        max_index = score_idx;
        max_score = cur_score;
      }
    }
    nums.push_back(max_index);
  }

  // skip splitted/repeated characters
  int last_index = -1;
  for (auto iter = nums.begin(); iter != nums.end();) {
    if (*iter == last_index || *iter == 0) {
      last_index = *iter;
      iter = nums.erase(iter);
    } else {
      last_index = *iter;
      iter++;
    }
  }

  plate_num->values_.assign(nums.begin(), nums.end());

  // plate num can only be 7 or 8 digits
  if (plate_num->values_.size() != 7 && plate_num->values_.size() != 8) {
    plate_num->values_.clear();
  }

#ifdef DEBUG
  LOGW << "plate num: " << Convert2String(plate_num->values_);
#endif
}

std::string PlateNumPostProcess::Convert2String(
    std::vector<int> numerical_plate_num) {
  static std::vector<std::string> plate_num_map{
      " ",  "A",  "B",  "C",  "D",  "E",  "F",  "G",  "H",  "J",  "K",
      "L",  "M",  "N",  "P",  "Q",  "R",  "S",  "T",  "U",  "V",  "W",
      "X",  "Y",  "Z",  "0",  "1",  "2",  "3",  "4",  "5",  "6",  "7",
      "8",  "9",  "京", "津", "沪", "渝", "黑", "吉", "辽", "冀", "晋",
      "鲁", "豫", "陕", "甘", "青", "苏", "浙", "皖", "鄂", "湘", "闽",
      "赣", "川", "贵", "云", "粤", "琼", "蒙", "宁", "新", "桂", "藏",
      "学", "警", "领", "军", "使", "港", "挂", "澳"};
  std::string alphabetic_plate_num = "";
  for (auto index : numerical_plate_num) {
    alphabetic_plate_num += plate_num_map[index];
  }
  LOGW << alphabetic_plate_num;
  return alphabetic_plate_num;
}

}  // namespace inference
