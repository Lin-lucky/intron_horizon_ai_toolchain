/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: act_postprocess.h
 * @Brief: declaration of GesturePostProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu May 20 2021 22:03:35
 */

#ifndef GESTURE_POSTPROCESS_H_
#define GESTURE_POSTPROCESS_H_

#include <vector>
#include <memory>
#include <string>
#include "model_inference/postprocess/postprocess.h"
#include "model_inference/postprocess/utils/gesture_postprocess_util.h"

namespace inference {

class GesturePostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  void InitThresholds(std::string &thresholds_str);

  void InitMergeGroups(std::string &merge_groups_str);

  xstream::BaseDataPtr ActPostPro(std::vector<FloatTensor> &float_tensors,
                                  uint64_t timestamp, uint64_t track_id);

 private:
  int en_score_avg_;
  float threshold_;
  std::vector<float> thresholds_;
  std::vector<std::vector<int>> merge_groups_;
};

}  // namespace inference

#endif  // GESTURE_POSTPROCESS_H_
