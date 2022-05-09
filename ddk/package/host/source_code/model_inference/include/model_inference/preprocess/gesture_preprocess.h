/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of gesture_preprocess
 * @file   gesture_preprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef GESTURE_PREPROCESS_H_
#define GESTURE_PREPROCESS_H_

#include <vector>
#include <string>
#include <memory>
#include "model_inference/preprocess/preprocess.h"
#include "model_inference/preprocess/utils/lmks_process.h"

namespace inference {

class GesturePreProcess : public PreProcess {
 public:
  explicit GesturePreProcess(Inferencer* infer) : PreProcess(infer) {}

  virtual ~GesturePreProcess() {}

  virtual int Init(const std::string &json_str);
  virtual int Execute(
      const std::vector<xstream::BaseDataPtr> &input,
      std::vector<std::shared_ptr<InferenceEngineTask>>& tasks);

 private:
  int seq_len_;
  int kps_len_;
  int buf_len_;
  int input_shift_;
  float max_gap_;
  float stride_;
  float kps_norm_scale_;
  float skip_hand_thr_ratio_;
  bool norm_kps_conf_ = false;
  LmksProcess lmks_proc_;
};

}  // namespace inference

#endif  // GESTURE_PREPROCESS_H_
