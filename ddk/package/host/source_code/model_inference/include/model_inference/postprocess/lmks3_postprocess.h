/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: lmks3_postprocess.h
 * @Brief: declaration of Lmks3PostProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Wed May 19 2021 02:36:49
 */

#ifndef LMKS3_POSTPROCESS_H_
#define LMKS3_POSTPROCESS_H_

#include <memory>
#include <string>
#include <vector>
#include "model_inference/postprocess/postprocess.h"
#include "xstream/vision_type.h"

namespace inference {

class Lmks3PostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  void LmksPostPro(const std::vector<FloatTensor> &float_tensors,
                   const int valid_offset, const int valid_result_idx,
                   const InferBBox roi,
                   std::vector<xstream::BaseDataPtr> *output);

 private:
  int i_o_stride_;
};

}  // namespace inference

#endif  // LMKS3_POSTPROCESS_H_
