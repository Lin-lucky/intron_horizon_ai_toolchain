/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: lmks4_postprocess.h
 * @Brief: decalration of Lmks4PostProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Tue May 18 2021 02:13:03
 */

#ifndef LMKS4_POSTPROCESS_H_
#define LMKS4_POSTPROCESS_H_

#include <memory>
#include <string>
#include <vector>
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"
#include "xstream/vision_type.h"

namespace inference {

class Lmks4PostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  void HandleLmks(const std::vector<FloatTensor> &float_tensors,
                  const std::vector<int> &valid_offset,
                  const int valid_result_idx, const InferBBox roi,
                  std::vector<xstream::BaseDataPtr> *output);

  void LmksPostPro(const float *mxnet_out, const InferBBox roi, const int axis,
                   std::shared_ptr<xstream::Landmarks> lmks);

 private:
  int i_o_stride_;
  int lmks_num_;
  int vector_size_;
};

}  // namespace inference

#endif  // LMKS4_POSTPROCESS_H_
