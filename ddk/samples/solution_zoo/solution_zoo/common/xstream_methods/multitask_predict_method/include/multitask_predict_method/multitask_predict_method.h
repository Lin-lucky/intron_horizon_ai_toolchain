/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: MultitaskPredictMethod.h
 * @Brief: declaration of the MultitaskPredictMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-12-03 15:39:18
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-12-03 18:21:33
 */

#ifndef INCLUDE_MULTITASKPREDICTMETHOD_MULTITASKPREDICTMETHOD_H_
#define INCLUDE_MULTITASKPREDICTMETHOD_MULTITASKPREDICTMETHOD_H_

#include <string>
#include <vector>
#include "dnn_predict_method/dnn_predict_method.hpp"
#include "bpu_predict_extension.h"

namespace xstream {

class MultitaskPredictMethod : public DnnPredictMethod {
 public:
  MultitaskPredictMethod() {}
  virtual ~MultitaskPredictMethod() {}

  int Init(const std::string &cfg_path) override;

  // 派生类需要实现
  // PrepareInputData内部需要根据一帧图像目标数量，多次调用AllocInputTensor分配空间
  // 框架不进行输入与输出的Tensor分配
  // IN: input, param; OUT: input_tensors, output_tensors
  // 返回码：0，成功；否则失败
  int PrepareInputData(
      const std::vector<BaseDataPtr> &input,
      const InputParamPtr param,
      std::vector<std::vector<BPU_TENSOR_S>> &input_tensors,
      std::vector<std::vector<BPU_TENSOR_S>> &output_tensors) override;

  int GetSrcImageSize(
      const std::vector<BaseDataPtr> &input,
      int &src_image_height,
      int &src_image_width) override;

 private:
  int pyramid_layer_ = 0;  // 金字塔层数,默认第0层
  int model_input_width_ = 0;
  int model_input_height_ = 0;
};
}  // namespace xstream
#endif  // INCLUDE_MULTITASKPREDICTMETHOD_MULTITASKPREDICTMETHOD_H_
