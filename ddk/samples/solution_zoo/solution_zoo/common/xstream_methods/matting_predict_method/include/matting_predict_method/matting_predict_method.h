/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: MattingPredictMethod.h
 * @Brief: declaration of the MattingPredictMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-11-25 14:07:18
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-11-25 18:21:33
 */

#ifndef INCLUDE_MATTINGPREDICTMETHOD_MATTINGPREDICTMETHOD_H_
#define INCLUDE_MATTINGPREDICTMETHOD_MATTINGPREDICTMETHOD_H_

#include <string>
#include <vector>
#include "dnn_predict_method/dnn_predict_method.hpp"
#include "bpu_predict_extension.h"

namespace xstream {

class MattingPredictMethod : public DnnPredictMethod {
 public:
  MattingPredictMethod() {}
  virtual ~MattingPredictMethod() {}

  virtual int Init(const std::string &cfg_path);

  // 派生类需要实现
  // PrepareInputData内部需要根据一帧图像目标数量，多次调用AllocInputTensor分配空间
  // 框架不进行输入与输出的Tensor分配
  // IN: input, param; OUT: input_tensors, output_tensors
  // 返回码：0，成功；否则失败；若存在申请失败，函数内部还需负责已申请空间的释放
  virtual int PrepareInputData(
      const std::vector<BaseDataPtr> &input,
      const InputParamPtr param,
      std::vector<std::vector<BPU_TENSOR_S>> &input_tensors,
      std::vector<std::vector<BPU_TENSOR_S>> &output_tensors);

 private:
  int erode_kernel_ = 33;   // 腐蚀核大小
  int dilate_kernel_ = 17;  // 膨胀核大小
};
}  // namespace xstream
#endif  // INCLUDE_MATTINGPREDICTMETHOD_MATTINGPREDICTMETHOD_H_
