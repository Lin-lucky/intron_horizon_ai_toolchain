/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: vehicle_color_predict_method.h
 * @Brief: declaration of VehicleColorPredictMethod
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Wed May 12 2021 14:52:33
 */


#ifndef INCLUDE_VEHICLECOLORPREDICTMETHOD_VEHICLECOLORPREDICTMETHOD_H_
#define INCLUDE_VEHICLECOLORPREDICTMETHOD_VEHICLECOLORPREDICTMETHOD_H_

#include <string>
#include <vector>
#include "dnn_util.h"
#include "dnn_predict_method/dnn_predict_method.hpp"
#include "bpu_predict_extension.h"

namespace xstream {

class VehicleColorPredictMethod : public DnnPredictMethod {
 public:
  VehicleColorPredictMethod() {}
  virtual ~VehicleColorPredictMethod() {}

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

 private:
  int pyramid_layer_;
  uint32_t max_handle_num_;
  NormParams norm_params_;
  FilterMode filter_method_;
};
}  // namespace xstream
#endif  // INCLUDE_VEHICLECOLORPREDICTMETHOD_VEHICLECOLORPREDICTMETHOD_H_
