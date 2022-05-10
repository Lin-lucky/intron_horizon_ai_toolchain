/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: VehicleTypePostProcessMethod.h
 * @Brief: declaration of VehicleTypePostProcessMethod
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Wed Dec 23 2020 17:38:00
 */

#ifndef INCLUDE_VEHICLETYPEPOSTPROCESSMETHOD_VEHICLETYPEPOSTPROCESSMETHOD_H_
#define INCLUDE_VEHICLETYPEPOSTPROCESSMETHOD_VEHICLETYPEPOSTPROCESSMETHOD_H_

#include <memory>
#include <string>
#include <vector>
#include "dnn_async_data.h"
#include "dnn_postprocess_method/dnn_postprocess_method.hpp"
#include "bpu_parse_utils_extension.h"
#include "bpu_predict_extension.h"

namespace xstream {

class VehicleTypePostProcessMethod : public DnnPostProcessMethod {
 public:
  VehicleTypePostProcessMethod() {}
  virtual ~VehicleTypePostProcessMethod() {}

  int Init(const std::string &cfg_path) override;

  // 派生类需要实现
  // 完成模型的后处理，以及转换成Method输出格式
  // IN: dnn_result. OUT: frame_result
  int ParseDnnResult(DnnAsyncData &dnn_result,
                     std::vector<BaseDataPtr> &frame_result) override;

 private:
  void ParseVehicleType(BPU_MODEL_S bpu_model, std::vector<int> &valid_box,
                        BPU_TENSOR_S output_tensor,
                        std::shared_ptr<BaseDataVector> results);

 private:
  uint32_t output_size_;
};

}  // namespace xstream
#endif  // INCLUDE_VEHICLETYPEPOSTPROCESSMETHOD_VEHICLETYPEPOSTPROCESSMETHOD_H_
