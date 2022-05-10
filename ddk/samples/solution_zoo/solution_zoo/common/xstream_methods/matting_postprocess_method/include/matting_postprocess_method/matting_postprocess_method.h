/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: MattingPostProcessMethod.h
 * @Brief: declaration of the MattingPostProcessMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-11-26 13:32:18
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-11-26 15:21:32
 */

#ifndef INCLUDE_MATTINGPOSTPROCESSMETHOD_MATTINGPOSTPROCESSMETHOD_H_
#define INCLUDE_MATTINGPOSTPROCESSMETHOD_MATTINGPOSTPROCESSMETHOD_H_

#include <vector>
#include <string>
#include "dnn_async_data.h"
#include "dnn_postprocess_method/dnn_postprocess_method.hpp"

namespace xstream {

class MattingPostProcessMethod : public DnnPostProcessMethod {
 public:
  MattingPostProcessMethod() {}
  virtual ~MattingPostProcessMethod() {}

  int Init(const std::string &cfg_path) override;

  // 派生类需要实现
  // 完成模型的后处理，以及转换成Method输出格式
  // IN: dnn_result. OUT: frame_result
  virtual int ParseDnnResult(DnnAsyncData &dnn_result,
                             std::vector<BaseDataPtr> &frame_result);

 private:
  int threshold_ = 70;
};
}  // namespace xstream
#endif  // INCLUDE_MATTINGPOSTPROCESSMETHOD_MATTINGPOSTPROCESSMETHOD_H_
