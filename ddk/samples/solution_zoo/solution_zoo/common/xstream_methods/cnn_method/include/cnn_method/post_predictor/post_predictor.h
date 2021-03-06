/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: PostPredictor.h
 * @Brief: declaration of the PostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 15:13:07
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTOR_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTOR_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cnn_method/util/cnn_method_data.h"
#include "cnn_method/util/cnn_method_config.h"
#include "xstream/vision_type.h"

namespace xstream {

class PostPredictor {
 public:
  PostPredictor() {}
  virtual ~PostPredictor() {}

  virtual int32_t Init(std::shared_ptr<CNNMethodConfig> config);
  virtual void Finalize() {}
  virtual void Do(CNNMethodRunData *run_data) = 0;
  virtual void UpdateParam(std::shared_ptr<CNNMethodConfig> config) {}

 protected:
  std::string model_name_;  // just for log
  int output_slot_size_ = 0;
};
}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTOR_H_
