/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethod.h
 * @Brief: declaration of the CNNMethod
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:01:54
 */

#ifndef INCLUDE_CNNMETHOD_CNNMETHOD_H_
#define INCLUDE_CNNMETHOD_CNNMETHOD_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "xstream/simple_method.h"

namespace xstream {
class PostPredictor;
class Predictor;
class CNNMethodConfig;

class CNNMethod : public SimpleMethod {
 public:
  CNNMethod() {}
  virtual ~CNNMethod() {}

  virtual int Init(const std::string &cfg_path);
  virtual void Finalize();

  virtual std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param);
  virtual int UpdateParameter(xstream::InputParamPtr ptr);
  virtual InputParamPtr GetParameter() const;
  virtual std::string GetVersion() const;

 private:
  std::shared_ptr<Predictor> predictor_;
  std::shared_ptr<PostPredictor> post_predict_;

  std::shared_ptr<CNNMethodConfig> config_;
};
}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_CNNMETHOD_H_
