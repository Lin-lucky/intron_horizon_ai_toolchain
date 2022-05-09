/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     PassThrough Method
 * @author    zhe.sun
 * @version   0.0.0.1
 * @date      2020.11.5
 */

#ifndef XSTREAM_FRAMEWORK_BENCHMARK_METHOD_PASSTHROUGH_METHOD_H_
#define XSTREAM_FRAMEWORK_BENCHMARK_METHOD_PASSTHROUGH_METHOD_H_

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include "xstream/method.h"

namespace xstream {

class PassThrough : public Method {
 public:
  int Init(const std::string &config_file_path) override {
    std::cout << "PassThrough Init" << std::endl;
    return 0;
  }

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<xstream::InputParamPtr> &param) override {
    return input;
  }

  void Finalize() override {
    std::cout << "PassThrough Finalize" << std::endl;
  }

  int UpdateParameter(InputParamPtr ptr) override {
    return 0;
  }

  InputParamPtr GetParameter() const override {
    return InputParamPtr();
  }

  std::string GetVersion() const override {
    return "";
  }
};

}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_BENCHMARK_METHOD_PASSTHROUGH_METHOD_H_
