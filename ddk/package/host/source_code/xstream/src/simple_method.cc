/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     SimpleMethod interface of xstream framework
 * @author    xudong.du
 * @email     xudong.du@horizon.ai
 * @version   0.0.0.1
 * @date      2021.02.03
 */

#include "xstream/simple_method.h"

namespace xstream {

SimpleMethod::~SimpleMethod() = default;

std::vector<std::vector<BaseDataPtr>> SimpleMethod::DoProcess(
    const std::vector<std::vector<BaseDataPtr>> &input,
    const std::vector<InputParamPtr> &param) {
  std::vector<std::vector<BaseDataPtr>> output;
  // input size > 0 -> many frames， batch mode
  for (size_t i = 0; i < input.size(); ++i) {
    const auto &frame_input = input[i];
    const auto &frame_param = param[i];
    auto frame_output = DoProcess(frame_input, frame_param);
    output.push_back(frame_output);
  }
  return output;
}
// Init from json string
int SimpleMethod::InitFromJsonString(const std::string &config) { return -1; }

// Update SimpleMethod parameter
int SimpleMethod::UpdateParameter(InputParamPtr ptr) { return -1; }

// Get SimpleMethod parameter
InputParamPtr SimpleMethod::GetParameter() const { return InputParamPtr(); }

// Get SimpleMethod version
std::string SimpleMethod::GetVersion() const {
  return "SimpleMethod::GetVersion()";
}

// Get SimpleMethod info
MethodInfo SimpleMethod::GetMethodInfo() { return MethodInfo(); }
}  // namespace xstream
