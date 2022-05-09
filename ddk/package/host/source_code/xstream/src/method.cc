/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Method interface of xstream framework
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.17
 */

#include "xstream/method.h"

namespace xstream {

Method::~Method() = default;

// Init from json string
int Method::InitFromJsonString(const std::string &config) { return -1; }

// Update method parameter
int Method::UpdateParameter(InputParamPtr ptr) { return -1; }

// Get method parameter
InputParamPtr Method::GetParameter() const { return InputParamPtr(); }

// Get method version
std::string Method::GetVersion() const { return "Method::GetVersion()"; }

// Get method info
MethodInfo Method::GetMethodInfo() { return MethodInfo(); }
}  // namespace xstream
