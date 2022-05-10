// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/method_factory.h"

#include <string>

#include "xstream/simple_method.h"
#include "xstream/user_method_factory.h"

namespace xstream {
namespace method_factory {
MethodPtr CreateMethod(const std::string &method_name) {
  return MethodPtr(MethodFactory::GetInstance()->GetMethod(method_name));
}
}  // namespace method_factory
}  // namespace xstream
