/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      method_factory.cc
 * @brief     MethodFactory class implementation
 * @author    zhe.sun
 * @version   0.0.0.1
 * @date      2020/10/31
 */

#include "xstream/xstream_world.h"
#include "method/method.h"

namespace xstream {
namespace method_factory {

MethodPtr CreateMethod(const std::string &method_name) {
  if ("StandardDeviation" == method_name) {
    return MethodPtr(new StandardDeviation());
  } else if ("Average" == method_name) {
    return MethodPtr(new Average());
  } else {
    return MethodPtr();
  }
}

}  // namespace method_factory
}  // namespace xstream
