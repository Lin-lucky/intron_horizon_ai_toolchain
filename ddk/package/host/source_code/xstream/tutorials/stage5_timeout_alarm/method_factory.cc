/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      method_factory.cc
 * @brief     MethodFactory class implementation
 * @author    Ronghui Zhang (ronghui.zhang@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#include "xstream/xstream_world.h"
#include "method/alarm_method.h"

namespace xstream {
namespace method_factory {

MethodPtr CreateMethod(const std::string &method_name) {
  if ("TimeoutAlarm" == method_name) {
    return MethodPtr(new TimeoutAlarm());
  } else {
    return MethodPtr();
  }
}
}  // namespace method_factory
}  // namespace xstream
