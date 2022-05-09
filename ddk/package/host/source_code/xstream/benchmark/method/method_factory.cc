/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      method_factory.cc
 * @brief     MethodFactory class implementation
 * @author    zhe.sun
 * @version   0.0.0.1
 * @date      2020/11/5
 */

#include "xstream/xstream_world.h"
#include "method/passthrough_method.h"

namespace xstream {
namespace method_factory {

MethodPtr CreateMethod(const std::string &method_name) {
  if ("PassThrough" == method_name) {
    return MethodPtr(new PassThrough());
  } else {
    return MethodPtr();
  }
}

}  // namespace method_factory
}  // namespace xstream
