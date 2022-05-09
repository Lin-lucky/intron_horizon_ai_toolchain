/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      method_factory.cc
 * @brief     MethodFactory class definition
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#include "xstream/xstream_world.h"
#include "method/bbox_filter_a.h"
#include "method/bbox_filter_b.h"

namespace xstream {
namespace method_factory {

MethodPtr CreateMethod(const std::string &method_name) {
  if (method_name == "BBoxFilterA") {
    return MethodPtr(new BBoxFilterA());
  } else if (method_name == "BBoxFilterB") {
    return MethodPtr(new BBoxFilterB());
  } else {
    return MethodPtr();
  }
}

}  // namespace method_factory
}  // namespace xstream
