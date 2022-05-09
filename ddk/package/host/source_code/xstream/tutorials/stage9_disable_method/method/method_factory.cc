/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      method_factory.cc
 * @brief     MethodFactory class implementation
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020/01/03
 */

#include "xstream/xstream_world.h"
#include "method/bbox_filter.h"

namespace xstream {
namespace method_factory {

MethodPtr CreateMethod(const std::string &method_name) {
  if ("BBoxScoreFilter" == method_name) {
    return MethodPtr(new BBoxScoreFilter());
  } else if ("BBoxLengthFilter" == method_name) {
    return MethodPtr(new BBoxLengthFilter());
  } else if ("BBoxAreaFilter" == method_name) {
    return MethodPtr(new BBoxAreaFilter());
  } else {
    return MethodPtr();
  }
}

}  // namespace method_factory
}  // namespace xstream
