/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      method_factory.cc
 * @brief     MethodFactory class implementation
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020/01/03
 */

#include "xstream/xstream_world.h"
#include "./bbox_filter.h"
#include "./faster_detect.h"

namespace xstream {
namespace method_factory {

MethodPtr CreateMethod(const std::string &method_name) {
  if ("BBoxLocationFilter" == method_name) {
    return MethodPtr(new BBoxLocationFilter());
  } else if ("BBoxAreaFilter" == method_name) {
    return MethodPtr(new BBoxAreaFilter());
  } else if ("FasterDetect" == method_name) {
    return MethodPtr(new FasterDetect());
  } else if ("BBoxShapeFilter" == method_name) {
    return MethodPtr(new BBoxShapeFilter());
  } else {
    return MethodPtr();
  }
}

}  // namespace method_factory
}  // namespace xstream
