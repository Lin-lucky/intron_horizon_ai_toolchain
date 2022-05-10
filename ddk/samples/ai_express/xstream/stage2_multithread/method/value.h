/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     definition of float value
 * @author    zhe.sun
 * @version   0.0.0.1
 * @date      2020.10.31
 */

#ifndef XSTREAM_FRAMEWORK_TUTORIALS_STAGE2_METHOD_VALUE_H_
#define XSTREAM_FRAMEWORK_TUTORIALS_STAGE2_METHOD_VALUE_H_

#include <memory>
#include "xstream/xstream_world.h"
namespace xstream {
struct FloatValue : public BaseData {
  FloatValue() {}
  explicit FloatValue(float x) {
    value_ = x;
  }

  float value_ = 0;
};

typedef std::shared_ptr<FloatValue> FloatValuePtr;
}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_TUTORIALS_STAGE2_METHOD_VALUE_H_
