 /*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     LowPassFilter
 * @author    ronghui.zhang
 * @email     ronghui.zhang@horizon.ai
 * @version   0.0.0.1
 * @date      2020.11.02
 */

#ifndef INCLUDE_LOWPASSFILTER_H_
#define INCLUDE_LOWPASSFILTER_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include "xstream/method.h"

namespace xstream {
// -----------------------------------------------------------------
typedef double TimeStamp;  // in seconds

static const TimeStamp UndefinedTime = -1.0;
// -----------------------------------------------------------------
class LowPassFilter {
 public:
  explicit LowPassFilter(double alpha, double initval = 0.0) {
    last_value_ = initval;
    last_result_ = initval;
    set_alpha(alpha);
    initialized_ = false;
  }

  double Filter(double value) {
    double result;
    if (initialized_) {
      result = alpha_ * value + (1.0 - alpha_) * last_result_;
    } else {
      result = value;
      initialized_ = true;
    }
    last_value_ = value;
    last_result_ = result;
    return result;
  }

  double FilterWithAlpha(double value, double alpha) {
    set_alpha(alpha);
    return Filter(value);
  }

  bool has_last_raw_value() {
    return initialized_;
  }

  double last_raw_value() {
    return last_value_;
  }

 private:
  double last_value_;
  double alpha_;
  double last_result_;
  bool initialized_;

  void set_alpha(double alpha) {
    if (alpha <= 0.0 || alpha > 1.0) {
      throw std::range_error("alpha should be in (0.0., 1.0]");
    }
    alpha_ = alpha;
  }
};
}  // namespace xstream

#endif  // INCLUDE_LOWPASSFILTER_H_
