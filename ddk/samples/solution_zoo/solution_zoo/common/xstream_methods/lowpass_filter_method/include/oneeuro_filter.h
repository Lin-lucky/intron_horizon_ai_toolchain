 /*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     OneEuroFilter
 * @author    ronghui.zhang
 * @email     ronghui.zhang@horizon.ai
 * @version   0.0.0.1
 * @date      2020.11.02
 */

#ifndef INCLUDE_ONEENROFILTER_H_
#define INCLUDE_ONEENROFILTER_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "xstream/method.h"
#include "lowpass_filter.h"

namespace xstream {
class OneEuroFilter {
 public:
  OneEuroFilter(double freq,
  double mincutoff = 0.3, double beta = 1.0, double dcutoff = 0.3) {
  setFrequency(freq);
  setMinCutoff(mincutoff);
  setBeta(beta);
  setDerivateCutoff(dcutoff);
  x_ = new LowPassFilter(alpha(mincutoff));
  dx_ = new LowPassFilter(alpha(dcutoff));
  last_time_ = UndefinedTime;
}

  double filter(double value, TimeStamp timestamp = UndefinedTime) {
  //  update the sampling frequency based on timestamps
  if (last_time_ != UndefinedTime && timestamp != UndefinedTime
     && timestamp != last_time_) {
    freq_ = 1.0 / (timestamp-last_time_);
  }
  last_time_ = timestamp;
  //  estimate the current variation per second
  double dvalue = x_->has_last_raw_value() ?
  (value - x_->last_raw_value()) * freq_ : 0.0;  //  FIXME: 0.0 or value?
  //  double dvalue = last_value ? (value - last_value) * freq : 0.0;
  double edvalue = dx_->FilterWithAlpha(dvalue, alpha(dcutoff_));
  // use it to update the cutoff frequency
  double cutoff = min_cutoff_ + beta_ * fabs(edvalue);
  // filter the given value
  return x_->FilterWithAlpha(value, alpha(cutoff));
}

~OneEuroFilter(void) {
  delete x_;
  delete dx_;
}

double alpha(double cutoff) {
  double te = 1.0 / freq_;
  double tau = 1.0 / (2*M_PI*cutoff);
  return 1.0 / (1.0 + tau/te);
}

void setFrequency(double freq) {
  if (freq <= 0) {
    throw std::range_error("freq should be >0");
  }
  freq_ = freq;
}

void setMinCutoff(double min_cutoff) {
  if (min_cutoff <= 0) {
    throw std::range_error("mincutoff should be >0");
  }
  min_cutoff_ = min_cutoff;
}

void setBeta(double beta) {
  beta_ = beta;
}

void setDerivateCutoff(double dcutoff) {
  if (dcutoff <= 0) {
    throw std::range_error("dcutoff should be >0");
  }
  dcutoff_ = dcutoff;
}

 private:
  double freq_;
  double min_cutoff_;
  double beta_;
  double dcutoff_;
  LowPassFilter *x_;
  LowPassFilter *dx_;
  TimeStamp last_time_;
};
}  // namespace xstream
#endif  // INCLUDE_MERGEMETHOD_MERGEMETHOD_H_
