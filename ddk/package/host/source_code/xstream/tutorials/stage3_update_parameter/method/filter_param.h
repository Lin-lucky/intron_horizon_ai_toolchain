/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      filter_param.h
 * @brief     FilterParam class
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#ifndef XSTREAM_FRAMEWORK_TUTORIALS_STAGE3_METHOD_FILTER_PARAM_H_
#define XSTREAM_FRAMEWORK_TUTORIALS_STAGE3_METHOD_FILTER_PARAM_H_

#include <string>
#include <memory>
#include <utility>
#include "xstream/xstream_world.h"
#include "json/json.h"

namespace xstream {

typedef struct _FilterParam__isset {
  _FilterParam__isset() : threshold(false) {}

  bool threshold : 1;
} _FilterParam__isset;

class FilterParam : public InputParam {
 public:
  explicit FilterParam(std::string method_name) : InputParam(method_name) {
    threshold_ = 2500.0;
  }
  FilterParam(const std::string &method_name,
              const std::string &json_content) : InputParam(method_name) {
    Json::CharReaderBuilder builder;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    json_reader->parse(json_content.c_str(),
                       json_content.c_str() + json_content.size(),
                       &cfg_jv_, &error);
  }
  virtual ~FilterParam() = default;

  virtual std::string Format() {
    return std::move(cfg_jv_.toStyledString());
  }

  void SetThreshold(float thres) {
    threshold_ = thres;
    is_set_.threshold = true;
  }

  bool HasThreshold() { return is_set_.threshold; }

  float GetThreshold() { return threshold_; }

 private:
  _FilterParam__isset is_set_;
  float threshold_;
  Json::Value cfg_jv_;
};

}  // namespace xstream
#endif  // XSTREAM_FRAMEWORK_TUTORIALS_STAGE3_METHOD_FILTER_PARAM_H_
