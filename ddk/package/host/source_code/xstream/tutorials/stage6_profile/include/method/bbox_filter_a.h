/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      bbox_filter_a.h
 * @brief     BBoxFilterA class
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#ifndef XSTREAM_FRAMEWORK_TUTORIALS_STAGE6_INCLUDE_METHOD_BBOX_FILTER_A_H_
#define XSTREAM_FRAMEWORK_TUTORIALS_STAGE6_INCLUDE_METHOD_BBOX_FILTER_A_H_

#include <atomic>
#include <string>
#include <vector>
#include "xstream/xstream_world.h"

namespace xstream {

class BBoxFilterA : public SimpleMethod {
 public:
  int Init(const std::string &config_file_path) override;

  virtual std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param);

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;

 private:
  std::atomic<float> area_threshold_;
};

class BBoxFilterAParam : public xstream::InputParam {
 public:
  explicit BBoxFilterAParam(const std::string &module_name) :
           xstream::InputParam(module_name) {}
  std::string Format() override {
    return "";
  }
};

}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_TUTORIALS_STAGE6_INCLUDE_METHOD_BBOX_FILTER_A_H_
