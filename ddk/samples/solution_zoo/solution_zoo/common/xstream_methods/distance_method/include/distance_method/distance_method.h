/*
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief: distance method
 * @author : hangjun.yang
 * @email : hangjun.yang@horizon.ai
 * @date: 2021-05-15
 */

#ifndef INCLUDE_DISTANCE_METHOD_DISTANCE_METHOD_H
#define INCLUDE_DISTANCE_METHOD_DISTANCE_METHOD_H

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "xstream/simple_method.h"
#include "xstream/vision_type.h"

namespace xstream {

class DistanceMethod : public SimpleMethod {
 public:
  DistanceMethod() : SimpleMethod() {}
  virtual ~DistanceMethod() {}
  // return 0 for successed, -1 for failed.
  int Init(const std::string &config_file_path) override;

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

  void Finalize() override {}

  int UpdateParameter(InputParamPtr ptr) override { return 0; }

  InputParamPtr GetParameter() const override { return nullptr; }

  std::string GetVersion() const override { return ""; }

  MethodInfo GetMethodInfo() override {
    MethodInfo method_info;
    method_info.is_thread_safe_ = true;
    method_info.is_need_reorder_ = false;
    return method_info;
  };

 private:
  int dist_calibration_width_ = 3840;
  float dist_fit_factor_ = 61557.0;
  float dist_fit_impower_ = -1.085;
  bool dist_smooth_ = true;
};

}  // namespace xstream

#endif  // INCLUDE_DISTANCE_METHOD_DISTANCE_METHOD_H
