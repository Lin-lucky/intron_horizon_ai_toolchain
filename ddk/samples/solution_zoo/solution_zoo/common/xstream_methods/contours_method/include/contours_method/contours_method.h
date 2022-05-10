/*
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief: contours method
 * @author : hangjun.yang
 * @email : hangjun.yang@horizon.ai
 * @date: 2021-05-15
 */

#ifndef INCLUDE_CONTOURS_METHOD_CONTOURS_METHOD_H
#define INCLUDE_CONTOURS_METHOD_CONTOURS_METHOD_H

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "xstream/simple_method.h"
#include "xstream/vision_type.h"

namespace xstream {

class ContoursMethod : public SimpleMethod {
 public:
  ContoursMethod() : SimpleMethod() {}
  virtual ~ContoursMethod() {}
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
  // the ratio = rect_size / mask_size.
  // when the ratio is bigger than contours_max_ratio_,
  // then mask will enarge to max ratio
  // to reduce the amount of calculation
  float contours_max_ratio_ = 4.0f;
};

}  // namespace xstream

#endif  // INCLUDE_CONTOURS_METHOD_CONTOURS_METHOD_H
