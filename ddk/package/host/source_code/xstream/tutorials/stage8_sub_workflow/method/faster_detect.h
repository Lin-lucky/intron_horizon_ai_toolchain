/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    xudong.du
 * @email     xudong.du@horizon.ai
 * @version   0.0.0.1
 * @date      2020.10.29
 */

#ifndef XSTREAM_TUTORIALS_STAGE8_METHOD_FASTER_DETECT_H_
#define XSTREAM_TUTORIALS_STAGE8_METHOD_FASTER_DETECT_H_

#include <string>
#include <vector>

#include "xstream/xstream_world.h"

namespace xstream {

class FasterDetect : public Method {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<std::vector<BaseDataPtr>> DoProcess(
      const std::vector<std::vector<BaseDataPtr>> &input,
      const std::vector<xstream::InputParamPtr> &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;
};

class FasterDetectParam : public xstream::InputParam {
 public:
  explicit FasterDetectParam(const std::string &module_name)
      : xstream::InputParam(module_name) {}
  std::string Format() override { return ""; }
};

}  // namespace xstream

#endif  // XSTREAM_TUTORIALS_STAGE8_METHOD_FASTER_DETECT_H_
