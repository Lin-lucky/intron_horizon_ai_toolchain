/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    wenhao.zou
 * @email     wenhao.zou@horizon.ai
 * @version   0.0.0.1
 * @date      2020.2.12
 */

#ifndef XSTREAM_TUTORIALS_STAGE4_METHOD_BBOX_FILTER_H_
#define XSTREAM_TUTORIALS_STAGE4_METHOD_BBOX_FILTER_H_

#include <string>
#include <vector>
#include "xstream/xstream_world.h"

namespace xstream {

class BBoxFilter : public SimpleMethod {
 public:
  int Init(const std::string &config_file_path) override;

  virtual std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) = 0;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;
};

class BBoxScoreFilter : public BBoxFilter {
 public:
  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

 private:
  float score_threshold_ = 0.5;
};

class BBoxLengthFilter : public BBoxFilter {
 public:
  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

 private:
  float length_threshold_ = 5;
};

class BBoxAreaFilter : public BBoxFilter {
 public:
  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

 private:
  float area_threshold_ = 50;
};

class BBoxFilterParam : public xstream::InputParam {
 public:
  explicit BBoxFilterParam(const std::string &module_name) :
           xstream::InputParam(module_name) {}
  std::string Format() override {
    return "";
  }
};

}  // namespace xstream

#endif  // XSTREAM_TUTORIALS_STAGE4_METHOD_BBOX_FILTER_H_
