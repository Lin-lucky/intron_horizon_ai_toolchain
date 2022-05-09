/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      b_box_filter.h
 * @brief     BBoxFilter class
 * @author    Qingpeng Liu (qingpeng.liu@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-01-03
 */

#ifndef XSTREAM_FRAMEWORK_TUTORIALS_STAGE3_METHOD_BBOX_FILTER_H_
#define XSTREAM_FRAMEWORK_TUTORIALS_STAGE3_METHOD_BBOX_FILTER_H_

#include <atomic>
#include <string>
#include <vector>
#include "xstream/xstream_world.h"
#include "method/filter_param.h"

namespace xstream {

class BBoxFilter : public SimpleMethod {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;

 private:
  std::atomic<float> area_threshold_;
};

}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_TUTORIALS_STAGE3_METHOD_BBOX_FILTER_H_
