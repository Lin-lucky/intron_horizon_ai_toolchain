/**
 * @copyright Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file      alarm_method.h
 * @brief     Alarm_Method class
 * @author    Ronghui Zhang (ronghui.zhang@horizon.ai)
 * @version   0.0.0.1
 * @date      2020-10-26
 */

#ifndef XSTREAM_TUTORIALS_STAGE5_TIMEOUT_ALARM_H_
#define XSTREAM_TUTORIALS_STAGE5_TIMEOUT_ALARM_H_

#include <atomic>
#include <string>
#include <vector>
#include "xstream/xstream_world.h"

namespace xstream {

class TimeoutAlarm : public SimpleMethod {
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
  int timeout_;
};
}  // namespace xstream
#endif
