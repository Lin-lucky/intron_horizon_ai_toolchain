/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xstream framework interface
 * @file      xstream.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef HOBOTXSTREAM_XSTREAM_H_
#define HOBOTXSTREAM_XSTREAM_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "hobotxstream/method_factory.h"
#include "hobotxstream/scheduler.h"
#include "xstream/profiler.h"
#include "xstream/xstream_sdk.h"

namespace xstream {
/// XStreamFlow API
class XStreamFlow : public XStreamSDK {
 public:
  XStreamFlow();
  virtual ~XStreamFlow();

 public:
  virtual int SetConfig(const std::string &key,
                        const std::string &value);  // set config file
  int SetCallback(XStreamCallback callback,
                  const std::string &name) override;  // set callback func
  virtual int UpdateConfig(const std::string &unique_name,
                           InputParamPtr param_ptr);
  InputParamPtr GetConfig(const std::string &unique_name) const override;
  std::string GetVersion(const std::string &unique_name) const override;
  virtual int Init();
  int Init(MethodFactoryPtr method_factory);
  // SyncPredict Func for SingleOutput mode
  OutputDataPtr SyncPredict(InputDataPtr input) override;
  // SyncPredict Func for MultiOutput mode
  std::vector<OutputDataPtr> SyncPredict2(InputDataPtr input) override;
  // AsyncPredict Func
  int64_t AsyncPredict(InputDataPtr input) override;

  // Get current task_num
  int64_t GetTaskNum();

 private:
  OutputDataPtr OnError(int64_t error_code, const std::string &error_detail);

 private:
  SchedulerPtr scheduler_;
  XStreamCallback callback_;
  std::string config_file_;
  std::mutex mutex_;
  bool is_initial_;
  // param_dict_ has "config_file" and "config_string"
  std::unordered_map<std::string, std::string> param_dict_;
  ProfilerPtr profiler_;
};

}  // namespace xstream

#endif  // HOBOTXSTREAM_XSTREAM_H_
