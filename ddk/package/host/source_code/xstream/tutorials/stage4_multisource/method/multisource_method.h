/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file MultiSourceMethod.h
 * @brief
 * @author xudong.du
 * @email xudong.du@horizon.ai
 * @date 2020/10/28
 */
#ifndef XSTREAM_FRAMEWORK_TUTORIAL_MULTISOURCEMETHOD_H_
#define XSTREAM_FRAMEWORK_TUTORIAL_MULTISOURCEMETHOD_H_
#include <atomic>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "xstream/xstream_world.h"

namespace xstream {

struct MultiSourceOutput : public BaseData {
  MultiSourceOutput() {}
  size_t sum_out = 0;
  size_t method_id = 0;
};
typedef std::shared_ptr<MultiSourceOutput> MultiSourceOutputPtr;

struct MultiSourceInput : public BaseData {
  MultiSourceInput() {}
  size_t sum_in = 0;
};
typedef std::shared_ptr<MultiSourceInput> MulSrcTestInputPtr;

class MultiSourceMethod : public SimpleMethod {
 public:
  MultiSourceMethod() {
    method_id_ = instance_count_++;
    sum_ = 0;
  }
  // 初始化
  int Init(const std::string &config_file_path) override { return 0; }

  int UpdateParameter(InputParamPtr ptr) override { return 0; }

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const InputParamPtr &param) override;

  // 获取Method运行参数配置
  InputParamPtr GetParameter() const override { return InputParamPtr(); }
  // 获取版本号
  std::string GetVersion() const override { return ""; }
  // 析构
  void Finalize() override{};
  // 获取Method基本信息
  MethodInfo GetMethodInfo() override { return methodinfo_; }

  static void SetMethodInfo(const MethodInfo &methodinfo);

 private:
  static std::atomic_ulong instance_count_;
  static MethodInfo methodinfo_;
  int32_t method_id_;
  size_t sum_;
};

}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_TUTORIAL_MULTISOURCEMETHOD_H_
