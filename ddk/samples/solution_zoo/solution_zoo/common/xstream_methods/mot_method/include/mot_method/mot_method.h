/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     MOT Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.15
 */

#ifndef XSTREAM_METHOD_ZOO_MOTMETHOD_INCLUDE_MOTMETHOD_MOTMETHOD_H_
#define XSTREAM_METHOD_ZOO_MOTMETHOD_INCLUDE_MOTMETHOD_MOTMETHOD_H_

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "xstream/simple_method.h"

namespace xstream {

class MOTParam;

class Mot {
 public:
  virtual ~Mot() {}
  virtual int MotInit(const std::string &config_file_path) = 0;

  virtual void MotFinalize() = 0;

  virtual int Track(const std::vector<BaseDataPtr> &in,
                    std::vector<BaseDataPtr> &out) = 0;

  InputParamPtr GetParameter();

  virtual int UpdateParameter(const std::string &content) = 0;

  virtual void Reset() {}

 protected:
  std::shared_ptr<MOTParam> config_param_;
};

class MOTMethod : public SimpleMethod {
 public:
  int Init(const std::string &config_file_path) override;

  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(InputParamPtr ptr) override;

  InputParamPtr GetParameter() const override;

  std::string GetVersion() const override { return "0.0.27"; }

  MethodInfo GetMethodInfo() override {
    MethodInfo method_info;
    method_info.is_thread_safe_ = false;
    method_info.is_need_reorder_ = true;
    return method_info;
  };

 private:
  int PassThrough(const std::vector<BaseDataPtr> &in,
                  std::vector<BaseDataPtr> &out);

  std::shared_ptr<Mot> mot_;
};
}  // namespace xstream

#endif  // XSTREAM_METHOD_ZOO_MOTMETHOD_INCLUDE_MOTMETHOD_MOTMETHOD_H_
