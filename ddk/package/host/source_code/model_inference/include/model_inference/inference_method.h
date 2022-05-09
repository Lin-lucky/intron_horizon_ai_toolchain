/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of InferMethod and PostMethod
 * @file   method.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.05.13
 */

#include "xstream/xstream_world.h"
#include "inference_task.h"
#include "inferencer.h"

namespace inference {

class InferMethod : public xstream::SimpleMethod {
 public:
  InferMethod() {
    infer_ = std::make_shared<inference::Inferencer>();
  }
  virtual ~InferMethod() {}

  virtual int Init(const std::string &cfg_path) {
    std::ifstream infile(cfg_path);
    if (!infile) {
      LOGF << "InferMethod config file: " << cfg_path << ", is not exist!!";
      return -1;
    }
    Json::Value config;
    infile >> config;
    if (config["with_postprocess"].isBool()) {
      with_postprocess_ = config["with_postprocess"].asBool();
    }
    return infer_->Init(cfg_path);
  }

  virtual void Finalize() {
    infer_->Finalize();
  }

  virtual xstream::MethodInfo GetMethodInfo() {
    xstream::MethodInfo method_info = xstream::Method::GetMethodInfo();
    method_info.is_thread_safe_ = true;
    return method_info;
  }

  std::vector<xstream::BaseDataPtr> DoProcess(
      const std::vector<xstream::BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override {
    LOGD << "InferMethod DoProcess Start";

    std::vector<std::shared_ptr<inference::InferenceEngineTask>> tasks =
        infer_->Predict(input);

    std::vector<xstream::BaseDataPtr> output;

    // 预测与后处理异步，即需要两个method
    if (!with_postprocess_) {
      // 封装task 以及infer_到std::vector<BaseDataPtr> output
      // output: size 2(infer_, tasks)
      output.push_back(infer_);

      // 封装tasks为BaseDataVectorPtr
      auto base_data_tasks = std::make_shared<xstream::BaseDataVector>();
      for (auto task : tasks) {
        base_data_tasks->datas_.push_back(task);
      }
      output.push_back(base_data_tasks);
    } else {
      // 同步
      output = infer_->GetResult(tasks);
    }

    return output;
  }

 private:
  std::shared_ptr<inference::Inferencer> infer_;
  bool with_postprocess_ = true;  // 默认预测与后处理是在同个method中
};

class PostMethod : public xstream::SimpleMethod {
 public:
  std::vector<xstream::BaseDataPtr> DoProcess(
      const std::vector<xstream::BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override {
    // 解析task和infer_
    assert(input.size() == 2);
    std::shared_ptr<inference::Inferencer> infer =
        std::static_pointer_cast<inference::Inferencer>(input[0]);
    auto base_data_tasks =
        std::static_pointer_cast<xstream::BaseDataVector>(input[1]);

    std::vector<std::shared_ptr<inference::InferenceEngineTask>> tasks;
    for (auto base_data_task : base_data_tasks->datas_) {
      auto task = std::static_pointer_cast<
          inference::InferenceEngineTask>(base_data_task);
      tasks.push_back(task);
    }
    return infer->GetResult(tasks);
  }

  virtual ~PostMethod() {}

  virtual xstream::MethodInfo GetMethodInfo() {
    xstream::MethodInfo method_info = xstream::Method::GetMethodInfo();
    method_info.is_thread_safe_ = true;
    return method_info;
  }

  int Init(const std::string &cfg_path) {
    return 0;
  }

  void Finalize() {}
};

}  // namespace inference
