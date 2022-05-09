/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inferencer
 * @file   inferencer.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/inferencer.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

int Inferencer::Init(const std::string &cfg_path) {
  config_file_path_ = cfg_path;

  auto get_parent_path = [](const std::string path) -> std::string {
    auto pos = path.rfind('/');
    if (std::string::npos != pos) {
      auto parent = path.substr(0, pos);
      return parent + "/";
    } else {
      return std::string("./");
    }
  };
  std::string parent_path = get_parent_path(config_file_path_);
  LOGD << "Inferencer config path: " << config_file_path_;
  LOGD << "Inferencer config parent path: " << parent_path;

  int ret = 0;

  // 0、读取配置文件内容config_
  std::ifstream infile(config_file_path_);
  infile >> config_;
  // core_id_
  if (config_["model_predict"]["run_mode"]["bpu_core"].isInt()) {
    core_id_ = config_["model_predict"]["run_mode"]["bpu_core"].asInt();
  } else {
    LOGE << "please set bpu_core config";
    return -1;
  }

  // run_model_with_bbox_
  if (config_["mode"].asString() == "run_model_with_bbox") {
    run_model_with_bbox_ = true;
  } else if (config_["mode"].asString() == "run_model") {
    run_model_with_bbox_ = false;
  }

  // convert_to_float_
  if (config_["convert_to_float"].isBool()) {
    convert_to_float_ = config_["convert_to_float"].asBool();
  }

  // is_packed_model_
  if (config_["model_predict"]["is_packed_model"].isBool()) {
    is_packed_model_ = config_["model_predict"]["is_packed_model"].asBool();
  } else {
    LOGE << "please set is_packed_model config";
    return -1;
  }
  // model_path_
  if (config_["model_predict"]["model_file_path"].isString()) {
    std::string model_file_path =
        config_["model_predict"]["model_file_path"].asString();
    if (model_file_path.empty()) {
      LOGF << "please set model_file_path config";
      return -1;
    } else {
      if (model_file_path[0] == '/') {
        model_path_ = model_file_path;
      } else {
        model_path_ =
        parent_path + config_["model_predict"]["model_file_path"].asString();
      }
    }
  } else {
    LOGF << "please set model_file_path config";
    return -1;
  }
  // model_name_
  if (config_["model_predict"]["model_name"].isString()) {
    model_name_ = config_["model_predict"]["model_name"].asString();
  } else {
    LOGF << "please set model_name config";
    return -1;
  }

  // 1、根据配置生成对应的Preproc以及Postproc对象
  // "model_preprocess"、"model_post_process"
  auto preproc_name = config_["model_preprocess"]["class_name"].asString();
  preproc_ = PreProcess::GetInstance(preproc_name, this);
  if (nullptr == preproc_) {
    LOGF << "[ERROR] Not support model_preprocess-class_name: " << preproc_name;
    return -1;
  }
  ret = preproc_->Init(config_["model_preprocess"].toStyledString());
  if (ret != 0) {
    LOGF << "PreProcess Init failed";
    return -1;
  }
  auto postproc_name = config_["model_post_process"]["class_name"].asString();
  postproc_ = PostProcess::GetInstance(postproc_name);
  if (nullptr == postproc_) {
    LOGF << "[ERROR] Not Support model_post_process-class_name: "
         << postproc_name;
    return -1;
  }
  ret = postproc_->Init(config_["model_post_process"].toStyledString());
  if (ret != 0) {
    LOGF << "PostProcess Init failed";
    return -1;
  }

  // 2、加载模型文件，获取model_handle
  model_file_handle_ = InferenceEngine::GetInstance()->LoadModelFileHandle(
      model_path_.c_str(), is_packed_model_);
  // load model failed
  if (model_file_handle_ == nullptr) {
    LOGF << "Load model file failed";
    return -1;
  }
  model_handle_ = InferenceEngine::GetInstance()->LoadModelHandle(
      model_file_handle_, model_name_.c_str());
  if (model_handle_ == nullptr) {
    LOGF << "LoadModelHandle failed.";
    return -1;
  }
  // 3、 初始化模型信息
  ret = InferenceEngine::GetInstance()->GetModelInputInfo(model_handle_,
                                                          input_model_info_);
  if (ret != 0) {
    LOGF << "GetModelInputInfo failed";
    return -1;
  }
  ret = InferenceEngine::GetInstance()->GetModelOutputInfo(model_handle_,
                                                           output_model_info_);
  if (ret != 0) {
    LOGF << "GetModelOutputInfo failed";
    return -1;
  }
  return 0;
}

void Inferencer::Finalize() {
  // 释放model_handle_
  InferenceEngine::GetInstance()->ReleaseModelHandle(model_handle_);
  InferenceEngine::GetInstance()->ReleaseModelFileHandle(model_file_handle_);
}

std::vector<std::shared_ptr<InferenceEngineTask>> Inferencer::Predict(
    const std::vector<xstream::BaseDataPtr> &input) {
  // 1、推理任务
  std::vector<std::shared_ptr<InferenceEngineTask>> tasks;

  // 2、在预处理Execute构建tasks
  // 预处理,需要将数据copy或assign input_tensors_
  // 在预处理中构建InferenceEngineTask
  // 在预处理中Alloc Tensor, 设置sequence_id_
  // alloc前需要根据模型输入输出配置TensorProperties
  // tensor输入需要alloc input tensor, 金字塔以及resizer输入不需alloc
  // alloc output tensor
  preproc_->Execute(input, tasks);

  for (auto task : tasks) {
    // 3、根据配置设置task参数，core_id、model_handle_等
    task->run_ctrls_.bpu_core_id = core_id_;
    LOGD << "core_id: " << core_id_;
    task->model_handle_ = model_handle_;
    task->convert_to_float_ = convert_to_float_;
    // 4、推理任务入队
    InferenceEngine::GetInstance()->AddInferenceTask(task);
  }
  return tasks;
}

std::vector<xstream::BaseDataPtr> Inferencer::GetResult(
    std::vector<std::shared_ptr<InferenceEngineTask>> tasks) {
  std::vector<xstream::BaseDataPtr> frame_result;
  for (auto task : tasks) {
    std::future<int> infer_future = task->inference_promise_.get_future();
    int ret = infer_future.get();  // 等待推理结束
  }
  // 进行后处理
  postproc_->Execute(tasks, &frame_result);

  return frame_result;
}

}  // namespace inference

