// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_classfication_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQClassficationPostProcessMethod);

int PTQClassficationPostProcessMethod::Init(
    const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQClassficationPostProcessMethod init from file";
  std::ifstream ifs(config_file_path.c_str());
  if (!ifs) {
    VLOG(EXAMPLE_SYSTEM) << "Open config file " << config_file_path
                         << " failed";
    return -1;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  std::string contents(buffer.str());
  return this->InitFromJsonString(contents);
}

int PTQClassficationPostProcessMethod::InitFromJsonString(
    const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQClassficationPostProcessMethod Json string:"
                      << config.data();

  char *latency_log = getenv("SHOW_LATENCY_LOG");
  latency_status_ = (latency_log != nullptr) ? atoi(latency_log) : 0;

  rapidjson::Document document;
  document.Parse(config.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }
  if (document.HasMember("top_k")) {
    top_k_ = document["top_k"].GetInt();
  }

  if (document.HasMember("cls_names_list")) {
    rapidjson::Value &cls_names_list = document["cls_names_list"];
    initClsName(cls_names_list.GetString(), class_names_);
  } else {
    VLOG(EXAMPLE_SYSTEM) << "no cls_names_list in cfg file";
    return -1;
  }
  return 0;
}

std::vector<xstream::BaseDataPtr> PTQClassficationPostProcessMethod::DoProcess(
    const std::vector<xstream::BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  auto image_tensor = std::static_pointer_cast<ImageTensor>(input[0]);
  auto output_tensor = std::static_pointer_cast<TensorVector>(input[1]);
  auto perception = std::shared_ptr<Perception>(new Perception);

  static thread_local Latency latency;
  if (latency_status_) SetLatency(latency);
  //  PostProcess
  this->PostProcess(
      &(output_tensor->tensors[0]), image_tensor.get(), perception.get());
  if (latency_status_) UpdateLatency(latency);

  VLOG(EXAMPLE_DETAIL) << "PostProcess success!";
  release_output_tensor(output_tensor->tensors);
  VLOG(EXAMPLE_DETAIL) << "release output tensor success!";

  std::vector<xstream::BaseDataPtr> res;
  perception->name_ = output_name_;
  res.emplace_back(xstream::BaseDataPtr(perception));
  VLOG(EXAMPLE_DETAIL) << "PTQClassficationPostProcessMethod DoProcess "
                          "finished, predict result: "
                       << *(perception.get());
  return res;
}

void PTQClassficationPostProcessMethod::Finalize() {}

int PTQClassficationPostProcessMethod::UpdateParameter(
    xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in PTQClassficationPostProcessMethod not supported "
         "yet.";
  return 0;
}

xstream::InputParamPtr PTQClassficationPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQClassficationPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQClassficationPostProcessMethod::GetVersion() const {}

xstream::MethodInfo PTQClassficationPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQClassficationPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQClassficationPostProcessMethod::~PTQClassficationPostProcessMethod() =
    default;

int PTQClassficationPostProcessMethod::PostProcess(hbDNNTensor *tensors,
                                                   ImageTensor *image_tensor,
                                                   Perception *perception) {
  perception->type = Perception::CLS;
  GetTopkResult(tensors, perception->cls);
  return 0;
}

void PTQClassficationPostProcessMethod::GetTopkResult(
    hbDNNTensor *tensor, std::vector<Classification> &top_k_cls) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  std::priority_queue<Classification,
                      std::vector<Classification>,
                      std::greater<Classification>>
      queue;
  int *shape = tensor->properties.validShape.dimensionSize;
  for (auto i = 0; i < shape[1] * shape[2] * shape[3]; i++) {
    float score = reinterpret_cast<float *>(tensor->sysMem[0].virAddr)[i];
    queue.push(Classification(i, score, GetClsName(i)));
    if (queue.size() > top_k_) {
      queue.pop();
    }
  }
  while (!queue.empty()) {
    top_k_cls.emplace_back(queue.top());
    queue.pop();
  }
  std::reverse(top_k_cls.begin(), top_k_cls.end());
}

const char *PTQClassficationPostProcessMethod::GetClsName(int id) {
  if (!class_names_.empty()) {
    return class_names_[id].c_str();
  }
  return nullptr;
}
