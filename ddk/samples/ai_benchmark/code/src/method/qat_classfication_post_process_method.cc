// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/qat_classfication_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(QATClassficationPostProcessMethod);

int QATClassficationPostProcessMethod::Init(
    const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "QATClassficationPostProcessMethod init from file";
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

int QATClassficationPostProcessMethod::InitFromJsonString(
    const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "QATClassficationPostProcessMethod Json string:"
                      << config.data();
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

std::vector<xstream::BaseDataPtr> QATClassficationPostProcessMethod::DoProcess(
    const std::vector<xstream::BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  auto image_tensor = std::static_pointer_cast<ImageTensor>(input[0]);
  auto output_tensor = std::static_pointer_cast<TensorVector>(input[1]);
  auto perception = std::shared_ptr<Perception>(new Perception);
  //  PostProcess
  this->PostProcess(
      output_tensor->tensors, image_tensor.get(), perception.get());
  VLOG(EXAMPLE_DETAIL) << "PostProcess success!";
  release_output_tensor(output_tensor->tensors);
  VLOG(EXAMPLE_DETAIL) << "release output tensor success!";

  std::vector<xstream::BaseDataPtr> res;
  perception->name_ = output_name_;
  res.emplace_back(xstream::BaseDataPtr(perception));
  VLOG(EXAMPLE_DETAIL) << "QATClassficationPostProcessMethod DoProcess "
                          "finished, predict result: "
                       << *(perception.get());
  return res;
}

void QATClassficationPostProcessMethod::Finalize() {}

int QATClassficationPostProcessMethod::UpdateParameter(
    xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in QATClassficationPostProcessMethod not supported "
         "yet.";
  return 0;
}

xstream::InputParamPtr QATClassficationPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "QATClassficationPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string QATClassficationPostProcessMethod::GetVersion() const {}

xstream::MethodInfo QATClassficationPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "QATClassficationPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = true;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

QATClassficationPostProcessMethod::~QATClassficationPostProcessMethod() {}

int QATClassficationPostProcessMethod::PostProcess(
    std::vector<hbDNNTensor> &tensors,
    ImageTensor *image_tensor,
    Perception *perception) {
  perception->type = Perception::CLS;
  GetTopkResult(&tensors[0], perception->cls);
  return 0;
}

void QATClassficationPostProcessMethod::GetTopkResult(
    hbDNNTensor *tensor, std::vector<Classification> &top_k_cls) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  std::priority_queue<Classification,
                      std::vector<Classification>,
                      std::greater<Classification>>
      queue;
  int *shape = tensor->properties.validShape.dimensionSize;
  auto properties = tensor->properties;
  auto aligned_shape = properties.alignedShape.dimensionSize;
  int offset = 1;
  if (properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    offset = aligned_shape[2] * aligned_shape[3];
  }
  for (auto i = 0; i < shape[1] * shape[2] * shape[3]; i++) {
    float score;
    if (properties.quantiType == SHIFT) {
      score = quanti_shift(
          reinterpret_cast<int32_t *>(tensor->sysMem[0].virAddr)[i * offset],
          properties.shift.shiftData[i]);
    } else if (properties.quantiType == SCALE) {
      score = quanti_scale(
          reinterpret_cast<int32_t *>(tensor->sysMem[0].virAddr)[i * offset],
          properties.scale.scaleData[i]);
    } else {
      VLOG(EXAMPLE_SYSTEM) << "quanti type error!";
    }
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

const char *QATClassficationPostProcessMethod::GetClsName(int id) {
  if (!class_names_.empty()) {
    return class_names_[id].c_str();
  }
  return nullptr;
}
