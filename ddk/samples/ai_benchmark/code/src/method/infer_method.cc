// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/infer_method.h"

#include <time.h>

#include "dnn/hb_dnn.h"
#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "utils/tensor_utils.h"
#include "utils/utils.h"

DEFINE_AND_REGISTER_METHOD(InferMethod);

int InferMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "InferMethod init from file";
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

int InferMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "InferMethod Json string:" << config.data();

  char *latency_log = getenv(LATENCY_LOG);
  latency_status_ = (latency_log != nullptr) ? atoi(latency_log) : 0;

  rapidjson::Document document;
  document.Parse(config.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }

  if (document.HasMember("core")) {
    core_ = document["core"].GetInt();
  }

  if (document.HasMember("model_file")) {
    std::string model_file = document["model_file"].GetString();
    const char *model_file_path = model_file.c_str();
    HB_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_file_path, 1),
        "hbDNNInitializeFromFiles failed");

    const char **model_name_list;
    int model_count = 0;
    HB_CHECK_SUCCESS(hbDNNGetModelNameList(
                         &model_name_list, &model_count, packed_dnn_handle_),
                     "hbDNNGetModelNameList failed");

    HB_CHECK_SUCCESS(hbDNNGetModelHandle(
                         &dnn_handle_, packed_dnn_handle_, model_name_list[0]),
                     "hbDNNGetModelHandle failed");
  } else {
    VLOG(EXAMPLE_SYSTEM) << "Please set model_file in method_config_file";
    return -1;
  }
  return 0;
}

std::vector<xstream::BaseDataPtr> InferMethod::DoProcess(
    const std::vector<xstream::BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  // prepare input tensor
  std::vector<hbDNNTensor> input_tensors;
  input_tensors.resize(input.size());
  for (uint32_t i = 0U; i < input.size(); i++) {
    auto input_tensor = std::static_pointer_cast<ImageTensor>(input[i]);
    input_tensors[i] = input_tensor->tensor;
  }

  std::vector<xstream::BaseDataPtr> res;

  // prepare output tensor
  std::shared_ptr<TensorVector> output_tensors =
      std::make_shared<TensorVector>();  // output data
  prepare_output_tensor(output_tensors->tensors, dnn_handle_);
  VLOG(EXAMPLE_DEBUG) << "prepare output tensor success";

  // Run inference
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
  infer_ctrl_param.bpuCoreId = core_;
  auto output = output_tensors->tensors.data();

  if (latency_status_) SetLatency(latency_);
  METHOD_CHECK_SUCCESS(hbDNNInfer(&task_handle,
                                  &output,
                                  input_tensors.data(),
                                  dnn_handle_,
                                  &infer_ctrl_param),
                       "hbDNNInfer failed",
                       res);
  VLOG(EXAMPLE_DEBUG) << "infer success";

  // wait task done
  METHOD_CHECK_SUCCESS(
      hbDNNWaitTaskDone(task_handle, 0), "hbDNNWaitTaskDone failed", res);
  VLOG(EXAMPLE_DEBUG) << "task done";
  if (latency_status_) UpdateLatency(latency_, "Inference  ");

  // release task done
  METHOD_CHECK_SUCCESS(
      hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed", res);
  VLOG(EXAMPLE_DEBUG) << "task release";

  output_tensors->name_ = output_name_;
  res.emplace_back(xstream::BaseDataPtr(output_tensors));
  VLOG(EXAMPLE_DETAIL) << "Predict DoProcess finished.";
  return res;
}

void InferMethod::Finalize() {}

int InferMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT) << "UpdateParameter in InferMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr InferMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "InferMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string InferMethod::GetVersion() const {}

xstream::MethodInfo InferMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "InferMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

InferMethod::~InferMethod() {}
