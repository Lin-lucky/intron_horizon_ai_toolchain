// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_unet_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQUnetPostProcessMethod);

int PTQUnetPostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQUnetPostProcessMethod init from file";
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

int PTQUnetPostProcessMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQUnetPostProcessMethod Json string:"
                      << config.data();

  char *latency_log = getenv("SHOW_LATENCY_LOG");
  latency_status_ = (latency_log != nullptr) ? atoi(latency_log) : 0;

  rapidjson::Document document;
  document.Parse(config.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }

  if (document.HasMember("num_classes")) {
    num_classes_ = document["num_classes"].GetInt();
  }

  return 0;
}

std::vector<xstream::BaseDataPtr> PTQUnetPostProcessMethod::DoProcess(
    const std::vector<xstream::BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  auto image_tensor = std::static_pointer_cast<ImageTensor>(input[0]);
  auto output_tensor = std::static_pointer_cast<TensorVector>(input[1]);
  auto perception = std::shared_ptr<Perception>(new Perception);

  static thread_local Latency latency;
  if (latency_status_) SetLatency(latency);
  //  PostProcess
  this->PostProcess(
      output_tensor->tensors, image_tensor.get(), perception.get());
  if (latency_status_) UpdateLatency(latency);

  VLOG(EXAMPLE_DETAIL) << "PostProcess success!";
  release_output_tensor(output_tensor->tensors);
  VLOG(EXAMPLE_DETAIL) << "release output tensor success!";

  std::vector<xstream::BaseDataPtr> res;
  perception->name_ = output_name_;
  res.emplace_back(xstream::BaseDataPtr(perception));
  VLOG(EXAMPLE_DETAIL)
      << "PTQUnetPostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void PTQUnetPostProcessMethod::Finalize() {}

int PTQUnetPostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in PTQUnetPostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr PTQUnetPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQUnetPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQUnetPostProcessMethod::GetVersion() const {}

xstream::MethodInfo PTQUnetPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQUnetPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQUnetPostProcessMethod::~PTQUnetPostProcessMethod() = default;

int PTQUnetPostProcessMethod::PostProcess(std::vector<hbDNNTensor> &tensors,
                                          ImageTensor *image_tensor,
                                          Perception *perception) {
  perception->type = Perception::SEG;
  hbSysFlushMem(&(tensors[0].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

  // get shape
  int h_index, w_index, c_index;
  get_tensor_hwc_index(&tensors[0], &h_index, &w_index, &c_index);
  int height = tensors[0].properties.validShape.dimensionSize[h_index];
  int width = tensors[0].properties.validShape.dimensionSize[w_index];
  int channel = tensors[0].properties.validShape.dimensionSize[c_index];

  float *data = reinterpret_cast<float *>(tensors[0].sysMem[0].virAddr);
  perception->seg.seg.resize(height * width);
  perception->seg.width = width;
  perception->seg.height = height;
  perception->seg.num_classes = num_classes_;

  // argmax, operate in NHWC format
  for (int h = 0; h < height; ++h) {
    for (int w = 0; w < width; ++w) {
      float top_score = -1000000.0f;
      int top_index = 0;
      float *c_data = data + (width * h + w) * channel;
      for (int c = 0; c < channel; c++) {
        if (c_data[c] > top_score) {
          top_score = c_data[c];
          top_index = c;
        }
      }
      perception->seg.seg[h * width + w] = top_index;
    }
  }
  return 0;
}
