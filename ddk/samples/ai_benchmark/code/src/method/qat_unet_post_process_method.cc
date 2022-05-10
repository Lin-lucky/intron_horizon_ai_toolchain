// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/qat_unet_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(QATUnetPostProcessMethod);

int QATUnetPostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "QATUnetPostProcessMethod init from file";
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

int QATUnetPostProcessMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "QATUnetPostProcessMethod Json string:"
                      << config.data();
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

std::vector<xstream::BaseDataPtr> QATUnetPostProcessMethod::DoProcess(
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
  VLOG(EXAMPLE_DETAIL)
      << "QATUnetPostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void QATUnetPostProcessMethod::Finalize() {}

int QATUnetPostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in QATUnetPostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr QATUnetPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "QATUnetPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string QATUnetPostProcessMethod::GetVersion() const {}

xstream::MethodInfo QATUnetPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "QATUnetPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

QATUnetPostProcessMethod::~QATUnetPostProcessMethod() = default;

int QATUnetPostProcessMethod::PostProcess(std::vector<hbDNNTensor> &tensors,
                                          ImageTensor *image_tensor,
                                          Perception *perception) {
  perception->type = Perception::SEG;
  hbSysFlushMem(&(tensors[0].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

  // get shape
  int h_index, w_index, c_index;
  get_tensor_hwc_index(&tensors[0], &h_index, &w_index, &c_index);
  int height = tensors[0].properties.validShape.dimensionSize[h_index];
  int width = tensors[0].properties.validShape.dimensionSize[w_index];

  int8_t *data = reinterpret_cast<int8_t *>(tensors[0].sysMem[0].virAddr);
  perception->seg.seg.resize(height * width);
  perception->seg.width = width;
  perception->seg.height = height;
  perception->seg.num_classes = num_classes_;

  // parsing output data
  for (int h = 0; h < height; ++h) {
    for (int w = 0; w < width; ++w) {
      // the number of channel is 1, mean is category
      perception->seg.seg[h * width + w] =
          reinterpret_cast<int8_t *>(data + (h * width + w))[0];
    }
  }
  return 0;
}
