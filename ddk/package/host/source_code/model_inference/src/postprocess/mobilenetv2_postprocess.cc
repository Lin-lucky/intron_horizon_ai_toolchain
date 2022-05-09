/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of mobilenetv2_postprocess
 * @file   mobilenetv2_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/postprocess/mobilenetv2_postprocess.h"
#include "model_inference/inference_engine.h"
#include "json/json.h"
#include <algorithm>

namespace inference {

int MobileNetV2PostProcess::Init(const std::string &json_str) {
  // string转json
  Json::Reader Reader;
  Json::Value config;
  Reader.parse(json_str, config);

  if (!config["class_names_list_path"].isString()) {
    LOGE << "not found class_names_list_path in config";
    return -1;
  }

  std::string cls_names_list = config["class_names_list_path"].asString();
  std::ifstream fi(cls_names_list);
  if (fi) {
    std::string line;
    while (std::getline(fi, line)) {
      class_names_.push_back(line);
    }
  } else {
    LOGE << "load class_names_list faild, file_path: "
         << cls_names_list;
    return -1;
  }

  return 0;
}

int MobileNetV2PostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "MobileNetV2PostProcess Execute";

  auto xstream_det_result = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(xstream_det_result);

  // 一个检测框对应一个task
  for (size_t i = 0; i < tasks.size(); i++) {
    auto task = tasks[i];

    if (task->float_tensors_.size() == 0) {  // task failed
      auto classify = std::make_shared<xstream::Classification>();
      classify->state_ = xstream::DataState::INVALID;
      xstream_det_result->datas_.push_back(classify);
      continue;
    }

    HOBOT_CHECK(task->float_tensors_.size() == 1);  // 模型仅一层输出

    xstream::Classification cls;
    int valid_shape = task->float_tensors_[0].dim[1] *
                      task->float_tensors_[0].dim[2] *
                      task->float_tensors_[0].dim[3];
    GetMaxResult(task->float_tensors_[0].value.data(), valid_shape, cls);
    LOGD << "id: " << cls.value_
         << ", category_name: " << cls.specific_type_;

    // 转换BaseData
    auto classify = std::make_shared<xstream::Classification>(cls);
    xstream_det_result->datas_.push_back(classify);
  }

  return 0;
}

void MobileNetV2PostProcess::GetMaxResult(
    const float* scores, int valid_shape,
    xstream::Classification &cls) {
  float score = 0;
  int id = 0;
  for (auto i = 0; i < valid_shape; i++) {
    if (scores[i] > score) {
      score = scores[i];
      id = i;
    }
  }
  cls.value_ = id;
  cls.score_ = score;
  if (!class_names_.empty()) {
    cls.specific_type_ = class_names_[id];
  }
}

}  // namespace inference
