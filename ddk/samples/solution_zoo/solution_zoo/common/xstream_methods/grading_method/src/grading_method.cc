/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Grading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.25
 */

#include "grading_method/grading_method.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include "grading_method/error_code.h"
#include "grading_method/weight_grading.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

namespace xstream {

int GradingMethod::Init(const std::string &config_file_path) {
  LOGI << "GradingMethod::Init " << config_file_path << std::endl;
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGI << "SnapShotParam: no config, using default parameters" << std::endl;
  } else {
    Json::Value config_jv;
    config_if >> config_jv;
    std::string grading_type;
    if (config_jv.isMember("grading_type") &&
        config_jv["grading_type"].isString())
      grading_type = config_jv["grading_type"].asString();
    if (grading_type != "weight_grading") {
      LOGE << "config param error";
      return XSTREAM_GRADING_ERR_PARAM;
    }
  }
  grading_ = std::make_shared<WeightGrading>();
  grading_->GradingInit(config_file_path);

  return XSTREAM_GRADING_OK;
}

std::vector<BaseDataPtr> GradingMethod::DoProcess(
    const std::vector<BaseDataPtr> &input, const InputParamPtr &param) {
  LOGD << "GradingMethod::DoProcess" << std::endl;
  std::vector<BaseDataPtr> output;
  grading_->ProcessFrame(input, param, &output);
  return output;
}

int GradingMethod::UpdateParameter(InputParamPtr ptr) {
  if (ptr->is_json_format_) {
    std::string content = ptr->Format();
    return grading_->UpdateParameter(content);
  } else {
    HOBOT_CHECK(0) << "only support json format config";
    return XSTREAM_GRADING_ERR_PARAM;
  }
}

InputParamPtr GradingMethod::GetParameter() const {
  return grading_->GetParameter();
}

std::string GradingMethod::GetVersion() const { return "0.0.12"; }

void GradingMethod::Finalize() {
  grading_->GradingFinalize();
  LOGI << "GradingMethod::GFinalize" << std::endl;
}

int GradingParam::UpdateParameter(const std::string &content) {
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  JSONCPP_STRING error;
  std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
  try {
    bool ret = json_reader->parse(
        content.c_str(), content.c_str() + content.size(), &config_jv, &error);
    SET_GRADING_METHOD_PARAM(config_jv, String, grading_type);
    if (ret) {
      return XSTREAM_GRADING_OK;
    } else {
      return XSTREAM_GRADING_ERR_PARAM;
    }
  } catch (std::exception &e) {
    return XSTREAM_GRADING_ERR_PARAM;
  }
}

std::string GradingParam::Format() { return config_jv.toStyledString(); }
}  // namespace xstream
