/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethod.cpp
 * @Brief: definition of the CNNMethod
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:17:08
 */

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "./plat_cnn.h"
#include "cnn_method/cnn_const.h"
#include "cnn_method/cnn_method.h"
#include "cnn_method/post_predictor/post_predictor.h"
#include "cnn_method/post_predictor/post_predictor_factory.h"
#include "cnn_method/predictor/predictor.h"
#include "cnn_method/predictor/predictor_factory.h"
#include "cnn_method/util/cnn_method_config.h"
#include "cnn_method/util/cnn_method_data.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/vision_type.h"

namespace xstream {

int32_t CNNMethod::Init(const std::string &cfg_path) {
  LOGI << "CNNMethod Init";
  std::ifstream infile(cfg_path.c_str());
  HOBOT_CHECK(infile.good()) << "CNNMethod error config file path:" << cfg_path;

  std::stringstream buffer;
  buffer << infile.rdbuf();
  config_.reset(new CNNMethodConfig(buffer.str()));
  config_->SetSTDStringValue("parent_path", get_parent_path(cfg_path));

  std::string input_type = config_->GetSTDStringValue("in_msg_type");

  auto iter = g_input_type_map.find(input_type);
  HOBOT_CHECK(iter != g_input_type_map.end())
      << "in_msg_type unknown: " << input_type;
  predictor_.reset(PredictorFactory::GetPredictor(iter->second));

  std::string post_fn = config_->GetSTDStringValue("post_fn");
  auto fn_iter = g_post_fun_map.find(post_fn);
  HOBOT_CHECK(fn_iter != g_post_fun_map.end()) << "post_fn unknown:" << post_fn;
  post_predict_.reset(PostPredictorFactory::GetPostPredictor(fn_iter->second));

  predictor_->Init(config_);
  post_predict_->Init(config_);
  return 0;
}

void CNNMethod::Finalize() {}

std::vector<BaseDataPtr> CNNMethod::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  HOBOT_CHECK(input.size() > 0);
  CNNMethodRunData run_data;
  run_data.input = &input;
  run_data.param = &param;

  predictor_->Do(&run_data);
  post_predict_->Do(&run_data);
  return run_data.output;
}

int CNNMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  if (ptr->is_json_format_) {
    std::string content = ptr->Format();
    CNNMethodConfig cf(content);
    UpdateParams(cf.config, config_->config);
    predictor_->UpdateParam(config_);
    post_predict_->UpdateParam(config_);
    return 0;
  } else {
    HOBOT_CHECK(0) << "only support json format config";
    return -1;
  }
}

InputParamPtr CNNMethod::GetParameter() const {
  return std::static_pointer_cast<xstream::InputParam>(config_);
}

std::string CNNMethod::GetVersion() const { return predictor_->GetVersion(); }

}  // namespace xstream
