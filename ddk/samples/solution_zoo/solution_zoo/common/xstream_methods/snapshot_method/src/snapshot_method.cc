/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     Grading Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.01.03
 */

#include "snapshot_method/snapshot_method.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include "snapshot_method/error_code.h"
#include "snapshot_method/strategy/crop.h"
#include "snapshot_method/strategy/first_num_best.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

namespace xstream {

using std::chrono::duration;
using std::chrono::high_resolution_clock;

int SnapShotMethod::Init(const std::string &config_file_path) {
  LOGI << "SnapShotMethod::Init " << config_file_path << std::endl;
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGI << "SnapShotParam: no config, using default parameters" << std::endl;
  }
  std::ostringstream buf;
  char ch;
  while (buf && config_if.get(ch)) {
    buf.put(ch);
  }
  method_config_param_ = std::make_shared<FirstNumBestParam>();
  method_config_param_->UpdateParameter(buf.str());
  auto select = std::make_shared<FirstNumBest>();
  select->Init(method_config_param_);
  auto crop = std::make_shared<Crop>();
  crop->Init(method_config_param_);
  strategy_map_["select"] = select;
  strategy_map_["crop"] = crop;
  default_method_config_param_ = std::make_shared<FirstNumBestParam>();
  default_method_config_param_->UpdateParameter(buf.str());
  return XSTREAM_SNAPSHOT_OK;
}

std::vector<BaseDataPtr> SnapShotMethod::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr &param) {
  auto start_time = high_resolution_clock::now();
  LOGI << "SnapShotMethod::DoProcess";

  std::vector<BaseDataPtr> output;
  output = ProcessOneBatch(input, param);

  auto end_time = high_resolution_clock::now();
  duration<double, std::milli> proc_cost = end_time - start_time;
  LOGI << "DoProcess cost(ms):" << proc_cost.count();

  return output;
}

std::vector<BaseDataPtr> SnapShotMethod::ProcessOneBatch(
    const std::vector<BaseDataPtr> &in, const InputParamPtr &param) {
  HOBOT_CHECK(!in.empty());
  if (param) {
    if (param->is_json_format_) {
      std::string content = param->Format();
      Json::CharReaderBuilder builder;
      builder["collectComments"] = false;
      JSONCPP_STRING error;
      std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
      Json::Value out_jv;
      bool ok = json_reader->parse(
          content.c_str(), content.c_str() + content.size(), &out_jv, &error);
      if (ok && out_jv["snapshot_type"].isString()) {
        auto type = out_jv["snapshot_type"].asString();
        if (type == "crop") {
          LOGI << "Crop Mode";
          if (strategy_map_.count("crop") && strategy_map_["crop"]) {
            return strategy_map_["crop"]->ProcessFrame(in, param);
          }
        }
      }
    }
  }
  if (method_config_param_->snapshot_type == "crop") {
    LOGI << "Crop Mode";
    return strategy_map_["crop"]->ProcessFrame(in, param);
  }
  LOGI << "Select Mode";
  return strategy_map_["select"]->ProcessFrame(in, param);
}

void SnapShotMethod::Finalize() {
  for (auto &item : strategy_map_) {
    auto strategy = item.second;
    strategy->Finalize();
  }
  LOGI << "SnapShotMethod::Finalize" << std::endl;
}

int SnapShotMethod::UpdateParameter(InputParamPtr ptr) {
  if (ptr->is_json_format_) {
    std::string content = ptr->Format();
    int ret;
    for (auto &item : strategy_map_) {
      auto strategy = item.second;
      ret = strategy->UpdateParameter(content);
      if (XSTREAM_SNAPSHOT_OK != ret) {
        return ret;
      }
    }
    return XSTREAM_SNAPSHOT_OK;
  } else {
    if (ptr && ptr->Format() == "Reset") {
      LOGI << "Reset snapshot";
      for (auto &item : strategy_map_) {
        auto strategy = item.second;
        strategy->Reset();
      }
      *method_config_param_ = *default_method_config_param_;
      return XSTREAM_SNAPSHOT_OK;
    }
    HOBOT_CHECK(0) << "only support json format config and reset config";
    return XSTREAM_SNAPSHOT_ERR_PARAM;
  }
}

InputParamPtr SnapShotMethod::GetParameter() const {
  return method_config_param_;
}
}  // namespace xstream
