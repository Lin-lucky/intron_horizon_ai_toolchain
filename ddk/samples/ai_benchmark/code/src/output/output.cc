// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "output/output.h"

#include <fstream>
#include <string>

#include "glog/logging.h"
#include "rapidjson/document.h"

OutputAssembler *OutputAssembler::GetIns() {
  static OutputAssembler *ins = nullptr;
  if (nullptr == ins) {
    ins = new OutputAssembler();
  }
  return ins;
}

OutputAssembler::~OutputAssembler() {
  for (auto o : output_list_) {
    if (o) {
      delete (o);
    }
  }
  output_list_.clear();
}

int OutputAssembler::Init(const std::string &config) {
  std::ifstream cfg(config);
  if (!cfg) {
    VLOG(EXAMPLE_SYSTEM) << "config file not found: " << config;
    return -1;
  }

  VLOG(EXAMPLE_DEBUG) << "config file found " << config;
  std::stringstream buffer;
  buffer << cfg.rdbuf();
  std::string contents(buffer.str());

  rapidjson::Document document;
  document.Parse(contents.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "parse config file failed: " << config;
    return -1;
  }

  if (document.HasMember("eval_enable")) {
    rapidjson::Value &eval_enable = document["eval_enable"];
    if (eval_enable.GetBool()) {
      OutputModule *ins = OutputModule::GetImpl("eval");
      int ret = ins->Init(document);
      if (ret != 0) {
        VLOG(EXAMPLE_SYSTEM) << "here init raw output fail.";
        delete ins;
      } else {
        VLOG(EXAMPLE_DEBUG) << "here init raw output success.";
        output_list_.push_back(ins);
      }
    }
  }

  if (document.HasMember("image_list_enable")) {
    rapidjson::Value &image_list_enable = document["image_list_enable"];
    if (image_list_enable.GetBool()) {
      OutputModule *ins = OutputModule::GetImpl("image");
      int ret = ins->Init(document);
      if (ret != 0) {
        VLOG(EXAMPLE_SYSTEM) << "here init image_list writer fail.";
        delete ins;
      } else {
        VLOG(EXAMPLE_DEBUG) << "here init image_list writer sucess.";
        output_list_.push_back(ins);
      }
    }
  }

  return 0;
}

void OutputAssembler::Send(ImageTensor *data, Perception *perception) {
  for (auto out : output_list_) {
    if (out) {
      out->Write(data, perception);
    }
  }
}

int OutputModule::Init(std::string config_file, std::string config_string) {
  if (!config_file.empty()) {
    int ret_code = this->LoadConfigFile(config_file);
    if (ret_code != 0) {
      return ret_code;
    }
  }

  if (!config_string.empty()) {
    int ret_code = this->LoadConfig(config_string);
    if (ret_code != 0) {
      return ret_code;
    }
  }

  return 0;
}

int OutputModule::LoadConfigFile(std::string &config_file) {
  std::ifstream ifs(config_file.c_str());
  if (!ifs) {
    VLOG(EXAMPLE_SYSTEM) << "Open config file " << config_file << " failed";
    return -1;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  std::string contents(buffer.str());
  return this->LoadConfig(contents);
}

OutputModule *OutputModule::GetImpl(const std::string &module_name) {
  return OutputModuleFactory::GetInstance()->GetOutputModule(
      module_name.data());
}
