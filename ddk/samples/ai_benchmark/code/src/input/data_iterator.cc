// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "input/data_iterator.h"

#include "base/common_def.h"
#include "glog/logging.h"

int DataIterator::Init(std::string config_file,
                       std::string config_string,
                       hbDNNTensorLayout tensor_layout) {
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

  tensor_layout_ = tensor_layout;

  return 0;
}

int DataIterator::LoadConfigFile(std::string& config_file) {
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

DataIterator* DataIterator::GetImpl(const std::string& module_name) {
  return DataIteratorFactory::GetInstance()->GetDataIterator(
      module_name.data());
}
