// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "plugin/output_plugin.h"

#include <iostream>

#include "base/common_def.h"
#include "glog/logging.h"
#include "rapidjson/document.h"
#include "xproto/message/msg_registry.h"

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_OUTPUT_MESSAGE)

int OutputConsumerPlugin::Init(std::string config_file,
                               std::string config_string) {
  int ret_code = BasePlugin::Init(config_file, config_string);
  if (ret_code != 0) {
    return ret_code;
  }

  char *fps_log = getenv(FPS_LOG);
  fps_status_ = (fps_log != nullptr) ? atoi(fps_log) : 0;

  RegisterMsg(
      TYPE_OUTPUT_MESSAGE,
      std::bind(&OutputConsumerPlugin::Send, this, std::placeholders::_1));
  return XPluginAsync::Init();
}

int OutputConsumerPlugin::Send(XProtoMessagePtr msg) {
  auto frame_id =
      std::static_pointer_cast<OutputMessage>(msg)->image_tensor->frame_id;
  cache_[frame_id] = msg;
  while (cache_.count(next_frame)) {
    auto output_msg =
        std::static_pointer_cast<OutputMessage>(cache_[next_frame]);
    auto image_tensor = output_msg->image_tensor;
    auto perception = output_msg->perception;
    output_module_->Write(image_tensor.get(), perception.get());
    PushMsg(std::make_shared<ReleaseMessage>(image_tensor));
    cache_.erase(next_frame++);
  }
  VLOG(EXAMPLE_DEBUG) << "OutputConsumerPlugin Send finished.";
  if (fps_status_) {
    FPS_PERF_RECORD
  }
  return 0;
}

int OutputConsumerPlugin::Start() {
  VLOG(EXAMPLE_DETAIL) << "OutputConsumerPlugin start.";
  return 0;
}

int OutputConsumerPlugin::Stop() {
  VLOG(EXAMPLE_DETAIL) << "OutputConsumerPlugin stop.";
  return 0;
}

int OutputConsumerPlugin::LoadConfig(std::string &config_string) {
  rapidjson::Document document;
  document.Parse(config_string.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }
  if (document.HasMember("output_type")) {
    std::string output_type = document["output_type"].GetString();
    output_module_ = OutputModule::GetImpl(output_type);
  } else {
    VLOG(EXAMPLE_SYSTEM)
        << "output config don not have parameter output_type! please check!";
    return -1;
  }
  return output_module_->Init("", config_string);
}

OutputConsumerPlugin::~OutputConsumerPlugin() {
  if (output_module_) {
    delete output_module_;
    output_module_ = nullptr;
  }
}
