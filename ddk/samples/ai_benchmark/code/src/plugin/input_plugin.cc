// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "plugin/input_plugin.h"

#include <atomic>

#include "glog/logging.h"
#include "rapidjson/document.h"
#include "xproto/message/msg_registry.h"

Time fps_start;
std::atomic_long fps_count(0);  // total frame count

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_INPUT_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_RELEASE_MESSAGE)

int InputProducerPlugin::Init(std::string config_file,
                              std::string config_string) {
  int ret_code = BasePlugin::Init(config_file, config_string);
  if (ret_code != 0) {
    return ret_code;
  }

  RegisterMsg(
      TYPE_RELEASE_MESSAGE,
      std::bind(&InputProducerPlugin::Release, this, std::placeholders::_1));

  return XPluginAsync::Init();
}

void InputProducerPlugin::Produce() {
  fps_start = std::chrono::steady_clock::now();
  while (data_iterator_->HasNext() && !stop_) {
    ImageTensorPtr image_tensor(new ImageTensor);
    if (!data_iterator_->Next(image_tensor.get())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&] { return produced_count_ - released_count_ < limit_; });
    VLOG(EXAMPLE_DEBUG) << "Produce input msg, id:" << produced_count_;
    produced_count_++;
    PushMsg(std::make_shared<InputMessage>(image_tensor));
  }
  VLOG(EXAMPLE_DEBUG) << "InputProducerPlugin Produce finish.";
}

int InputProducerPlugin::Release(XProtoMessagePtr msg) {
  auto input_msg = std::static_pointer_cast<InputMessage>(msg);
  auto image_tensor = input_msg->image_tensor;
  data_iterator_->Release(image_tensor.get());
  VLOG(EXAMPLE_DEBUG) << "Release input msg";
  std::unique_lock<std::mutex> lk(m_);
  released_count_++;
  lk.unlock();
  cv_.notify_one();
  return 0;
}

int InputProducerPlugin::Start() {
  stop_ = false;
  produce_thread_ =
      std::make_shared<std::thread>(&InputProducerPlugin::Produce, this);
  if (!produce_thread_) {
    VLOG(EXAMPLE_SYSTEM) << "Start thread failed.";
    return -1;
  }
  VLOG(EXAMPLE_DETAIL) << "InputProducerPlugin start.";
  return 0;
}

bool InputProducerPlugin::IsRunning() {
  return !stop_ && data_iterator_->HasNext();
}

int InputProducerPlugin::Stop() {
  stop_ = true;
  while (released_count_ < produced_count_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (produce_thread_) {
    if (produce_thread_->joinable()) {
      produce_thread_->join();
    }
  }
  VLOG(EXAMPLE_DETAIL) << "InputProducerPlugin stop.";
  return 0;
}

int InputProducerPlugin::LoadConfig(std::string &config_string) {
  rapidjson::Document document;
  document.Parse(config_string.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }

  if (document.HasMember("limit")) {
    limit_ = document["limit"].GetInt();
  }

  if (document.HasMember("input_type")) {
    std::string input_type = document["input_type"].GetString();
    data_iterator_ = DataIterator::GetImpl(input_type);
  } else {
    VLOG(EXAMPLE_SYSTEM)
        << "input config don not have parameter input_type! please check!";
    return -1;
  }

  // TODO(ruxin.song): get tensor layout
  return data_iterator_->Init("", config_string, HB_DNN_LAYOUT_NCHW);
}

InputProducerPlugin::~InputProducerPlugin() {
  if (data_iterator_) {
    delete data_iterator_;
    data_iterator_ = nullptr;
  }
}
