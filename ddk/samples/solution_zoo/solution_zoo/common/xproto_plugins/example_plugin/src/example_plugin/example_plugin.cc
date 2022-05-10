/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     example_plugin.cc
 * \Author   xudong.du
 * \Version  1.0.0.0
 * \Date     2021.4.6
 * \Brief    implement of api file
 */
#include "example_plugin/example_plugin.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "xproto/msg_type/smart_legible_message.h"
#include "xproto/message/msg_registry.h"

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_LEGIBLE_MESSAGE);

namespace xproto {
using xproto::message::SmartLegibleMessage;
using xproto::message::SmartLegibleMessagePtr;


ExamplePlugin::ExamplePlugin(std::string config_path) {
  LOGD << "ExamplePlugin config_path = " << config_path;
}

ExamplePlugin::~ExamplePlugin() {}

int ExamplePlugin::Init() {
  LOGI << "ExamplePlugin Init.";
  RegisterMsg(
      TYPE_SMART_LEGIBLE_MESSAGE,
      std::bind(&ExamplePlugin::FeedSmart, this, std::placeholders::_1));
  return 0;
}

int ExamplePlugin::DeInit() {
  LOGD << "ExamplePlugin::DeInit.";
  return 0;
}

int ExamplePlugin::Start() {
  LOGD << "ExamplePlugin::Start.";
  return 0;
}

int ExamplePlugin::Stop() {
  LOGD << "ExamplePlugin::Stop.";

  return 0;
}

int ExamplePlugin::FeedSmart(XProtoMessagePtr msg) {
  LOGD << "ExamplePlugin::FeedSmart.";
  if (nullptr == msg) {
    return -1;
  }
  auto smart_data = std::static_pointer_cast<SmartLegibleMessage>(msg);
  LOGD << "******ExamplePlugin::FeedSmart******";
  LOGD << "time_stamp: " << smart_data->time_stamp_;
  LOGD << "frame_id： " << smart_data->frame_id_;
  for (auto t : smart_data->smart_data_.targets_) {
    LOGD << "track_id: " << t->track_id_ << ", ";
    for (auto b : t->boxs_) {
      LOGD << "box: [";
      LOGD << *b;
      LOGD << "]";
    }
    for (auto lmk : t->lmks_) {
      LOGD << "lmk: [";
      LOGD << *lmk;
      LOGD << "]";
    }
    for (auto attr : t->attributes_) {
      LOGD << "attribute: [";
      LOGD << *attr;
      LOGD << "]";
    }
    if (t->face_feature_ != nullptr) {
      LOGD << "face_feature: [";
      LOGD << *t->face_feature_;
      LOGD << "]";
    }
    if (t->face_pose_ != nullptr) {
      LOGD << "face_pose: [";
      LOGD << *t->face_pose_;
      LOGD << "]";
    }
    for (auto seg : t->body_seg_) {
      LOGD << "body_seg: [";
      LOGD << *seg;
      LOGD << "]";
    }
  }
  LOGD << "******ExamplePlugin::FeedSmart end******";
  return 0;
}
}  // namespace xproto
