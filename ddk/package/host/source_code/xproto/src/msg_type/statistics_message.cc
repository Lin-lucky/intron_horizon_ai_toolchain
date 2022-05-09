//
// Created by xudong.du@hobot.cc on 14/15/2021.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#include "xproto/msg_type/statistics_message.h"

#include <memory>

#include "hobotlog/hobotlog.hpp"
#include "xproto/msg_type/protobuf/x3.pb.h"

namespace xproto {
namespace message {
std::string StatisticsMessage::Serialize() {
  std::string proto_str;
  x3::FrameMessage proto_frame_message;
  proto_frame_message.set_timestamp_(time_stamp_);
  auto proto_messages = proto_frame_message.mutable_statistics_msg_();
  for (auto msg : messages_) {
    auto attr = proto_messages->add_attributes_();
    attr->set_value_(msg->value_);
    attr->set_score_(msg->score_);
    attr->set_type_(msg->name_);  //  比如年龄/性别/CPU/温度等等
    attr->set_value_string_(msg->specific_type_);
  }
  proto_frame_message.SerializeToString(&proto_str);
  return std::move(proto_str);
}

bool StatisticsMessage::DeSerialize(const std::string &data) {
  if (data.empty()) {
    return false;
  }
  x3::FrameMessage proto_frame_message;
  auto ret = proto_frame_message.ParseFromString(data);
  if (ret) {
    time_stamp_ = proto_frame_message.timestamp_();
    auto proto_message = proto_frame_message.statistics_msg_();
    for (auto attr : proto_message.attributes_()) {
      auto msg = std::make_shared<Attribute_<int32_t>>();
      msg->name_ = attr.type_();
      msg->score_ = attr.value_();
      msg->specific_type_ = attr.value_string_();
      msg->value_ = attr.value_();
      messages_.push_back(msg);
    }
  }
  return ret;
}
}  // namespace message
}  // namespace xproto

