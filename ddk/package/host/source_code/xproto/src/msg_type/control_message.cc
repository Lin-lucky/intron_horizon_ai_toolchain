//
// Created by xudong.du@hobot.cc on 14/15/2021.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#include "xproto/msg_type/control_message.h"

#include <memory>

#include "hobotlog/hobotlog.hpp"
#include "xproto/msg_type/protobuf/x3.pb.h"

namespace xproto {
namespace message {
std::string ControlMessage::Serialize() {
  std::string data;
  x3::FrameMessage proto_frame_message;
  proto_frame_message.set_timestamp_(time_stamp_);
  auto proto_command = proto_frame_message.mutable_control_msg_();
  proto_command->set_type_(message_.type_);
  proto_command->set_cmd_id_(message_.cmd_id_);
  proto_command->set_value_(message_.value_);
  proto_frame_message.SerializeToString(&data);
  return std::move(data);
}

bool ControlMessage::DeSerialize(const std::string &data) {
  if (data.empty()) {
    return false;
  }
  x3::FrameMessage proto_frame_message;
  auto ret = proto_frame_message.ParseFromString(data);
  if (ret) {
    time_stamp_ = proto_frame_message.timestamp_();
    auto proto_command = proto_frame_message.control_msg_();
    message_.cmd_id_ = proto_command.cmd_id_();
    message_.type_ = proto_command.type_();
    message_.value_ = std::move(proto_command.value_());
  }
  return ret;
}
}  // namespace message
}  // namespace xproto

