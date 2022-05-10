/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: reorder_message.h
 * @Brief: decalration of reorder message
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Mon Mar 15 2021 10:53:22
 */

#ifndef REORDER_MESSAGE_REORDER_MESSAGE_H_
#define REORDER_MESSAGE_REORDER_MESSAGE_H_

#include <memory>
#include <string>

#include "xproto/message/flowmsg.h"

namespace xproto {
namespace message {

using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;

#define TYPE_REORDER_MESSAGE "XPLUGIN_REORDER_MESSAGE"

struct ReorderMessage : XProtoMessage {
  ReorderMessage() { type_ = TYPE_REORDER_MESSAGE; }
  std::string Serialize() override { return "Default reorder message"; }
  virtual ~ReorderMessage() = default;

  XProtoMessagePtr actual_msg_;
  std::string actual_type_;
  uint64_t frame_id_;
};

using ReorderMsgPtr = std::shared_ptr<ReorderMessage>;

}  // namespace message
}  // namespace xproto

#endif  // REORDER_MESSAGE_REORDER_MESSAGE_H_
