/*
 * @Description: implement of reorder_message.h
 * @Author: shiyu.fu@horizon.ai
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef XPROTO_MESSAGE_STRATEGY_REORDER_MESSAGE_H_
#define XPROTO_MESSAGE_STRATEGY_REORDER_MESSAGE_H_

#include <string>
#include <memory>
#include <vector>

#include "xproto/message/pluginflow/flowmsg.h"

namespace xproto {
namespace message {

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

#endif  // XPROTO_MESSAGE_STRATEGY_REORDER_MESSAGE_H_
