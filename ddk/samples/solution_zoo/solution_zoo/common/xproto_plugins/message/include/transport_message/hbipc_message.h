/*
 * @Description: implement of hbipc_message.h
 * @Author: yingmin.li@horizon.ai
 * @Date: 2019-08-24 11:29:24
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:11:08
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef XPROTO_MESSAGE_TRANSPORT_HBIPC_MESSAGE_H_
#define XPROTO_MESSAGE_TRANSPORT_HBIPC_MESSAGE_H_

#include <string>

#include "xproto/message/flowmsg.h"

namespace xproto {
namespace message {

using xproto::XProtoMessage;

#define TYPE_HBIPC_MESSAGE "XPLUGIN_HBIPC_MESSAGE"

struct HbipcMessage : XProtoMessage {
  HbipcMessage() { type_ = TYPE_HBIPC_MESSAGE; }
  std::string Serialize() override { return "Default hbipc message"; }
  virtual ~HbipcMessage() = default;
};

}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MESSAGE_TRANSPORT_HBIPC_MESSAGE_H_
