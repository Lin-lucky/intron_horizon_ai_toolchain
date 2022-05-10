/*
 * @Description: implement of uvc_message.h
 * @Author: yingmin.li@horizon.ai
 * @Date: 2019-08-24 11:29:24
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:11:08
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef XPROTO_MESSAGE_UVC_TRANSPORT_MESSAGE_H_
#define XPROTO_MESSAGE_UVC_TRANSPORT_MESSAGE_H_

#include <string>
#include <utility>
#include "xproto/message/flowmsg.h"

namespace xproto {
namespace message {

#define TYPE_UVC_MESSAGE "XPLUGIN_UVC_MESSAGE"
#define TYPE_TRANSPORT_MESSAGE "XPLUGIN_TRANSPORT_MESSAGE"

struct UvcMessage : XProtoMessage {
  UvcMessage() { type_ = TYPE_UVC_MESSAGE; }
  std::string Serialize() override { return "Default uvc message"; }
  virtual ~UvcMessage() = default;
};

struct TransportMessage : XProtoMessage {
  explicit TransportMessage(const std::string& proto) {
    //  这里并不会起到预期右值转移的作用，
    //  因为const限制了右值转移函数的调用
    proto_ = std::move(proto);
    type_ = TYPE_TRANSPORT_MESSAGE;
  }
  explicit TransportMessage(std::string&& proto) {
    proto_ = std::forward<std::string>(proto);
    type_ = TYPE_TRANSPORT_MESSAGE;
  }
  std::string Serialize() override { return "Default transport message"; }
  virtual ~TransportMessage() = default;

  std::string proto_;
};
}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MESSAGE_UVC_TRANSPORT_MESSAGE_H_
