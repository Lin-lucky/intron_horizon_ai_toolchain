/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     xplugin.cpp
 * \Author Songshan Gong
 * \Mail     songshan.gong@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-07-30
 * \Brief    implmentation of xplugin
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#include "xproto/plugin/xplugin.h"

#include "xproto/message//flowmsg.h"
#include "xproto/msg_manager.h"

namespace xproto {

void XPlugin::RegisterMsg(const std::string& type, int32_t msg_source) {
  XMsgQueue::Instance().RegisterPlugin(shared_from_this(), type, msg_source);
}

void XPlugin::UnRegisterMsg(const std::string& type) {
  XMsgQueue::Instance().UnRegisterPlugin(shared_from_this(), type);
}

void XPlugin::PushMsg(XProtoMessagePtr msg, int32_t msg_dst) {
  XMsgQueue::Instance().PushMsg(msg, msg_dst);
}

int XPlugin::TryPushMsg(XProtoMessagePtr msg, int32_t msg_dst) {
  return XMsgQueue::Instance().TryPushMsg(msg, msg_dst);
}

}  // namespace xproto
