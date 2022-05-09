/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     xsocket.h
 * \Author   Xudong Du
 * \Mail     Xudong.du@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-04-22
 * \Brief    implement of msg_manager.h
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#ifndef XPROTO_INCLUDE_XPROTO_SESSION_XSOCKET_H_
#define XPROTO_INCLUDE_XPROTO_SESSION_XSOCKET_H_

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xproto/session/xsession_info.h"
#include "xproto/threadpool.h"
#include "xproto/utils/singleton.h"

namespace xproto {
enum ActionType {
  ACTION_NONE = -1,
  ACTION_SEND_MSG = 0,
  ACTION_RECV_MSG = 1,
  ACTION_SUBSCRIBE_MSG = 2,
  ACTION_UNSUBSCRIBE_MSG = 3,
  ACTION_CONNECT_TO_SERVER = 4,
  ACTION_HEART_UPDATE = 5,
};

class MessageCallBack {
 public:
  /**
   * @description: Process command message
   * @param action_type action type
   * @param msg_type msg type
   * @param data msg serialized data buffer, witch can be transefer on network
   * @param extra_len extra_len, as flag ...
   * @return {*} 0: do not reply to client; 1:reply 'ok' to client; 2:reply
   * 'error' to client.
   */
  virtual int OnMessage(ActionType action_type, std::string& msg_type,
                        std::string& data) = 0;
};

class XSocket {
 public:
  XSocket() {}
  ~XSocket() {}
  virtual int Init(bool is_server, MessageCallBack* call_back) = 0;
  virtual int UnInit() = 0;
  virtual int Bind(const std::string& host_ip, uint16_t server_port) = 0;
  virtual int ConnectTo(const std::string& ip, uint16_t port,
                        std::string& version) = 0;
  /**
   * @description: Send msg data to subsriber
   * @param action_type action type
   * @param msg_type msg type
   * @param data msg serialized data buffer, witch can be transefer on network
   * @param extra_len extra_len, as flag ...
   * @return {*} 0 suc or -1 failed.
   */
  virtual int ProcessOutputMsg(const ActionType action_type,
                               const std::string& msg_type,
                               const std::string& data, void* extra_data) = 0;
  /**
   * @description: Get Input msg
   * @param action_type action type
   * @param msg_type return recv msg type
   * @param data msg serialized data buffer from network
   * @param extra_len extra_len, as flag ...
   * @return {*} 0 suc or -1 failed.
   */
  virtual int GetInputMsg(ActionType& action_type, std::string& msg_type,
                          std::string& data) = 0;
  virtual void Close() = 0;
};

}  // namespace xproto

#endif  // XPROTO_INCLUDE_XPROTO_SESSION_XSOCKET_H_
