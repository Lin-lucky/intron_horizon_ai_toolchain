/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     zmq_manager.h
 * \Author   Xudong Du
 * \Mail     Xudong.du@horizon.ai
 * \Version  1.0.0.0
 * \Date     2019-04-22
 * \Brief    implement of msg_manager.h
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#ifndef XPROTO_INCLUDE_XPROTO_SESSION_ZMQ_MANAGER_H_
#define XPROTO_INCLUDE_XPROTO_SESSION_ZMQ_MANAGER_H_

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "xproto/session/xsession_info.h"
#include "xproto/session/xsocket.h"
#include "xproto/threadpool.h"
#include "xproto/utils/ring_queue.h"
#include "xproto/utils/singleton.h"
#include "zmq.hpp"

namespace xproto {

struct MsgData {
  ActionType action_type_;
  std::string msg_type_;
  std::string msg_data_;
  MsgData() {
    action_type_ = ACTION_NONE;
    msg_type_.clear();
    msg_data_.clear();
  }
  MsgData(const ActionType action_type, const std::string& msg_type,
          const std::string& msg_data) {
    action_type_ = action_type;
    msg_type_ = std::move(msg_type);
    msg_data_ = std::move(msg_data);
  }
};

class ZmqManager : public XSocket {
 public:
  ZmqManager();
  ~ZmqManager();
  virtual int Init(bool is_server, MessageCallBack* call_back);
  virtual int UnInit();
  virtual int Bind(const std::string& host_ip, uint16_t server_port);
  virtual int ConnectTo(const std::string& ip, uint16_t port,
                        std::string& version);
  virtual int ProcessOutputMsg(const ActionType action_type,
                               const std::string& msg_type,
                               const std::string& data, void* extra_data);
  virtual int GetInputMsg(ActionType& action_type, std::string& msg_type,
                          std::string& data);
  virtual void Close();

 private:
  void ServerPubProc(const std::string& tcpaddr);
  void ServerRouterProc(const std::string& tcpaddr);
  void ClientSubProc(const std::string& tcpaddr);
  void ClientDealerProc(const std::string& tcpaddr,
                        const std::string& version_data);
  void CommandProc();

 private:
  std::shared_ptr<zmq::context_t> zmq_context_;
  std::atomic<bool> is_server_;
  bool is_inited_ = false;
  std::atomic<bool> is_online_;
  std::string ip_;
  uint16_t port_;
  xproto::CThreadPool server_pub_handle_;
  xproto::CThreadPool server_router_handle_;
  xproto::CThreadPool client_sub_handle_;
  xproto::CThreadPool client_dealer_handle_;
  xproto::CThreadPool command_pub_handle_;
  const int ring_queue_max_ = 60;
  xproto::RingQueue<MsgData> wait_send_datas_;
  xproto::RingQueue<MsgData> recv_datas_;
  std::atomic<bool> stop_;
  MessageCallBack* call_back_ = NULL;
  /* 进程内各个zmq socket之间通信的地址 */
  const std::string command_address_ = "inproc://xprotocommand";
  /* 用于发布订阅消息的地址 */
  std::string msg_tcp_address_;
  /* 用于交换version等信息的地址 */
  std::string version_tcp_address_;
  int zero_ = 0;
  std::mutex first_request_mtx_;
  std::condition_variable first_request_cv_;
  bool firtst_reply_ = false;
};

}  // namespace xproto

#endif  // XPROTO_INCLUDE_XPROTO_SESSION_ZMQ_MANAGER_H_
