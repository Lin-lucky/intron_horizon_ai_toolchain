/*
 * Copyright (c) 2020-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     rndis_manager.h
 * \Author   xudong.du
 * \Version  1.0.0.0
 * \Date     2020/9/27
 * \Brief    implement of api header
 */
#ifndef INCLUDE_UVCPLUGIN_RNDISMANAGER_H_
#define INCLUDE_UVCPLUGIN_RNDISMANAGER_H_
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <zmq.hpp>

#include "smart_manager.h"
#include "json/json.h"

namespace xproto {
class RndisManager : public SmartManager {
 public:
  RndisManager() = delete;
  explicit RndisManager(std::string config_path);
  ~RndisManager();
  int Init() override;

 private:
  int Recv(std::string &recvstr) override;
  int Send(const std::string &sendstr) override;

  std::string config_path_;
  Json::Value config_;

  std::shared_ptr<zmq::context_t> server_context_;
  std::shared_ptr<zmq::context_t> server_reply_context_;
  std::shared_ptr<zmq::socket_t> smart_publisher_;
  std::shared_ptr<zmq::socket_t> smart_reply_;
  int smart_port_ = 9999;
  int smart_reply_port_ = 10001;
};
}  // namespace xproto

#endif  // INCLUDE_UVCPLUGIN_RNDISMANAGER_H_
