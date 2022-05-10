/*!
 * Copyright (c) 2020-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     rndis_manager.cpp
 * \Author   xudong.du
 * \Version  1.0.0.0
 * \Date     2020.9.27
 * \Brief    implement of api file
 */
#include "rndis_manager.h"

#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <string>

#include "hobotlog/hobotlog.hpp"

namespace xproto {

RndisManager::RndisManager(std::string config_path)
    : SmartManager(config_path) {
  config_path_ = config_path;
  LOGI << "RndisManager smart config file path:" << config_path_;

  server_context_ = std::make_shared<zmq::context_t>(1);
  smart_publisher_ = std::make_shared<zmq::socket_t>(*server_context_, ZMQ_PUB);

  server_reply_context_ = std::make_shared<zmq::context_t>(1);
  smart_reply_ =
      std::make_shared<zmq::socket_t>(*server_reply_context_, ZMQ_REP);
}

RndisManager::~RndisManager() {}

int RndisManager::Init() {
  LOGI << "RndisManager Init";
  // config_
  if (config_path_ != "") {
    std::ifstream ifs(config_path_);
    if (!ifs.is_open()) {
      LOGF << "open config file " << config_path_ << " failed";
      return -1;
    }
    Json::CharReaderBuilder builder;
    std::string err_json;
    try {
      bool ret = Json::parseFromStream(builder, ifs, &config_, &err_json);
      if (!ret) {
        LOGF << "invalid config file " << config_path_;
        return -1;
      }
    } catch (std::exception &e) {
      LOGF << "exception while parse config file " << config_path_ << ", "
           << e.what();
      return -1;
    }

    // rndis_port
    if (config_.isMember("rndis_port")) {
      smart_port_ = config_["rndis_port"].asInt();
    }
  }

  // start zmq server
  std::string tcpaddr = "tcp://*:";
  tcpaddr += std::to_string(smart_port_);
  try {
    smart_publisher_->bind(tcpaddr);
  } catch (std::exception &e) {
    LOGE << "bind port: " << smart_port_ << " failed, " << e.what();
  }

  std::string repaddr = "tcp://*:";
  repaddr += std::to_string(smart_reply_port_);
  try {
    smart_reply_->bind(repaddr);
  } catch (std::exception &e) {
    LOGE << "bind port: " << smart_reply_port_ << " failed, " << e.what();
  }
  smart_reply_->setsockopt(ZMQ_RCVTIMEO, 500);

  return SmartManager::Init();
}

int RndisManager::Send(const std::string &sendstr) {
  if (sendstr.empty()) {
    return 0;
  }
  zmq::message_t message(sendstr.c_str(), sendstr.length());
  smart_publisher_->send(message, zmq::send_flags::none);
  return sendstr.size();
}

int RndisManager::Recv(std::string &recvstr) {
  zmq::message_t request;
  auto result = smart_reply_->recv(request);
  if (result) {
    zmq::message_t reply(2);
    memcpy(reinterpret_cast<void *>(reply.data()), "OK", 2);
    smart_reply_->send(reply, zmq::send_flags::dontwait);
  }
  recvstr = request.to_string();
  LOGD << "RndisManager Recv " << recvstr.size();
  return recvstr.size();
}
}  // namespace xproto
