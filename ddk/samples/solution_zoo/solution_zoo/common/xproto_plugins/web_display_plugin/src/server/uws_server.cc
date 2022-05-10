/*!
 * -------------------------------------------
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     uws_server.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020/5/12
 * \Brief    implement of uws_server.cpp
 * \DO NOT MODIFY THIS COMMENT, \
 * \WHICH IS AUTO GENERATED BY EDITOR
 * -------------------------------------------
 */

#include "web_display_plugin/server/uws_server.h"

#include <chrono>
#include <fstream>

#include "hobotlog/hobotlog.hpp"
#include "uWS/uWS.h"

namespace xproto {
using std::chrono::milliseconds;

int UwsServer::Init() {
  if (nullptr == worker_) {
    worker_ = std::make_shared<std::thread>(&UwsServer::StartServer, this);
    worker_->detach();
  }
  return 0;
}

int UwsServer::DeInit() {
  //  server_.
  LOGI <<"UwsServer::DeInit()";
  connetion_ = nullptr;
  return 0;
}

UwsServer::UwsServer(const std::string &config_file) : connetion_(nullptr),
  worker_(nullptr) {
}

void UwsServer::StartServer() {
  LOGI << "UwsServer::StartServer";
  uWS::Hub hub;
  hub.onConnection(
      [this](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
        LOGI << "UwsServer Connection with PC success";
        std::lock_guard<std::mutex> connection_mutex(mutex_);
        connetion_ = ws;
      });

  hub.onMessage([this](uWS::WebSocket<uWS::SERVER> *ws, char *message,
                       size_t length, uWS::OpCode opCode) {
    LOGI << "UwsServer onMessage: " << message;
  });

  hub.onDisconnection([this](uWS::WebSocket<uWS::SERVER> *ws, int code,
                             char *message, size_t length) {
    std::lock_guard<std::mutex> connection_mutex(mutex_);
    connetion_ = nullptr;
    LOGI << "UwsServer Disconnection with PC success";
  });
  if (!hub.listen(8080)) {
    LOGI << "UwsServer start failed";
    return;
  }
  LOGI << "UwsServer begin to run";
  hub.run();
}
int UwsServer::Send(const std::string &protocol) {
  if (connetion_ != nullptr) {
    LOGI << "UwsServer begin send protocol";
    connetion_->send(protocol.c_str(), protocol.size(), uWS::OpCode::BINARY);
    LOGI << "UwsServer send protocol size = " << protocol.size();
  }
  return 0;
}
}  // namespace xproto