//
// Created by xudong.du@hobot.cc on 14/15/2021.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#include "xproto/session/xsession.h"

#include "hobotlog/hobotlog.hpp"
#include "xproto/session/xsession_inner.h"
#include "xproto/session/zmq_manager.h"

namespace xproto {

XSession::~XSession() { XSessionInner::Instance().Close(); }

int32_t XSession::AsMaster(uint16_t host_port) {
  return XSessionInner::Instance().AsMaster("*", host_port);
}

int32_t XSession::ConnectTo(const std::string &ip, uint16_t port) {
  return XSessionInner::Instance().ConnectTo(ip, port);
}

std::vector<SessionInfo> XSession::Info() {
  return XSessionInner::Instance().Info();
}

void XSession::Reset() { XSessionInner::Instance().Reset(); }

XSession &XSession::Instance() {
  static XSession inst;
  return inst;
}
}  // namespace xproto
