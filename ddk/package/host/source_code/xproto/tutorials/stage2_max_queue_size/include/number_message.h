/*
 * Copyright (c) 2020 Horizon Robotics
 * @brief     definition of number_message
 * @author    zhe.sun
 * @date      2020.10.4
 */

#ifndef TUTORIALS_STAGE2_INCLUDE_NUMBERMESSAGE_H_
#define TUTORIALS_STAGE2_INCLUDE_NUMBERMESSAGE_H_

#include <sstream>
#include <string>
#include "xproto/xproto_world.h"

#define TYPE_NUMBER_MESSAGE1 "XPLUGIN_NUMBER_MESSAGE1"  // 定义消息类型
#define TYPE_NUMBER_MESSAGE2 "XPLUGIN_NUMBER_MESSAGE2"  // 定义消息类型
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_NUMBER_MESSAGE1)      // 注册消息
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_NUMBER_MESSAGE2)      // 注册消息

struct NumberProdMessage1 : xproto::XProtoMessage {
  float num_;
  explicit NumberProdMessage1(float num) :num_(num) {
    type_ = TYPE_NUMBER_MESSAGE1;     // 指定消息类型
  }
  std::string Serialize() override {
    std::ostringstream ss;
    ss << num_;
    return std::string(ss.str());
  }
};

struct NumberProdMessage2 : xproto::XProtoMessage {
  float num_;
  explicit NumberProdMessage2(float num) :num_(num) {
    type_ = TYPE_NUMBER_MESSAGE2;     // 指定消息类型
  }
  std::string Serialize() override {
    std::ostringstream ss;
    ss << num_;
    return std::string(ss.str());
  }
};

#endif  // TUTORIALS_STAGE2_INCLUDE_NUMBERMESSAGE_H_
