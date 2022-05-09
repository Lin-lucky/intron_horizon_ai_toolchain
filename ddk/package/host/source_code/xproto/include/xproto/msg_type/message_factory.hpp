/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xudong.du
 * @Mail: xudong.du@horizon.ai
 * @Date: 2019-09-29 01:50:41
 * @Version: v0.0.1
 * @Brief: smart message declaration.
 * @Last Modified by: xudong.du
 * @Last Modified time: 2021-03-31 02:49:28
 */
#include <memory>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "xproto/message/flowmsg.h"
#include "xproto/msg_type/control_message.h"
#include "xproto/msg_type/smart_legible_message.h"
#include "xproto/msg_type/statistics_message.h"
#include "xproto/version.h"

namespace xproto {
namespace message {
XPROTO_EXPORT XProtoMessagePtr CreateXprotoMsg(const std::string& msg_type,
                                               const std::string& data) {
  XProtoMessagePtr msg = nullptr;
  if (TYPE_CONTROL_MESSAGE == msg_type) {
    msg = std::make_shared<ControlMessage>();
    msg->DeSerialize(data);
  } else if (TYPE_SMART_LEGIBLE_MESSAGE == msg_type) {
    msg = std::make_shared<SmartLegibleMessage>();
    msg->DeSerialize(data);
  } else if (TYPE_STATISTICS_MESSAGE == msg_type) {
    msg = std::make_shared<StatisticsMessage>();
    msg->DeSerialize(data);
  } else {
    LOGW << "CreateXprotoMsg: Not support msg_type: " << msg_type;
  }
  return msg;
}
}  // namespace message
}  // namespace xproto

