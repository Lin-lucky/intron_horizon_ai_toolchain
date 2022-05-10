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
#ifndef XPROTO_PLUGINS_MESSAGE_CONTROL_MESSAGE_H_
#define XPROTO_PLUGINS_MESSAGE_CONTROL_MESSAGE_H_

#include <memory>
#include <string>

#include "xproto/version.h"
#include "xproto/message/flowmsg.h"

namespace xproto {
namespace message {
#define TYPE_CONTROL_MESSAGE "XPLUGIN_CONTROL_MESSAGE"
/**
 * 命令信息
 * @type_: 类型名称, 表示request或者replay等等
 * @cmd_id_: 命令唯一标志id
 * @value_: 数据载体
 */
struct XPROTO_EXPORT Command {
  std::string type_;
  uint64_t cmd_id_;
  std::string value_;
};

/**
 * ControlMessage
 * @time_stamp_: 时间戳
 * @static_message_: 统计信息
 */
struct XPROTO_EXPORT ControlMessage : public xproto::XProtoMessage {
  uint64_t time_stamp_;
  Command message_;

  ControlMessage() { type_ = TYPE_CONTROL_MESSAGE; }
  virtual ~ControlMessage() = default;
  std::string Serialize() override;
  bool DeSerialize(const std::string &data) override;
};
using ControlMessagePtr = std::shared_ptr<ControlMessage>;
}  // namespace message
}  // namespace xproto

#endif  // XPROTO_PLUGINS_MESSAGE_CONTROL_MESSAGE_H_
