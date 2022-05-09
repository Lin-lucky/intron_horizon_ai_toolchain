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
#ifndef XPROTO_PLUGINS_MESSAGE_STATISTICS_MESSAGE_H_
#define XPROTO_PLUGINS_MESSAGE_STATISTICS_MESSAGE_H_

#include <memory>
#include <string>
#include <vector>

#include "xproto/version.h"
#include "xproto/message/flowmsg.h"
#include "xstream/vision_type.h"

namespace xproto {
namespace message {
#define TYPE_STATISTICS_MESSAGE "XPLUGIN_STATISTICS_MESSAGE"

using xstream::Attribute_;
typedef std::shared_ptr<Attribute_<int32_t>> AttributePtr;

/**
 * StatisticsMessage
 * @time_stamp_: 时间戳
 * @static_message_: 统计信息
 */
struct XPROTO_EXPORT StatisticsMessage : public xproto::XProtoMessage {
  uint64_t time_stamp_;
  std::vector<AttributePtr> messages_;

  StatisticsMessage() { type_ = TYPE_STATISTICS_MESSAGE; }
  virtual ~StatisticsMessage() = default;
  std::string Serialize() override;
  bool DeSerialize(const std::string &data) override;
};
using StatisticsMessagePtr = std::shared_ptr<StatisticsMessage>;
}  // namespace message
}  // namespace xproto

#endif  // XPROTO_PLUGINS_MESSAGE_STATISTICS_MESSAGE_H_
