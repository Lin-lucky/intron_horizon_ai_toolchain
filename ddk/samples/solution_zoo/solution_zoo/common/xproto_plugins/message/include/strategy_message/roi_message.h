/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief: roiplugin impl based on xpp.
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#ifndef XPROTO_MESSAGE_STRATEGY_ROI_MESSAGE_H_
#define XPROTO_MESSAGE_STRATEGY_ROI_MESSAGE_H_

#include <memory>
#include <string>

#include "xproto/message/pluginflow/flowmsg.h"

namespace xproto {
namespace message {

#define TYPE_ROI_MESSAGE "XPLUGIN_ROI_MESSAGE"

struct RoiMessage : XProtoMessage {
  int channel_id;
  int frame_fps;
  uint64_t time_stamp;
  uint64_t frame_id;
  std::string image_name;
  RoiMessage() { type_ = TYPE_ROI_MESSAGE; }
  virtual ~RoiMessage() = default;

  std::string Serialize() override { return "Default roi smart message"; };
  virtual std::string Serialize(int ori_w, int ori_h, int dst_w, int dst_h) {
    return "";
  }
};
using RoiMessagePtr = std::shared_ptr<RoiMessage>;
}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MESSAGE_STRATEGY_ROI_MESSAGE_H_
