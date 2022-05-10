/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-09-29 01:50:41
 * @Version: v0.0.1
 * @Brief: smart message declaration.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 02:49:28
 */

#ifndef XPROTO_MESSAGE_SMART_MESSAGE_H_
#define XPROTO_MESSAGE_SMART_MESSAGE_H_

#include <memory>
#include <string>

#include "xproto/message/flowmsg.h"
#include "xstream/xstream_world.h"

namespace xproto {
namespace message {

#define TYPE_SMART_MESSAGE "XPLUGIN_SMART_MESSAGE"

struct SmartMessage : public xproto::XProtoMessage {
  int channel_id_;
  int frame_fps_;  // record fps
  uint64_t time_stamp_;
  uint64_t frame_id_;
  std::string image_name_;
  bool ap_mode_ = false;
  float matting_trimapfree_expansion_ratio_ = 0.2;
  SmartMessage() { type_ = TYPE_SMART_MESSAGE; }
  virtual ~SmartMessage() = default;

  std::string Serialize() override { return "Default smart message"; };
  virtual std::string Serialize(int ori_w, int ori_h, int dst_w, int dst_h) {
    return "";
  }
  virtual void SetExpansionRatio(float expansion_ratio) {
    matting_trimapfree_expansion_ratio_ = expansion_ratio;
  }
  virtual void SetAPMode(bool ap_mode) { ap_mode_ = ap_mode; }

  const xstream::OutputDataPtr& GetSmartResult() const {}
};
using SmartMessagePtr = std::shared_ptr<SmartMessage>;
}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MESSAGE_SMART_MESSAGE_H_
