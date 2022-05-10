/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Qingpeng Liu
 * @Mail: qingpeng.liu@horizon.ai
 * @Date: 2021-03-15 14:00:22
 * @Version: v0.0.1
 * @Brief: SmartFeatureMessage declaration
 * @Last Modified by: Qingpeng Liu
 * @Last Modified time: 2021-03-15 15:45:01
 */
#ifndef VIDEO_BOX_INCLUDE_MESSGAGE_WARE_RECOG_MESSAGE_H_
#define VIDEO_BOX_INCLUDE_MESSGAGE_WARE_RECOG_MESSAGE_H_
#include <memory>
#include <string>

#include "message/common_data_type/recog_result.h"
#include "xproto/xproto_world.h"

namespace solution {
namespace video_box {
using xproto::XProtoMessage;

#define TYPE_RECOG_MESSAGE "XPLUGIN_RECOG_MESSAGE"

class WareRecogMessage : public XProtoMessage {
 public:
  WareRecogMessage() = delete;
  explicit WareRecogMessage(std::shared_ptr<RecogResult> data) {
    type_ = TYPE_RECOG_MESSAGE;
    message_data_ = data;
  }
  ~WareRecogMessage() = default;
  std::shared_ptr<RecogResult> GetMessageData() { return message_data_; }
  std::string Serialize() { return "WareRecogMessage don't need serialize"; }

 private:
  std::shared_ptr<RecogResult> message_data_;
};

using WareRecogMessagePtr = std::shared_ptr<WareRecogMessage>;
}  // namespace video_box
}  // namespace solution
#endif  // VIDEO_BOX_INCLUDE_MESSGAGE_WARE_RECOG_MESSAGE_H_
