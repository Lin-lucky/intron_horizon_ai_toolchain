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

#ifndef VIDEO_BOX_INCLUDE_MESSGAGE_\
SMART_FEATURE_MESSAGE_SMART_FEATURE_MESSAGE_H_
#define VIDEO_BOX_INCLUDE_MESSGAGE_\
SMART_FEATURE_MESSAGE_SMART_FEATURE_MESSAGE_H_

#include <memory>
#include <string>

#include "xproto/xproto_world.h"
#include "feature_frame_message/feature_frame_message.h"

namespace solution {
namespace video_box {

#define TYPE_SMART_FEATURE_MESSAGE "XPLUGIN_SMART_FEATURE_MESSAGE"
class SmartFeatureMessage : public XProtoMessage {
 public:
  explicit SmartFeatureMessage(std::shared_ptr<FeatureFrameMessage> features) {
    type_ = TYPE_SMART_FEATURE_MESSAGE;
    features_ = features;
  }
  ~SmartFeatureMessage() = default;
  std::shared_ptr<FeatureFrameMessage> GetMessageData() { return features_; }
  std::string& GetMessageType() { return message_type_; }
  void SetMessageType(std::string msg_type) { message_type_ = msg_type; }
  std::string Serialize() override { return "no need serial"; }

 private:
  std::shared_ptr<FeatureFrameMessage> features_;
  std::string message_type_;
};

using SmartFeatureMessagePtr = std::shared_ptr<SmartFeatureMessage>;
}  // namespace video_box
}  // namespace solution

#endif  //  VIDEO_BOX_INCLUDE_MESSGAGE_\
SMART_FEATURE_MESSAGE_SMART_FEATURE_MESSAGE_H_
