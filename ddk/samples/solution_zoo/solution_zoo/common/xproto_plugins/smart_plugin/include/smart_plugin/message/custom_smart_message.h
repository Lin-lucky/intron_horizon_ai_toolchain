/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: zhe.sun
 * @Date: 2021-03-19 16:01:23
 * @Version: v0.0.1
 * @Brief: CustomSmartMessage declaration
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2021-03-19 16:41:22
 */

#ifndef INCLUDE_SMARTPLUGIN_CUSTOM_SMART_MESSAGE_H_
#define INCLUDE_SMARTPLUGIN_CUSTOM_SMART_MESSAGE_H_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "smart_message/smart_message.h"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "json/json.h"

namespace xproto {
namespace message {

struct CustomSmartMessage : SmartMessage {
  explicit CustomSmartMessage(
    xstream::OutputDataPtr out) : smart_result(out) {
    type_ = TYPE_SMART_MESSAGE;
  }
  std::string Serialize() override;
  std::string Serialize(int ori_w, int ori_h, int dst_w, int dst_h) override;
  void Serialize_Print(Json::Value &root);
  void Serialize_Dump_Result();
  const xstream::OutputDataPtr& GetSmartResult() const {
    return smart_result;
  }

 protected:
  xstream::OutputDataPtr smart_result;

 private:
  static std::mutex static_attr_mutex_;
};
}  // namespace message
}  // namespace xproto
#endif  // INCLUDE_SMARTPLUGIN_CUSTOM_SMART_MESSAGE_H_
