/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file example_web_display_plugin.h
 * @brief
 * @author ronghui.zhang
 * @email ronghui.zhang@horizon.ai
 *
 *
 * */
#ifndef INCLUDE_EXAMPLE_WEB_DISPLAY_PLUGIN_H_
#define INCLUDE_EXAMPLE_WEB_DISPLAY_PLUGIN_H_

#include <string>

#include "web_display_plugin/web_display_plugin.h"
#include "xproto/msg_type/smart_legible_message.h"

namespace solution {
namespace yolov3_mobilenetv2 {

using xproto::WebDisplayPlugin;

class ExampleWebDisplayPlugin : public WebDisplayPlugin {
 public:
  explicit ExampleWebDisplayPlugin(std::string config_path);

 protected:
  std::string GetSmartMessageType() {
    // return the message type from exampel_smart_plugin
    return TYPE_SMART_LEGIBLE_MESSAGE;
  }
};
}  // namespace yolov3_mobilenetv2
}  // namespace solution
#endif  // INCLUDE_EXAMPLE_WEB_DISPLAY_PLUGIN_H_
