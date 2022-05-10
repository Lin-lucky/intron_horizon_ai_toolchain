/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file ExampleSmartplugin.h
 * @brief
 * @author ronghui.zhang
 * @email ronghui.zhang@horizon.ai
 *
 *
 * */
#ifndef INCLUDE_EXAMPLE_SMART_PLUGIN_H_
#define INCLUDE_EXAMPLE_SMART_PLUGIN_H_

#include <string>
#include <memory>
#include "xproto/plugin/xpluginasync.h"
#include "smart_plugin/smart_plugin.h"
#include "xstream/xstream_world.h"

namespace solution {
namespace yolov3_mobilenetv2 {

using xproto::SmartPlugin;

class ExampleSmartPlugin : public SmartPlugin {
 public:
  explicit ExampleSmartPlugin(const std::string& config_file);
 private:
  std::string GetWorkflowInputImageName() {
    return "image";
  }
};
}  // namespace yolov3_mobilenetv2
}  // namespace solution
#endif  // INCLUDE_EXAMPLE_SMART_PLUGIN_H_
