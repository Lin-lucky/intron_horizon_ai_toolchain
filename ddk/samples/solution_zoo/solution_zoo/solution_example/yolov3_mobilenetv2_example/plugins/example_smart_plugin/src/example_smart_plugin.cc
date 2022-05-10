/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file example_smart_plugin.cc
 * @brief
 * @author ronghui.zhang
 * @email ronghui.zhang@horizon.ai
 *
 *
 * */
#include "example_smart_plugin.h"
#include <string>

namespace solution {
namespace yolov3_mobilenetv2 {

ExampleSmartPlugin::ExampleSmartPlugin(const std::string &config_file)
: SmartPlugin(config_file) {
}

}  // namespace yolov3_mobilenetv2
}  // namespace solution
