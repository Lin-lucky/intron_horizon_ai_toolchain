/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     example_plugin.h
 * \Author   xudong.du
 * \Version  1.0.0.0
 * \Date     2021.4.6
 * \Brief    implement of api file
 */
#ifndef INCLUDE_EXAMPLE_PLUGIN_EXAMPLE_PLUGIN_
#define INCLUDE_EXAMPLE_PLUGIN_EXAMPLE_PLUGIN_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "json/json.h"
#include "xproto/plugin/xpluginasync.h"

namespace xproto {

class ExamplePlugin : public xproto::XPluginAsync {
 public:
  ExamplePlugin() = delete;
  explicit ExamplePlugin(std::string config_path);
  ~ExamplePlugin();
  int Init() override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "ExamplePlugin"; }

 private:
  int FeedSmart(XProtoMessagePtr msg);
};
}  // namespace xproto

#endif  // INCLUDE_EXAMPLE_PLUGIN_EXAMPLE_PLUGIN_
