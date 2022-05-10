/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @file main.cpp
 * @brief
 * @author hangjun.yang
 * @email hangjun.yang@horizon.ai
 *
 *
 * */

#include <iostream>
#include "hobotlog/hobotlog.hpp"
#include "video_source_plugin/video_source_plugin.h"
#include "example_smart_plugin.h"
#include "example_web_display_plugin.h"
#include "bpu_predict_extension.h"

using xproto::VideoSourcePlugin;
using solution::yolov3_mobilenetv2::ExampleSmartPlugin;
using solution::yolov3_mobilenetv2::ExampleWebDisplayPlugin;

static bool exit_ = false;
static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    exit_ = true;
  }
}

int main(int argc, char **argv) {
  HB_BPU_setGlobalConfig(BPU_GLOBAL_CONFIG_MAX_TASK_NUM, "128");
  std::string video_source_config_file = std::string(argv[1]);
  std::string smart_config_file = std::string(argv[2]);
  std::string websocket_config_file = std::string(argv[3]);

  std::string log_level(argv[4]);
  if (log_level == "-i") {
    SetLogLevel(HOBOT_LOG_INFO);
  } else if (log_level == "-d") {
    SetLogLevel(HOBOT_LOG_DEBUG);
  } else if (log_level == "-w") {
    SetLogLevel(HOBOT_LOG_WARN);
  } else if (log_level == "-e") {
    SetLogLevel(HOBOT_LOG_ERROR);
  } else if (log_level == "-f") {
    SetLogLevel(HOBOT_LOG_FATAL);
  } else {
    LOGE << "log option: [-i/-d/-w/-f] ";
    return 0;
  }

  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);

  // 创建plugin对象
  auto video_source_plg = std::make_shared<VideoSourcePlugin>(
      video_source_config_file);
  auto smart_plg = std::make_shared<ExampleSmartPlugin>(smart_config_file);
  auto websocket_plg =
  std::make_shared<ExampleWebDisplayPlugin>(websocket_config_file);

  auto ret = video_source_plg->Init();
  if (ret != 0) {
    LOGE << "Failed to init video_source";
    return -1;
  }

  ret = smart_plg->Init();
  if (ret != 0) {
    return -1;
  }

  ret = websocket_plg->Init();
  if (ret != 0) {
    return -1;
  }

  video_source_plg->Start();
  smart_plg->Start();
  websocket_plg->Start();

  while (!exit_) {
    std::this_thread::sleep_for(std::chrono::microseconds(40));
  }

  video_source_plg->Stop();
  video_source_plg->DeInit();
  video_source_plg = nullptr;
  smart_plg->Stop();
  websocket_plg->Stop();
  smart_plg->DeInit();
  websocket_plg->DeInit();
  return 0;
}
