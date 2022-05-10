/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include <malloc.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <sstream>
// #include "hbipcplugin/hbipcplugin.h"
#include "bpu_predict_extension.h"
#include "hobotlog/hobotlog.hpp"
#include "rtsp_plugin/rtsp_plugin.h"
#include "smart_plugin/smart_plugin.h"
#include "visual_plugin/visual_plugin.h"
extern "C" {
#include "xwarehouse/xwarehouse.h"
#include "xwarehouse/xwarehouse_data.h"
}
#include "xware_plugin/ware_plugin.h"

using std::chrono::seconds;
using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;
// using horizon::vision::xproto::hbipcplugin::HbipcPlugin;
using solution::video_box::RtspPlugin;
using solution::video_box::SmartPlugin;
using solution::video_box::WarePlugin;

// using xproto::VioPlugin;
using xproto::VisualPlugin;

static bool exit_ = false;

static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    exit_ = true;
  }
  if (param == SIGSEGV) {
    std::cout << "recv segment fault, exit exe, maybe need reboot..."
              << std::endl;
    exit(-1);
  }
}

int main(int argc, char **argv) {
  HB_BPU_setGlobalConfig(BPU_GLOBAL_CONFIG_MAX_TASK_NUM, "128");
  auto mallopt_option = getenv("MALLOC");
  auto bpu_engine_option = getenv("BPU_ENGINE");
  if (mallopt_option && !strcmp(mallopt_option, "OFF")) {
    std::cout << "turn off mallopt" << std::endl;
  } else {
    // default use mallopt
    std::cout << "turn on mallopt" << std::endl;
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
  }
  if (bpu_engine_option && !strcmp(bpu_engine_option, "group")) {
    std::cout << "use bpu group engine" << std::endl;
    HB_BPU_setGlobalConfig(BPU_GLOBAL_ENGINE_TYPE, "group");
  } else {
    // default use native engine
    std::cout << "use bpu native engine" << std::endl;
    HB_BPU_setGlobalConfig(BPU_GLOBAL_ENGINE_TYPE, "native");
  }

  std::string run_mode = "ut";

  if (argc < 5) {
    std::cout << "Usage: smart_main vio_config_file "
              << "xstream_config_file visualplugin_config "
              << "[-i/-d/-w/-f] " << std::endl;
    return 0;
  }
  std::string box_config_file = std::string(argv[1]);
  std::string visual_config_file = std::string(argv[2]);
  std::string xware_config_file = std::string(argv[3]);
  std::string vio_config_file;
  //  std::shared_ptr<RtspPlugin> rtsp_plg;

  // std::shared_ptr<VioPlugin> vio_plg;

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

  if (argc >= 6) {
    run_mode.assign(argv[5]);
  }
  auto ware_plg = std::make_shared<WarePlugin>(xware_config_file);
  if (ware_plg) {
    ware_plg->Init();
    ware_plg->Start();
  }
  if (run_mode == "db_list") {
    if (ware_plg) {
      ware_plg->db_list();
      ware_plg->Stop();
      ware_plg->DeInit();
    }
    return 0;
  } else if (run_mode == "db_drop_table") {
    if (ware_plg) {
      ware_plg->db_table_clear();
      ware_plg->Stop();
      ware_plg->DeInit();
    }
    return 0;
  }

  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);

  auto rtsp_plg = std::make_shared<RtspPlugin>(box_config_file);
  auto visual_plg = std::make_shared<VisualPlugin>(visual_config_file);
  if (visual_plg) visual_plg->Init();
  if (visual_plg) visual_plg->Start();
  sleep(5);

  auto smart_plg = std::make_shared<SmartPlugin>(box_config_file);
  auto ret = smart_plg->Init();
  if (ret != 0) {
    LOGE << "Failed to init smart plugin";
    return 2;
  }
  smart_plg->Start();
  if (run_mode == "ruku") {
    LOGF << "do not support yet";
    // if (argc == 7) {
    //   vio_config_file = std::string(argv[6]);
    //   vio_plg = std::make_shared<VioPlugin>(vio_config_file);
    //   if (vio_plg) {
    //     vio_plg->Init();
    //     vio_plg->Start();
    //   } else {
    //     return 1;
    //   }
    // }
  } else if ((run_mode == "ut") || (run_mode == "normal")) {
    rtsp_plg->Init();
    auto ret = rtsp_plg->Start();
    if (ret < 0) {
      LOGE << "Failed to start rtsp plugin ret:" << ret;
      rtsp_plg->Stop();
      if (-2 == ret) {
        LOGE << "ERROR!!! open rtsp fail, please check url";
      }
      return 0;
    } else {
      LOGI << "rtsp plugin start success";
    }
  }

  if (run_mode == "ut") {
    std::this_thread::sleep_for(std::chrono::seconds(30));
  } else {
    while (!exit_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
      LOGD << "wait to quit";
    }
  }
  rtsp_plg->Stop();
  smart_plg->Stop();
  if (visual_plg) visual_plg->Stop();
  rtsp_plg->DeInit();
  smart_plg->DeInit();
  visual_plg->DeInit();
  return 0;
}
