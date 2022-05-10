/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file main.cpp
 * @brief
 * @author fei.cheng
 * @email fei.cheng@horizon.ai
 *
 *
 * */

#include <malloc.h>
#include <signal.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "json/json.h"
#include "smart_plugin/smart_plugin.h"
#include "video_source_plugin/video_source_plugin.h"
#include "visual_plugin/visual_plugin.h"
#include "bpu_predict_extension.h"
#include "web_display_plugin/web_display_plugin.h"
#include "rtsp_server_plugin.h"

#include "uvc_display_plugin/uvc_display_plugin.h"
#ifdef BUILD_WITH_AUDIO
#include "audio_capture_plugin/audio_capture_plugin.h"
#endif

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;
using std::chrono::seconds;

#ifdef BUILD_WITH_AUDIO
using xproto::AudioCapturePlugin;
#endif
using xproto::SmartPlugin;
using xproto::UvcPlugin;
using xproto::VideoSourcePlugin;
using xproto::VisualPlugin;
using xproto::WebDisplayPlugin;
using xproto::RTSPServerPlugin;
static bool exit_ = false;

static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    exit_ = true;
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
  if (bpu_engine_option && !strcmp(bpu_engine_option, "native")) {
    std::cout << "use bpu native engine" << std::endl;
    HB_BPU_setGlobalConfig(BPU_GLOBAL_ENGINE_TYPE, "native");
  } else {
    // default use group engine
    std::cout << "use bpu group engine" << std::endl;
    HB_BPU_setGlobalConfig(BPU_GLOBAL_ENGINE_TYPE, "native");
  }

  std::string run_mode = "ut";

  if (argc < 5) {
    std::cout << "Usage: smart_main video_source_config_file "
              << "xstream_config_file visualplugin_config "
              << "[-i/-d/-w/-f] " << std::endl;
    return 0;
  }

  std::string video_source_config_file = std::string(argv[1]);
  std::string smart_config_file = std::string(argv[2]);
  std::string display_config_file = std::string(argv[3]);

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

  if (argc == 6) {
    run_mode.assign(argv[5]);
    if (run_mode != "ut" && run_mode != "normal") {
      LOGE << "not support mode: " << run_mode;
      return 0;
    }
  }

  // parse output display mode config
  int display_mode = -1;
  std::string display_plugin_config;
  std::string visual_plugin_config;
  std::string web_plugin_config;
  std::string uvc_plugin_config;
  std::string rtsp_plugin_config;

  std::shared_ptr<VisualPlugin> visual_plugin = nullptr;
  std::shared_ptr<WebDisplayPlugin> web_display_plugin = nullptr;
  std::shared_ptr<UvcPlugin> uvc_display_plugin = nullptr;
  std::shared_ptr<RTSPServerPlugin> rtsp_display_plugin = nullptr;
  std::vector<std::shared_ptr<XPluginAsync>> display_plugins;

  int audio_enable = 0;
  std::string audio_plugin_config;
  std::ifstream ifs(display_config_file);
  if (!ifs.is_open()) {
    LOGF << "open config file " << display_config_file << " failed";
    return 0;
  }

  Json::CharReaderBuilder builder;
  std::string err_json;
  Json::Value json_obj;
  try {
    bool ret = Json::parseFromStream(builder, ifs, &json_obj, &err_json);
    if (!ret) {
      LOGF << "invalid config file " << display_config_file;
      return 0;
    }
  } catch (std::exception &e) {
    LOGF << "exception while parse config file " << display_config_file << ", "
         << e.what();
    return 0;
  }

  if (json_obj.isMember("display_mode")) {
    auto display_mode = json_obj["display_mode"];
    if (!display_mode.isNull()) {
      int display_num = display_mode.size();
      for (int i = 0; i < display_num; i++) {
        int mode = display_mode[i].asInt();
        switch (mode) {
          case 0:
            visual_plugin_config = json_obj["visual_plugin_config"].asString();
            visual_plugin =
              std::make_shared<VisualPlugin>(visual_plugin_config);
            display_plugins.push_back(visual_plugin);
            break;
          case 1:
            web_plugin_config = json_obj["web_plugin_config"].asString();
            web_display_plugin =
              std::make_shared<WebDisplayPlugin>(web_plugin_config);
            display_plugins.push_back(web_display_plugin);
            break;
          case 2:
            uvc_plugin_config = json_obj["uvc_plugin_config"].asString();
            uvc_display_plugin =
              std::make_shared<UvcPlugin>(uvc_plugin_config);
            display_plugins.push_back(uvc_display_plugin);
            break;
          case 3:
            rtsp_plugin_config = json_obj["rtsp_plugin_config"].asString();
            rtsp_display_plugin =
              std::make_shared<RTSPServerPlugin>(rtsp_plugin_config);
            display_plugins.push_back(rtsp_display_plugin);
            break;
        }
      }
    }
  } else {
    LOGF << display_config_file << " should set display mode";
    return 0;
  }

  if (json_obj.isMember("audio_enable")) {
    audio_enable = json_obj["audio_enable"].asInt();
  }
  if (audio_enable) {
    audio_plugin_config = json_obj["audio_plugin"].asString();
  }

  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);
#ifdef BUILD_WITH_AUDIO
    std::shared_ptr<AudioCapturePlugin> audio_capture_plg = nullptr;

    if (audio_enable) {
      audio_capture_plg =
        std::make_shared<AudioCapturePlugin>(audio_plugin_config);
    }
#endif
  auto video_source_plg = std::make_shared<VideoSourcePlugin>(
      video_source_config_file);
  auto smart_plg = std::make_shared<SmartPlugin>(smart_config_file);

  auto ret = video_source_plg->Init();
  if (ret != 0) {
    LOGE << "Failed to init video_source_plugin";
    return 1;
  }
#ifdef BUILD_WITH_AUDIO
  if (audio_enable && audio_capture_plg) {
    ret = audio_capture_plg->Init();
    if (ret != 0) {
      LOGE << "Failed to init audio capture plugin";
      return 1;
    }
  }
#endif
  ret = smart_plg->Init();
  if (ret != 0) {
    LOGE << "Failed to init smart plugin";
    return 2;
  }

  for (size_t i = 0; i < display_plugins.size(); i++) {
    ret = display_plugins[i]->Init();
    if (ret != 0) {
      LOGE << "output plugin Init failed";
      return 3;
    }
    ret = display_plugins[i]->Start();
    if (ret != 0) {
      LOGE << "output plugin Start failed";
      return 3;
    }
  }

  video_source_plg->Start();

#ifdef BUILD_WITH_AUDIO
  if (audio_enable && audio_capture_plg) {
    audio_capture_plg->Start();
  }
#endif
  smart_plg->Start();

  if (run_mode == "ut") {
    std::this_thread::sleep_for(std::chrono::seconds(60));
  } else {
    while (!exit_) {
      std::this_thread::sleep_for(std::chrono::microseconds(40));
    }
  }

  for (size_t i = 0; i < display_plugins.size(); i++) {
    ret = display_plugins[i]->Stop();
    if (ret != 0) {
      LOGE << "output plugin Stop failed";
      return 3;
    }
    ret = display_plugins[i]->DeInit();
    if (ret != 0) {
      LOGE << "output plugin DeInit failed";
      return 3;
    }
  }

  smart_plg->Stop();
  smart_plg->DeInit();

#ifdef BUILD_WITH_AUDIO
  if (audio_enable && audio_capture_plg) {
    audio_capture_plg->Stop();
    audio_capture_plg->DeInit();
    audio_capture_plg = nullptr;
  }
#endif

  video_source_plg->Stop();
  video_source_plg->DeInit();
  video_source_plg = nullptr;
  return 0;
}
