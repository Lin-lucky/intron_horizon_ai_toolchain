/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     Web_Display_Plugin.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020.5.12
 * \Brief    implement of api file
 */
#include "web_display_plugin/web_display_plugin.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "utils/time_helper.h"
#include "web_display_plugin/convert.h"
#include "web_display_plugin/server/uws_server.h"
#include "web_display_plugin/web_display_config.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/msg_type/vio_message.h"
#include "xproto/xproto_world.h"

using hobot::Timer;
namespace xproto {
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_UWS_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_MESSAGE)
using std::chrono::milliseconds;
bool write_flag = true;

WebDisplayPlugin::WebDisplayPlugin(std::string config_file) {
  config_file_ = config_file;
  LOGI << "UwsPlugin smart config file:" << config_file_;
  smart_stop_flag_ = false;
  Reset();
}

WebDisplayPlugin::~WebDisplayPlugin() {
  config_ = nullptr;
}

int WebDisplayPlugin::Init() {
  LOGI << "WebDisplayPlugin::Init";
  // load config
  config_ = std::make_shared<WebDisplayConfig>(config_file_);
  if (!config_ || !config_->LoadConfig()) {
    LOGE << "failed to load config file";
    return -1;
  }

  LOGI << "attribute_description_path: " << config_->attr_des_file_;

  RegisterMsg(GetSmartMessageType(), std::bind(&WebDisplayPlugin::FeedSmart,
                                               this, std::placeholders::_1));
  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();
  data_send_thread_.CreatThread(1);
  return 0;
}

int WebDisplayPlugin::Reset() {
  LOGI << __FUNCTION__ << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
  std::unique_lock<std::mutex> lock(map_smart_mutex_);
  while (!x3_smart_msg_.empty()) {
    x3_smart_msg_.pop();
  }
  return 0;
}

int WebDisplayPlugin::Start() {
  // start websocket server
  uws_server_ = std::make_shared<UwsServer>();
  if (uws_server_->Init()) {
    LOGE << "UwsPlugin Init uWS server failed";
    return -1;
  }
  if (!worker_) {
    if (GetSmartMessageType() == TYPE_SMART_LEGIBLE_MESSAGE) {
      worker_ = std::make_shared<std::thread>(
          std::bind(&WebDisplayPlugin::map_smart_proc, this));
    } else {
      LOGE << "Not support type : " << GetSmartMessageType();
    }
  }
  return 0;
}

void WebDisplayPlugin::map_smart_proc() {
  while (!map_stop_) {
    std::unique_lock<std::mutex> lock(map_smart_mutex_);
    map_smart_condition_.wait(lock);
    if (map_stop_) {
      break;
    }
    if (x3_smart_msg_.size() > 3) {
      LOGW << "map_proc, smart_msg size = " << x3_smart_msg_.size();
    }
    while (!x3_smart_msg_.empty()) {
      auto msg = x3_smart_msg_.top();  // 优先级队列已保证有序
      int task_num = data_send_thread_.GetTaskNum();
      if (task_num < 3) {
        data_send_thread_.PostTask(
            std::bind(&WebDisplayPlugin::SendSmartMessage, this, msg));
      }
      x3_smart_msg_.pop();
    }
  }
}

int WebDisplayPlugin::Stop() {
  {
    std::lock_guard<std::mutex> smart_lock(smart_mutex_);
    smart_stop_flag_ = true;
  }
  {
    if (worker_ && worker_->joinable()) {
      map_stop_ = true;
      map_smart_condition_.notify_one();
      worker_->join();
      worker_ = nullptr;
      LOGI << "WebDisplayPlugin stop worker";
    }
  }
  LOGI << "WebDisplayPlugin::Stop()";
  uws_server_->DeInit();
  return 0;
}

int WebDisplayPlugin::FeedSmart(XProtoMessagePtr msg) {
  {
    std::lock_guard<std::mutex> smart_lock(smart_mutex_);
    if (smart_stop_flag_) {
      LOGD << "Aleardy stop, WebDisplayPlugin FeedSmart return";
      return -1;
    }
  }
  auto smart_msg = std::static_pointer_cast<SmartLegibleMessage>(msg);
  if (smart_msg) {
    {
      std::lock_guard<std::mutex> smart_lock(map_smart_mutex_);
      x3_smart_msg_.push(smart_msg);
    }
    map_smart_condition_.notify_one();
  }

  return 0;
}

int WebDisplayPlugin::SendSmartMessage(XProtoMessagePtr smart_msg) {
  std::string protocol;
  auto ret = Convertor::PackSmartMsg(protocol, smart_msg);
  if (0 != ret) {
    LOGE << "PackSmartMsg failed.";
    return -1;
  }
  // smart frame
  x3::FrameMessage msg_send;
  msg_send.ParseFromString(protocol);
  // add system info
  std::string cpu_rate_file =
      "/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq";
  std::string temp_file = "/sys/class/thermal/thermal_zone0/temp";
  std::ifstream ifs(cpu_rate_file.c_str());
  if (!ifs.is_open()) {
    LOGF << "open config file " << cpu_rate_file << " failed";
    return -1;
  }
  std::stringstream ss;
  std::string str;
  ss << ifs.rdbuf();
  ss >> str;
  ifs.close();
  auto Statistics_msg_ = msg_send.mutable_statistics_msg_();
  auto attrs = Statistics_msg_->add_attributes_();
  attrs->set_type_("cpu");
  attrs->set_value_string_(str.c_str());

  ifs.clear();
  ss.clear();
  ifs.open(temp_file.c_str());
  ss << ifs.rdbuf();
  ss >> str;

  auto temp_attrs = Statistics_msg_->add_attributes_();
  temp_attrs->set_type_("temp");
  temp_attrs->set_value_string_(str.c_str());
  std::string proto_send;
  msg_send.SerializeToString(&proto_send);
  uws_server_->Send(proto_send);
  return XPluginErrorCode::ERROR_CODE_OK;
}
}  // namespace xproto
