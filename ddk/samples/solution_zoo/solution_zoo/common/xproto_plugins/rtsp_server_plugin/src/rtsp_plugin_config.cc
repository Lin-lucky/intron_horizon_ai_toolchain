/*
 * @Copyright 2020 Horizon Robotics, Inc.
 */
#include <fstream>
#include <iostream>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "rtsp_plugin_config.h"

namespace xproto {
// rtsp plugin config
RTSPPluginConfig::RTSPPluginConfig(const std::string &path) : path_(path) {
  layer_ = 4;
  enable_smart_ = 0;
  image_width_ = 1920;
  image_height_ = 1080;
  data_buf_size_ = 3110400;
  packet_size_ = 102400;
  is_cbr_ = 1;
  bitrate_ = 6000;
  rotation_ = 0;
  mirror_ = 0;
}

bool RTSPPluginConfig::LoadConfig() {
  std::ifstream ifs(path_);
  if (!ifs.is_open()) {
    LOGF << "open config file " << path_ << " failed";
    return false;
  }

  Json::CharReaderBuilder builder;
  std::string err_json;
  try {
    bool ret = Json::parseFromStream(builder, ifs, &json_, &err_json);
    if (!ret) {
      LOGF << "invalid config file " << path_;
      return false;
    }
  } catch (std::exception &e) {
    LOGF << "exception while parse config file " << path_ << ", " << e.what();
    return false;
  }

  return CheckConfig();
}

std::string RTSPPluginConfig::GetValue(const std::string &key) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (json_[key].empty()) {
    LOGW << "Can not find key: " << key;
    return "";
  }

  return json_[key].asString();
}

Json::Value RTSPPluginConfig::GetJson() const { return this->json_; }

bool RTSPPluginConfig::CheckConfig() {
  if (json_.isMember("layer")) {
    layer_ = json_["layer"].asUInt();
  }

  if (json_.isMember("enable_smart")) {
    enable_smart_ = json_["enable_smart"].asUInt();
  }

  if (json_.isMember("video_type")) {
    int type = json_["video_type"].asUInt();
    if (type == 0) {
      video_type_ = H264;
    } else if (type == 1) {
      video_type_ = H265;
    }
  }

  if (json_.isMember("data_buf_size")) {
    data_buf_size_ = json_["data_buf_size"].asUInt();
  }
  if (json_.isMember("packet_size")) {
    packet_size_ = json_["packet_size"].asUInt();
  }

  if (json_.isMember("image_width")) {
    image_width_ = json_["image_width"].asUInt();
  }
  if (json_.isMember("image_height")) {
    image_height_ = json_["image_height"].asUInt();
  }
  if (json_.isMember("frame_buf_depth")) {
    frame_buf_depth_ = json_["frame_buf_depth"].asUInt();
  }

  if (json_.isMember("debug_encode_cost")) {
    debug_encode_cost_ = json_["debug_encode_cost"].asUInt();
  }
  if (json_.isMember("debug_dump_stream")) {
    debug_dump_stream_ = json_["debug_dump_stream"].asUInt();
  }

  if (json_.isMember("use_vb")) {
    use_vb_ = json_["use_vb"].asUInt();
  }

  if (json_.isMember("rotation")) {
    rotation_ = json_["rotation"].asInt();
  }

  if (json_.isMember("mirror")) {
    mirror_ = json_["mirror"].asInt();
  }

  if (json_.isMember("is_cbr")) {
    is_cbr_ = json_["is_cbr"].asInt();
  }

  if (json_.isMember("bitrate")) {
    bitrate_ = json_["bitrate"].asInt();
  }

  if (json_.isMember("rtsp_server_config")) {
    rtsp_server_config_ = json_["rtsp_server_config"].asString();
  }
  // check the value
  // to do ..
  return true;
}
}  // namespace xproto
