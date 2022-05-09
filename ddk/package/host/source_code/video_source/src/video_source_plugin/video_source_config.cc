/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source_plugin/video_source_config.h"
#include "hobotlog/hobotlog.hpp"

namespace xproto {

std::string VideoSourceConfig::GetStringValue(const std::string &key) const {
  std::lock_guard<std::mutex> lk(mutex_);
  if (json_[key].empty()) {
    LOGW << "Can not find key: " << key;
    return "";
  }

  return json_[key].asString();
}

int VideoSourceConfig::GetIntValue(const std::string &key) const {
  std::lock_guard<std::mutex> lk(mutex_);
  if (json_[key].empty()) {
    LOGW << "Can not find key: " << key;
    return -1;
  }

  return json_[key].asInt();
}

std::vector<int> VideoSourceConfig::GetIntArrayItem(std::string key) const {
  std::vector<int> result;
  auto value_js = json_[key.c_str()];
  if (value_js.isNull()) {
    value_js = Json::Value(0);
  }

  if (value_js.isInt()) {
    auto item_list_obj = Json::Value();
    item_list_obj.resize(1);
    item_list_obj[0] = value_js.asInt();
    value_js = item_list_obj;
  }
  for (unsigned int i = 0; i < value_js.size(); i++) {
    result.push_back(value_js[i].asInt());
  }
  return result;
}


std::vector<std::string> VideoSourceConfig::GetStringArrayItem(
    std::string key) const {
  std::vector<std::string> result;
  auto value_js = json_[key.c_str()];
  if (value_js.isNull()) {
    value_js = Json::Value("");
  }

  if (value_js.isString()) {
    auto item_list_obj = Json::Value();
    item_list_obj.resize(1);
    item_list_obj[0] = value_js.asString();
    value_js = item_list_obj;
  }
  for (unsigned int i = 0; i < value_js.size(); i++) {
    result.push_back(value_js[i].asString());
  }
  return result;
}

std::shared_ptr<VideoSourceConfig> VideoSourceConfig::GetSubConfig(
    std::string key) {
  auto value_js = json_[key.c_str()];
  if (value_js.isNull()) {
    return nullptr;
  }
  return std::shared_ptr<VideoSourceConfig>(new VideoSourceConfig(value_js));
}

std::shared_ptr<VideoSourceConfig> VideoSourceConfig::GetSubConfig(int key) {
  auto value_js = json_[key];
  if (value_js.isNull()) {
    return nullptr;
  }
  return std::shared_ptr<VideoSourceConfig>(new VideoSourceConfig(value_js));
}

}  // namespace xproto
