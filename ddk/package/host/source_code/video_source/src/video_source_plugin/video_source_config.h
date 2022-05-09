/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_CONFIG_H_
#define VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_CONFIG_H_
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include "json/json.h"

namespace xproto {

class VideoSourceConfig {
 public:
  VideoSourceConfig() = default;
  explicit VideoSourceConfig(const Json::Value &json) : json_(json) {}
  std::string GetStringValue(const std::string &key) const;
  int GetIntValue(const std::string &key) const;
  std::vector<std::string> GetStringArrayItem(std::string key) const;
  std::vector<int> GetIntArrayItem(std::string key) const;
  std::shared_ptr<VideoSourceConfig> GetSubConfig(std::string key);
  std::shared_ptr<VideoSourceConfig> GetSubConfig(int key);
  Json::Value GetJson() const { return this->json_; }
  bool HasMember(std::string key) { return json_.isMember(key); }
  int ItemCount(void) { return json_.size(); }

 private:
  Json::Value json_;
  mutable std::mutex mutex_;
};

}  // namespace xproto
#endif  // VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_PLUGIN_H_
