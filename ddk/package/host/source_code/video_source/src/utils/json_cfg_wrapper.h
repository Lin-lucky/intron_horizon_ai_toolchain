/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_COMMON_JSON_CFG_WRAPPER_H_
#define VIDEO_SOURCE_COMMON_JSON_CFG_WRAPPER_H_
#include <string.h>
#include <memory>
#include <string>
#include <vector>
#include "json/json.h"

namespace videosource {

class JsonConfigWrapper {
 public:
  explicit JsonConfigWrapper(Json::Value config) : config_(config) {}

  int GetIntValue(std::string key, int default_value = 0) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asInt();
  }

  bool GetBoolValue(std::string key, bool default_value = false) {
    auto value_int = GetIntValue(key, default_value);
    return value_int == 0 ? false : true;
  }

  float GetFloatValue(std::string key, float default_value = 0.0) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asFloat();
  }

  std::string GetSTDStringValue(std::string key,
                                std::string default_value = "") {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asString();
  }

  std::vector<std::string> GetSTDStringArray(std::string key) {
    std::vector<std::string> result;
    auto value_js = config_[key.c_str()];
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

  std::vector<int> GetIntArray(std::string key) {
    auto value_js = config_[key.c_str()];
    std::vector<int> result;
    if (value_js.isNull()) {
      return result;
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

  std::shared_ptr<JsonConfigWrapper> GetSubConfig(std::string key) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return nullptr;
    }
    return std::shared_ptr<JsonConfigWrapper>(new JsonConfigWrapper(value_js));
  }

  std::shared_ptr<JsonConfigWrapper> GetSubConfig(int key) {
    auto value_js = config_[key];
    if (value_js.isNull()) {
      return nullptr;
    }
    return std::shared_ptr<JsonConfigWrapper>(new JsonConfigWrapper(value_js));
  }

  std::vector<std::shared_ptr<JsonConfigWrapper>> GetSubConfigList(
      std::string key) {
    std::vector<std::shared_ptr<JsonConfigWrapper>> result_list;
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return result_list;
    }

    for (unsigned int i = 0; i < value_js.size(); i++) {
      auto result = std::shared_ptr<JsonConfigWrapper>(
          new JsonConfigWrapper(value_js[i]));
      result_list.push_back(result);
    }
    return result_list;
  }

  bool HasMember(std::string key) { return config_.isMember(key); }
  int ItemCount(void) { return config_.size(); }
  Json::Value GetJsonCfg(void) { return config_; }

 protected:
  Json::Value config_;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_COMMON_JSON_CFG_WRAPPER_H_
