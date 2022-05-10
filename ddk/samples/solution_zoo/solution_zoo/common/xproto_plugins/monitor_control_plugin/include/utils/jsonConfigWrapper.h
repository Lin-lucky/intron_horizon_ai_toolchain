/*
 * @Description:  Json common interface
 * @Author:  songshan.gong@horizon.ai
 * @Date: 2019-10-25 15:12:54
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-11-30 15:24:40
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef INCLUDE_UTILS_JASON_CONFIG_WRAPPER_H_
#define INCLUDE_UTILS_JASON_CONFIG_WRAPPER_H_
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "json/json.h"
#include "monitor_control_plugin/common_data.h"

namespace xproto {

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
    auto value_js = config_[key.c_str()];
    std::vector<std::string> ret;
    if (value_js.isNull()) {
      return ret;
    }
    ret.resize(value_js.size());
    for (Json::ArrayIndex i = 0; i < value_js.size(); ++i) {
      ret[i] = value_js[i].asString();
    }
    return ret;
  }

  int32_t GetSTDIntValue(std::string key, int32_t default_value = 0) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return default_value;
    }
    return value_js.asInt();
  }

  std::vector<int32_t> GetSTDIntArray(std::string key) {
    auto value_js = config_[key.c_str()];
    std::vector<int32_t> ret;
    if (value_js.isNull()) {
      return ret;
    }
    ret.resize(value_js.size());
    for (Json::ArrayIndex i = 0; i < value_js.size(); ++i) {
      ret[i] = value_js[i].asInt();
    }
    return ret;
  }

  std::vector<box_t> GetBoxArray(std::string key) {
    auto value_js = config_[key.c_str()];
    std::vector<box_t> ret;
    if (value_js.isNull()) {
      return ret;
    }
    ret.resize(value_js.size());
    for (Json::ArrayIndex i = 0; i < value_js.size(); ++i) {
      ret[i].left = value_js[i][0].asInt();
      ret[i].top = value_js[i][1].asInt();
      ret[i].right = value_js[i][2].asInt();
      ret[i].bottom = value_js[i][3].asInt();
    }
    return ret;
  }

  std::shared_ptr<JsonConfigWrapper> GetSubConfig(std::string key) {
    auto value_js = config_[key.c_str()];
    if (value_js.isNull()) {
      return nullptr;
    }
    return std::shared_ptr<JsonConfigWrapper>(new JsonConfigWrapper(value_js));
  }

  std::string GetJson() { return config_.toStyledString(); }

  bool HasKey(const std::string& key) {
    auto value_js = config_[key.c_str()];
    return value_js.isNull() ? false : true;
  }

  template <typename T>
  void SetValue(const std::string& key, const T& value) {
    config_[key] = value;
  }

 protected:
  Json::Value config_;
};

class JsonReader {
 public:
  explicit JsonReader(std::string path) : path_(path) {}
  explicit JsonReader(Json::Value root) : root_(root) {}

  int32_t ParseJsonFile(void) {
    std::ifstream ifs(path_);
    if (!ifs.is_open()) {
      LOGE << "Open config file " << path_ << " fail!!!";
      return -1;
    }
    LOGD << "Path: " << path_;
    std::stringstream ss;
    ss << ifs.rdbuf();
    ifs.close();
    std::string content = ss.str();
    LOGD << "Load content: " << content;
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    try {
      bool ret = reader->parse(
          content.c_str(), content.c_str() + content.size(), &root_, &error);
      if (ret) {
        return 0;
      } else {
        return -1;
      }
    } catch (std::exception &e) {
      return -1;
    }
  }

  int GetIntValue(std::string key, int default_value = 0) {
    LOGD << "Key value: " << key;
    auto value_js = root_[key];
    if (value_js.isNull()) {
      LOGE << "Get default value";
      return default_value;
    }
    return value_js.asInt();
  }

  bool GetBoolValue(std::string key, bool default_value = false) {
    auto value_js = root_[key];
    if (value_js.isNull()) {
      LOGE << "Get default value";
      return default_value;
    }
    return value_js.asBool();
  }

  float GetFloatValue(std::string key, float default_value = 0.0) {
    auto value_js = root_[key];
    if (value_js.isNull()) {
      LOGE << "Get default value";
      return default_value;
    }
    return value_js.asFloat();
  }

  std::string GetStringValue(std::string key, std::string default_value = "") {
    auto value_js = root_[key];
    if (value_js.isNull()) {
      LOGE << "Get default value";
      return default_value;
    }
    return value_js.asString();
  }

  std::vector<int32_t> GetIntArray(std::string key) {
    auto value_js = root_[key];
    std::vector<int32_t> ret;
    if (value_js.isNull()) {
      LOGE << "Get default value";
      return ret;
    }
    ret.resize(value_js.size());
    for (Json::ArrayIndex i = 0; i < value_js.size(); ++i) {
      ret[i] = value_js[i].asInt();
    }
    return ret;
  }

  std::vector<std::string> GetStringArray(std::string key) {
    auto value_js = root_[key];
    std::vector<std::string> ret;
    if (value_js.isNull()) {
      return ret;
    }
    ret.resize(value_js.size());
    for (Json::ArrayIndex i = 0; i < value_js.size(); ++i) {
      ret[i] = value_js[i].asString();
    }
    return ret;
  }

  std::shared_ptr<JsonReader> GetSubConfig(std::string key) {
    auto value_js = root_[key];
    if (value_js.isNull()) {
      return nullptr;
    }
    return std::shared_ptr<JsonReader>(new JsonReader(value_js));
  }

  template <typename T>
  void SetValue(const std::string &key, const T &value) {
    root_[key] = value;
  }

  std::string GetJson() { return root_.toStyledString(); }

  Json::Value GetRawJson() {return root_;}

 private:
  std::string path_;
  Json::Value root_;
};

class JsonWriter {
 public:
  explicit JsonWriter(std::string path) : path_(path) {}

  int32_t OpenJsonFile(void) {
    int32_t ret = 0;
    std::ifstream ifs(path_);
    LOGD << "Path: " << path_;
    if (!ifs.is_open()) {
      LOGE << "The file is not exist";
    } else {
      std::stringstream ss;
      ss << ifs.rdbuf();
      ifs.close();
      std::string content = ss.str();
      LOGD << "Load content: " << content;
      Json::CharReaderBuilder ReadBuilder;
      ReadBuilder["collectComments"] = false;
      JSONCPP_STRING error;
      std::unique_ptr<Json::CharReader> JsonReader(ReadBuilder.newCharReader());
      try {
        ret = JsonReader->parse(
            content.c_str(), content.c_str() + content.size(), &root_, &error);
        if (ret) {
          LOGI << "Parse success";
          ret = 0;
        } else {
          LOGE << "Parse error";
          ret = -1;
        }
      } catch (std::exception &e) {
        LOGE << "Exception error: " << e.what();
        ret = -1;
      }
    }
    Json::StreamWriterBuilder WriteBuilder;
    WriteBuilder["commentStyle"] = "None";
    WriteBuilder["indentation"] = "  ";
    JsonWriter_.reset(WriteBuilder.newStreamWriter());
    return ret;
  }

  int32_t CloseJsonFile(void) {
    std::ofstream ofs(path_);
    LOGD << "Path: " << path_;
    if (!ofs.is_open()) {
      LOGE << "The file is not exist";
    }
    LOGD << "Try write";
    try {
      JsonWriter_->write(root_, &ofs);
    } catch (std::exception &e) {
      LOGE << "Exception error: " << e.what();
      return -1;
    }
    ofs.close();
    return 0;
  }

  std::string GetJson() { return root_.toStyledString(); }

  template <typename T>
  void SetValue(const std::string &key, const T &value) {
    root_[key] = value;
  }

 private:
  std::string path_;
  Json::Value root_;
  std::unique_ptr<Json::StreamWriter> JsonWriter_;
};

}  // namespace xproto
#endif  // INCLUDE_UTILS_JASON_CONFIG_WRAPPER_H_
