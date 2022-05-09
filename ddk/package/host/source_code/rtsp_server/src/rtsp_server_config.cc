#include "rtsp_server_config.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

namespace rtspcomponent {
RtspServerConfig::RtspServerConfig(const std::string &path) {
  LOGI << "RtspServerConfig";
  path_ = path;
  auth_mode_ = 0;
  user_ = "admin";
  password_ = "123456";
  port_ = 555;
}

RtspServerConfig::~RtspServerConfig() {
  if (json_) {
    delete json_;
    json_ = nullptr;
  }
}

bool RtspServerConfig::LoadConfig() {
  if (json_ == nullptr) {
    json_ = new Json::Value();
  }
  std::ifstream ifs(path_);
  if (!ifs.is_open()) {
    LOGF << "open config file " << path_ << " failed";
    return false;
  }

  Json::CharReaderBuilder builder;
  std::string err_json;
  try {
    bool ret = Json::parseFromStream(builder, ifs, json_, &err_json);
    if (!ret) {
      LOGE << "invalid config file " << path_;
      return false;
    }
  } catch (std::exception &e) {
    LOGE << "exception while parse config file " << path_ << ", " << e.what();
    return false;
  }
  return CheckConfig();
}

bool RtspServerConfig::CheckConfig() {
  if (json_->isMember("auth_mode")) {
      auth_mode_ = (*json_)["auth_mode"].asInt();
  }

  if (auth_mode_ != 0) {
      if (json_->isMember("user")) {
          user_ = (*json_)["user"].asString();
      }

      if (json_->isMember("password")) {
          password_ = (*json_)["password"].asString();
      }
  }

  if (json_->isMember("port")) {
    port_ = (*json_)["port"].asInt();
  }

  if (json_->isMember("chn_num")) {
    chn_num_ = (*json_)["chn_num"].asInt();
  }

  for (int i = 0; i < chn_num_; i++) {
    std::string chn("chn_" + std::to_string(i));
    std::string rtsp_url = (*json_)[chn.c_str()]["url"].asString();
    int pos = rtsp_url.find_last_of("/");

    std::string url_post;
    if (pos != -1) {
      url_post = rtsp_url.substr(pos+1, rtsp_url.size() - pos - 1);
    }

    stream_name_.push_back(url_post);

    int type = (*json_)[chn.c_str()]["video_type"].asInt();
    Video_Type video_type;
    if (type == 0) {
      video_type = H264;
    } else if (type == 1) {
      video_type = H265;
    } else {
      video_type = H264;
    }
    video_type_.push_back(video_type);

    Audio_Type audio_type;
    type = (*json_)[chn.c_str()]["audio_type"].asInt();
    if (type == 2) {
      audio_type = G711;
    } else if (type == 3) {
      audio_type = G726;
    } else {
      audio_type = G711;
    }
    audio_type_.push_back(audio_type);
  }

  return true;
}

std::string RtspServerConfig::GetStringValue(const std::string &key) {
  std::string value;
  if ((*json_)[key].empty()) {
    LOGW << "can not find key: " << key;
    return "";
  }

  return (*json_)[key].asString();
}

std::string RtspServerConfig::GetChnStringValue(
  int chn_num, const std::string &key) {
  std::string value;
  std::string chn = "chn_" + std::to_string(chn_num);
  if ((*json_)[chn][key].empty()) {
    LOGW << "can not find chn "<< chn <<" key: " << key;
    return "";
  }

  return (*json_)[chn][key].asString();
}

int RtspServerConfig::GetIntValue(const std::string &key) {
  std::string value;
  if ((*json_)[key].empty()) {
    LOGW << "can not find key: " << key;
    return -1;
  }

  return (*json_)[key].asInt();
}

int RtspServerConfig::GetChnIntValue(int chn_num, const std::string &key) {
  std::string value;
  std::string chn = "chn_" + std::to_string(chn_num);
  if ((*json_)[chn][key].empty()) {
    LOGW << "can not find chn "<< chn <<" key: " << key;
    return -1;
  }

  return (*json_)[chn][key].asInt();
}

Video_Type RtspServerConfig::GetChnVideoType(int chn) {
  return video_type_[chn];
}

Audio_Type RtspServerConfig::GetChnAudioType(int chn) {
  return audio_type_[chn];
}

std::string RtspServerConfig::GetChnStreamName(int chn) {
  return stream_name_[chn];
}
}  // namespace rtspcomponent
