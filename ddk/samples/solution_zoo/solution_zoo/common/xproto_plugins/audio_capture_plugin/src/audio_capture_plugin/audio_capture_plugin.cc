/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     audio_capture_plugin.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2021.1.7
 * \Brief    implement of api file
 */
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include "audio_capture_plugin/audio_capture_plugin.h"
#include "xproto/message/msg_registry.h"

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_AUDIO_MESSAGE);

namespace xproto {
AudioCapturePlugin::AudioCapturePlugin(std::string config_path) {
  config_path_ = config_path;
  audio_num_ = 0;
}

AudioCapturePlugin::~AudioCapturePlugin() {
}

int AudioCapturePlugin::ParseConfig(std::string config_file) {
  std::ifstream ifs(config_file);
  if (!ifs.is_open()) {
    LOGF << "open config file " << config_file << " failed";
    return -1;
  }
  Json::CharReaderBuilder builder;
  std::string err_json;
  Json::Value json_obj;
  try {
    bool ret = Json::parseFromStream(builder, ifs, &json_obj, &err_json);
    if (!ret) {
      LOGF << "invalid config file " << config_file;
      return -1;
    }
  } catch (std::exception &e) {
    LOGF << "exception while parse config file " << config_file << ", "
         << e.what();
    return -1;
  }

  if (json_obj.isMember("uac_chn")) {
    micphone_chn_ = json_obj["uac_chn"].asInt();
  }

  if (json_obj.isMember("uac_rate")) {
    micphone_rate_ = json_obj["uac_rate"].asInt();
  }

  if (json_obj.isMember("uac_buffer_time")) {
    micphone_buffer_time_ = json_obj["uac_buffer_time"].asInt();
  }

  if (json_obj.isMember("uac_nperiods")) {
    micphone_nperiods_ = json_obj["uac_nperiods"].asInt();
  }

  if (json_obj.isMember("uac_period_size")) {
    micphone_period_size_ = json_obj["micphone_period_size"].asInt();
  }
  return 0;
}

int AudioCapturePlugin::Init() {
  LOGI << "AudioCapturePlugin Init()";
  int ret = -1;

  ret = ParseConfig(config_path_);
  if (ret != 0) {
    LOGE << "ParseConfig failed";
    return -1;
  }
  micphone_device_ = alsa_device_allocate();
  if (!micphone_device_) {
    return -1;
  }

  /* init micphone device*/
  micphone_device_->name = const_cast<char *>("micphone");
  micphone_device_->format = SND_PCM_FORMAT_S16;
  micphone_device_->direct = SND_PCM_STREAM_CAPTURE;
  micphone_device_->rate = micphone_rate_;
  micphone_device_->channels = micphone_chn_;
  micphone_device_->buffer_time = micphone_buffer_time_;
  micphone_device_->nperiods = micphone_nperiods_;
  micphone_device_->period_size = micphone_period_size_;

  ret = alsa_device_init(micphone_device_);
  if (ret < 0) {
    LOGE << "AudioCapturePlugin Init() failed";
    return -1;
  }

  return 0;
}

int AudioCapturePlugin::DeInit() {
  LOGD << "AudioCapturePlugin::DeInit()";
  if (!micphone_device_)
    return -1;

  if (micphone_device_) {
    alsa_device_deinit(micphone_device_);
    alsa_device_free(micphone_device_);
    micphone_device_ = nullptr;
  }
  return 0;
}

int AudioCapturePlugin::Start() {
  exit_ = false;
  if (micphone_device_) {
    micphone_thread_ =
      std::make_shared<std::thread>(&AudioCapturePlugin::MicphoneGetThread,
          this);
  }
  return 0;
}

int AudioCapturePlugin::Stop() {
  LOGD << "AudioCapturePlugin::Stop";
  if (micphone_thread_) {
    exit_ = true;
    micphone_thread_->join();
    micphone_thread_ = nullptr;
  }

  return 0;
}

int AudioCapturePlugin::MicphoneGetThread() {
  int ret = -1;
  snd_pcm_sframes_t frames;
  char *buffer;
  int size;
  if (!micphone_device_) {
    return -1;
  }

  frames = micphone_device_->period_size;
  size = snd_pcm_frames_to_bytes(micphone_device_->handle, frames);
  buffer = reinterpret_cast<char *>(malloc(size));

  while (!exit_) {
    ret = alsa_device_read(micphone_device_, buffer, frames);
    if (ret > 0) {
     auto input = std::make_shared<AudioMessage>(buffer, frames, audio_num_);
     audio_num_++;
     PushMsg(input);
    }
  }
  if (buffer) {
    free(buffer);
  }
  return 0;
}
}  // namespace xproto
