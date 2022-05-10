/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     audio_capture_plugin.h
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2021.1.7
 * \Brief    implement of api file
 */
#ifndef INCLUDE_AUDIO_CAPTURE_PLUGIN_
#define INCLUDE_AUDIO_CAPTURE_PLUGIN_

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <thread>
#include "json/json.h"
#include "hobotlog/hobotlog.hpp"
#include "xproto/plugin/xpluginasync.h"
#include "audio_message/audio_message.h"
#include "alsa_device.h"

namespace xproto {
using xproto::message::AudioMessage;

class AudioCapturePlugin : public xproto::XPluginAsync {
 public:
  AudioCapturePlugin() = delete;
  explicit AudioCapturePlugin(std::string config_path);
  ~AudioCapturePlugin();
  int Init() override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "AudioCapturePlugin"; }

 private:
  int MicphoneGetThread();
  int ParseConfig(std::string config_file);
  std::shared_ptr<std::thread> micphone_thread_;
  alsa_device_t *micphone_device_;
  bool exit_;
  int audio_num_;
  std::string config_path_;
  int micphone_rate_;
  int micphone_chn_;
  int micphone_buffer_time_;
  int micphone_nperiods_;
  int micphone_period_size_;
};
}  // namespace xproto

#endif  // INCLUDE_AUDIO_CAPTURE_PLUGIN_
