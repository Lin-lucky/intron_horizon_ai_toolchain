/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     audio_output_plugin.h
 * \Author   xudong.du
 * \Version  1.0.0.0
 * \Date     2021/06/15
 * \Brief    implement of api header
 */

#ifndef INCLUDE_AUDIO_OUTPUT_PLUGIN_H_
#define INCLUDE_AUDIO_OUTPUT_PLUGIN_H_
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include "alsa_device.h"
#include "audio_message/audio_message.h"
#include "json/json.h"
#include "thread_pool.h"
#include "xproto/plugin/xpluginasync.h"

namespace xproto {
using xproto::message::AudioMessage;
class AudioOutputPlugin : public xproto::XPluginAsync {
 public:
  AudioOutputPlugin() = delete;
  explicit AudioOutputPlugin(std::string config_path);
  ~AudioOutputPlugin() {}
  int Init() override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "AudioOutputPlugin"; }

 private:
  int FeedAudioMsg(XProtoMessagePtr msg);
  int FeedMicphoneAudio(XProtoMessagePtr msg);
  int SendUacData(char* buffer, int size);
  int ParseConfig(std::string config_path);

 private:
  alsa_device_t* uac_device_;
  std::string config_path_;
  horizon::vision::CThreadPool send_audio_thread_;
  int uac_rate_;
  int uac_chn_;
  int uac_buffer_time_;
  int uac_nperiods_;
  const int max_thread_task_ = 50;
};
}  // namespace xproto
#endif  // INCLUDE_AUDIO_OUTPUT_PLUGIN_H_
