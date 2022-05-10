/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     UvcServerPlugin.h
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020/5/12
 * \Brief    implement of api header
 */
#ifndef INCLUDE_UVC_SERVER_PLUGIN_UVC_SERVERPLUGIN_H_
#define INCLUDE_UVC_SERVER_PLUGIN_UVC_SERVERPLUGIN_H_
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "./usb_common.h"
#include "hb_comm_venc.h"
#include "hb_vdec.h"
#include "hb_venc.h"
#include "hb_vio_interface.h"
#include "hb_vps_api.h"
#include "thread_pool.h"
#include "uvc_server/uvc_server.h"
#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"

namespace xproto {
using uvccomponent::UvcEvent;
using uvccomponent::UvcEventData;
using uvccomponent::UvcServer;
using xproto::XPluginAsync;
using xproto::XProtoMessagePtr;
class UvcConfig;
class UvcServerPlugin : public xproto::XPluginAsync,
                        uvccomponent::UvcEventCallback {
 public:
  UvcServerPlugin() = delete;
  explicit UvcServerPlugin(std::string config_path);
  ~UvcServerPlugin() override;
  int Init() override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "UvcServerPlugin"; }
  void OnUvcEvent(UvcEvent event_type, void *data, int data_len) override;

 public:
  inline bool IsUvcStreamOn() {
    std::lock_guard<std::mutex> lg(mutex_);
    return uvc_stream_on_ > 0;
  }

  inline void SetUvcStreamOn(int on) {
    std::lock_guard<std::mutex> lg(mutex_);
    uvc_stream_on_ = on;
  }

  inline bool IsEncoderRunning() {
    std::lock_guard<std::mutex> lg(mutex_);
    return encoder_running_;
  }

  inline void SetEncoderRunning(bool running) {
    std::lock_guard<std::mutex> lg(mutex_);
    encoder_running_ = running;
  }
  inline void SetNv12IsOn(bool is_on) {
    std::lock_guard<std::mutex> lg(mutex_);
    nv12_is_on_ = is_on;
  }

  inline bool IsNv12On() {
    std::lock_guard<std::mutex> lg(mutex_);
    return nv12_is_on_;
  }

 private:
  int FeedVideoMsg(XProtoMessagePtr msg);
  int FeedVideoDropMsg(XProtoMessagePtr msg);
  int FeedVideo(XProtoMessagePtr msg);
  int FeedVideoDrop(XProtoMessagePtr msg);
  int Reset();
  int ParseConfig(std::string config_file);
  int InitCodecManager(vencParam *param);
  int DeinitCodecManager(int chn);

  int ReInit();
  int ReStart();

 private:
  std::string config_file_;
  bool run_flag_;
  std::shared_ptr<std::thread> worker_;
  std::mutex map_mutex_;
  const uint8_t cache_size_ = 25;  // max input cache size
  std::shared_ptr<UvcServer> uvc_server_;
  std::shared_ptr<UvcConfig> config_;

  int origin_image_width_ = 1920;  // update by FeedVideo
  int origin_image_height_ = 1080;
  int dst_image_width_ = 1920;  // update by FeedVideo
  int dst_image_height_ = 1080;

  std::mutex video_send_mutex_;
  int video_sended_without_recv_count_;
  horizon::vision::CThreadPool encode_thread_;
  bool print_timestamp_ = false;
  int uvc_stream_on_ = 0;
  bool nv12_is_on_ = false;
  bool encoder_running_ = false;
  std::mutex mutex_;
  int chn_;
};
}  // namespace xproto
#endif  // INCLUDE_UVC_SERVER_PLUGIN_UVC_SERVERPLUGIN_H_
