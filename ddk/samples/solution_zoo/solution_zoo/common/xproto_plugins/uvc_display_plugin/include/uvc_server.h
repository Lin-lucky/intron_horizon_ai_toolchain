/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     venc_client.h
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020/5/12
 * \Brief    implement of api header
 */

#ifndef INCLUDE_UVCPLUGIN_UVC_SERVER_H_
#define INCLUDE_UVCPLUGIN_UVC_SERVER_H_
#include <string>
#include <thread>
#include <mutex>
#include <memory>
#include "hb_comm_venc.h"
#include "hb_vdec.h"
#include "hb_venc.h"
#include "hb_vio_interface.h"
#include "hb_vps_api.h"
#include "guvc/uvc_gadget.h"
#include "guvc/uvc_gadget_api.h"
// undef problematic defines sometimes defined by system headers (uvc_gadget.h)
#undef max
#include "uvcplugin_config.h"
#include "usb_common.h"
#include "media_codec/media_codec_manager.h"
#include "uevent_helper.h"

namespace xproto {
typedef enum {
  VENC_INVALID = 0,
  VENC_H264,
  VENC_H265,
  VENC_MJPEG,
} venc_type;

struct media_info {
  enum uvc_fourcc_format format;
  venc_type vtype;
  int vch;
  uint32_t width;
  uint32_t height;

  /* store one encoded video stream */
  VIDEO_STREAM_S vstream;
};

struct single_buffer {
  VIDEO_STREAM_S vstream;

  int used;
  pthread_mutex_t mutex;
};

// typedef function<void()> func;

class UvcServer {
  friend class VencClient;

 public:
  UvcServer();
  ~UvcServer();
  int Init(std::string config_file);
  int Start();
  int Stop();
  int DeInit();
  /* function declare */
 public:
  static void uvc_streamon_off(struct uvc_context *ctx, int is_on,
                               void *userdata);

  static int uvc_get_frame(struct uvc_context *ctx, void **buf_to, int *buf_len,
                           void **entity, void *userdata);

  static void uvc_release_frame(struct uvc_context *ctx, void **entity,
                                void *userdata);

  int uvc_init_with_params(venc_type type, int vechn, int width, int height);

  static int InitCodecManager(vencParam *param);

  static int DeinitCodecManager(int chn);

  static bool IsUvcStreamOn() {
    std::lock_guard<std::mutex> lg(mutex_);
    return uvc_stream_on_ == 1;
  }

  static void SetUvcStreamOn(int on) {
    std::lock_guard<std::mutex> lg(mutex_);
    uvc_stream_on_ = on;
  }

  static bool IsEncoderRunning() {
    std::lock_guard<std::mutex> lg(mutex_);
    return encoder_running_;
  }

  static void SetEncoderRunning(bool running) {
    std::lock_guard<std::mutex> lg(mutex_);
    encoder_running_ = running;
  }

  static void SetNv12IsOn(bool is_on) {
    std::lock_guard<std::mutex> lg(mutex_);
    nv12_is_on_ = is_on;
  }

  static bool IsNv12On() {
    std::lock_guard<std::mutex> lg(mutex_);
    return nv12_is_on_;
  }

 public:
  static int uvc_stream_on_;
  static bool encoder_running_;
  static std::mutex mutex_;
  static std::shared_ptr<UvcConfig> config_;
  static bool nv12_is_on_;

 private:
  struct uvc_params params_;
  static struct uvc_context *uvc_ctx_;
  char *uvc_devname_; /* uvc Null, lib will find one */
  char *v4l2_devname_;
  static uint64_t width_;
  static uint64_t height_;
  std::string config_path_;
  venc_type vtype_;
  int efd_;
  int monitor_flag_;
  int kbhit_flag_;
  std::shared_ptr<std::thread> worker_;
};
}  // namespace xproto
#endif /* _UVC_GADGET_API_H_ */
