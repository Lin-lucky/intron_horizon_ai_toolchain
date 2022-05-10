/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     uvc_server.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020.5.12
 * \Brief    implement of api file
 */
#include "uvc_server.h"

#include <dirent.h>
#include <pthread.h>
#include <stdio.h>

#include <fstream>
#include <string>

#include "./ring_queue.h"
#include "hobotlog/hobotlog.hpp"
#include "attribute_convert.h"

namespace xproto {

uint64_t UvcServer::height_ = 1080;
uint64_t UvcServer::width_ = 1920;
struct uvc_context *UvcServer::uvc_ctx_ = nullptr;
int UvcServer::uvc_stream_on_ = 0;
std::mutex UvcServer::mutex_;
bool UvcServer::encoder_running_ = false;
std::shared_ptr<UvcConfig> UvcServer::config_ = nullptr;
bool UvcServer::nv12_is_on_ = false;

using xproto::AttributeConvert;
using horizon::vision::MediaCodecManager;
using std::chrono::milliseconds;


#define RES_1080P_WIDTH 1920
#define RES_1080P_HEIGHT 1080
#define RES_2160P_WIDTH 3840
#define RES_2160P_HEIGHT 2160
#define RES_720P_WIDTH 1280
#define RES_720P_HEIGHT 720
#define DEFAULT_1080P_LAYER 4

UvcServer::UvcServer() {
  v4l2_devname_ = nullptr;
  uvc_devname_ = nullptr;
  uvc_stream_on_ = 0;
  encoder_running_ = false;
}

UvcServer::~UvcServer() { DeInit(); }

int UvcServer::Init(std::string config_file) {
  LOGI << "config_file = " << config_file;
  config_path_ = config_file;
  config_ = std::make_shared<UvcConfig>(config_file);
  if (!config_ || !config_->LoadConfig()) {
    LOGE << "failed to load config file";
    return -1;
  }

  width_ = config_->image_width_;
  height_ = config_->image_height_;
  int type = config_->video_type_;
  LOGD << "UvcServer: attribute_description_path: " << config_->attr_des_file_;
  AttributeConvert::Instance().Init(config_->attr_des_file_);
  if (type == UvcConfig::VIDEO_H264) {
    vtype_ = VENC_H264;
  } else if (type == UvcConfig::VIDEO_MJPEG) {
    vtype_ = VENC_MJPEG;
  }

  uvc_init_with_params(vtype_, 0, width_, height_);

  LOGI << "UvcServer::Init";
  return 0;
}

int UvcServer::DeInit() {
  if (uvc_ctx_) {
    //  Stop();
    uvc_gadget_stop(uvc_ctx_);
    uvc_gadget_deinit(uvc_ctx_);
    uvc_ctx_ = nullptr;
  }
  return 0;
}

int UvcServer::Start() { return 0; }

int UvcServer::Stop() { return 0; }

int UvcServer::InitCodecManager(vencParam *vencParam) {
  if (!vencParam) {
    return -1;
  }
  int width = vencParam->width;
  int height = vencParam->height;

  PAYLOAD_TYPE_E format = PT_MJPEG;
  if (vencParam->type == 1) {
    format = PT_MJPEG;
    config_->video_type_ = UvcConfig::VIDEO_MJPEG;
  } else if (vencParam->type == 2) {
    format = PT_H265;
    config_->video_type_ = UvcConfig::VIDEO_H265;
  } else if (vencParam->type == 3) {
    format = PT_H264;
    config_->video_type_ = UvcConfig::VIDEO_H264;
  }

  MediaCodecManager &manager = MediaCodecManager::Get();
  auto rv = manager.ModuleInit();  // ModuleInit()内部保证可以重复初始化
  HOBOT_CHECK(rv == 0);

  int chn_ = 0;
  int pic_width = width;
  int pic_height = height;
  LOGI << "pic_width = " << pic_width << " pic_height = " << pic_height;
  int frame_buf_depth = 3;
  int is_cbr = config_->is_cbr_;
  int bitrate = config_->bitrate_;

  rv = manager.EncodeChnInit(chn_, format, pic_width, pic_height,
                             frame_buf_depth, HB_PIXEL_FORMAT_NV12, is_cbr,
                             bitrate);
  HOBOT_CHECK(rv == 0);

  if (format == PT_MJPEG) {
    rv = manager.SetUserQfactorParams(chn_, config_->jpeg_quality_);
    HOBOT_CHECK(rv == 0);
  }

  rv = manager.EncodeChnStart(chn_);
  HOBOT_CHECK(rv == 0);
  return 0;
}

int UvcServer::DeinitCodecManager(int chn) {
  LOGI << "DeinitCodecManager";
  MediaCodecManager &manager = MediaCodecManager::Get();
  manager.EncodeChnStop(chn);
  manager.EncodeChnDeInit(chn);
  // manager.ModuleDeInit();
  return 0;
}

static int fcc_to_video_format(unsigned int fcc) {
  int format = 1;
  switch (fcc) {
    case V4L2_PIX_FMT_NV12:
      format = 0;
      break;
    case V4L2_PIX_FMT_MJPEG:
      format = 1;
      break;
    case V4L2_PIX_FMT_H265:
      format = 2;
      break;
    case V4L2_PIX_FMT_H264:
      format = 3;
      break;
    default:
      break;
  }

  return format;
}

void UvcServer::uvc_streamon_off(struct uvc_context *ctx, int is_on,
                                 void *userdata) {
  struct uvc_device *dev;
  unsigned int width, height, fcc = 0;

  if (!ctx || !ctx->udev) return;

  dev = ctx->udev;
  width = dev->width;
  height = dev->height;
  fcc = dev->fcc;
  vencParam param;
  param.width = width;
  param.height = height;
  param.type = fcc_to_video_format(fcc);
  param.veChn = 0;
  param.bitrate = 5000;

  if (is_on) {
    if (height == RES_1080P_HEIGHT && -1 != config_->res_1080p_layer_) {
      config_->layer_ = config_->res_1080p_layer_;
    } else if (height == RES_2160P_HEIGHT && -1 != config_->res_2160p_layer_) {
      config_->layer_ = config_->res_2160p_layer_;
    } else if (height == RES_720P_HEIGHT && -1 != config_->res_720p_layer_) {
      config_->layer_ = config_->res_720p_layer_;
    } else {
      config_->layer_ = DEFAULT_1080P_LAYER;
      param.width = RES_1080P_WIDTH;
      param.height = RES_1080P_HEIGHT;
    }
    SetUvcStreamOn(0);
    usleep(10000);

    while (true) {
      if (!IsEncoderRunning()) {
        break;
      }
      usleep(5000);
    }
    if (param.type == 0) {
      SetNv12IsOn(true);
      SetEncoderRunning(false);
      SetUvcStreamOn(1);
    } else {
      SetNv12IsOn(false);
      InitCodecManager(&param);
      usleep(10000);
      SetUvcStreamOn(1);
    }
  } else {
    SetUvcStreamOn(0);
    usleep(10000);
    while (true) {
      if (!IsEncoderRunning()) {
        break;
      }
      usleep(5000);
    }
    DeinitCodecManager(0);
    RingQueue<VIDEO_STREAM_S>::Instance().Clear();
  }
}

int UvcServer::uvc_get_frame(struct uvc_context *ctx, void **buf_to,
                             int *buf_len, void **entity, void *userdata) {
  struct media_info *info = (struct media_info *)userdata;

  if (!info || !ctx || !ctx->udev) {
    LOGE << "uvc_get_frame: input params is null";
    return -EINVAL;
  }
  if (IsUvcStreamOn() == 0) {
    return -EFAULT;
  }

  VIDEO_STREAM_S *one_video =
      reinterpret_cast<VIDEO_STREAM_S *>(calloc(1, sizeof(VIDEO_STREAM_S)));
  if (!one_video) {
    return -EINVAL;
  }
  auto bResult = RingQueue<VIDEO_STREAM_S>::Instance().Pop(*one_video);
  if (!bResult) {
    free(one_video);
    return -EINVAL;
  }
  *buf_to = one_video->pstPack.vir_ptr;
  *buf_len = one_video->pstPack.size;
  *entity = one_video;
#ifdef UVC_DEBUG
  interval_cost_trace_out();
#endif

  return 0;
}

void UvcServer::uvc_release_frame(struct uvc_context *ctx, void **entity,
                                  void *userdata) {
  if (IsUvcStreamOn() == 0) {
    return;
  }
  if (!ctx || !entity || !(*entity)) return;
  auto video_buffer = static_cast<VIDEO_STREAM_S *>(*entity);
  if (video_buffer->pstPack.vir_ptr) free(video_buffer->pstPack.vir_ptr);
  free(video_buffer);
  *entity = nullptr;
  return;
}

int UvcServer::uvc_init_with_params(venc_type type, int vechn, int width,
                                    int height) {
  static struct media_info info;
  struct uvc_params params;
  char *uvc_devname = NULL;  /* uvc Null, lib will find one */
  char *v4l2_devname = NULL; /* v4l2 Null is Null... */
  int ret;

  LOGD << "uvc gadget int with params in";

  /* init media info */
  info.vtype = type;
  info.vch = vechn;
  info.width = width;
  info.height = height;
  switch (type) {
    case VENC_MJPEG:
      info.format = UVC_FORMAT_MJPEG;
      break;
    case VENC_H264:
      info.format = UVC_FORMAT_H264;
      break;
    case VENC_H265:
      info.format = UVC_FORMAT_H265;
      break;
    default:
      info.format = UVC_FORMAT_MJPEG;
      break;
  }

  /* init uvc user params, just use default params and overlay
   * some specify params.
   */
  uvc_gadget_user_params_init(&params);
  params.width = width;
  params.height = height;
  params.format = info.format;
  params.bulk_mode = 1; /* isoc mode still hung, use bulk mode instead */
  params.h264_quirk = 0;

  ret = uvc_gadget_init(&uvc_ctx_, uvc_devname, v4l2_devname, &params);
  if (ret < 0) {
    LOGF << "uvc_gadget_init error!";
    return ret;
  }

  uvc_set_streamon_handler(uvc_ctx_, uvc_streamon_off, &info);
  /* prepare/release buffer with video queue */
  uvc_set_prepare_data_handler(uvc_ctx_, uvc_get_frame, &info);
  uvc_set_release_data_handler(uvc_ctx_, uvc_release_frame, &info);

  ret = uvc_gadget_start(uvc_ctx_);
  if (ret < 0) {
    LOGF << "uvc_gadget_start error!";
  }

  LOGD << "uvc gadget int with params out ret(" << ret << ")";

  return ret;
}
}  // namespace xproto
