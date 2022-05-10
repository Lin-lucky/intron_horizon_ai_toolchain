/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     UvcServerPlugin.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020.5.12
 * \Brief    implement of api file
 */

#include "uvc_server_plugin/uvc_server_plugin.h"

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include "./uvc_server_config.h"
#include "hobotlog/hobotlog.hpp"
#include "media_codec/media_codec_manager.h"
#include "utils/time_helper.h"
#include "uvc_server/uvc_server.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/vio_message.h"

namespace xproto {
using hobot::Timer;
using horizon::vision::MediaCodecManager;
using std::chrono::milliseconds;
using uvccomponent::StreamParams;
using uvccomponent::UvcVideoData;
using xproto::XPluginErrorCode;
using xproto::message::VioMessage;

#define RES_1080P_WIDTH 1920
#define RES_1080P_HEIGHT 1080
#define RES_2160P_WIDTH 3840
#define RES_2160P_HEIGHT 2160
#define RES_720P_WIDTH 1280
#define RES_720P_HEIGHT 720
#define DEFAULT_1080P_LAYER 4

UvcServerPlugin::UvcServerPlugin(std::string config_file) {
  config_file_ = config_file;
  LOGI << "UvcServerPlugin smart config file:" << config_file_;
  run_flag_ = false;
  uvc_stream_on_ = false;
  chn_ = 0;
  Reset();
}

UvcServerPlugin::~UvcServerPlugin() {
  //  config_ = nullptr;
}

int UvcServerPlugin::ParseConfig(std::string config_file) {
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
  return 0;
}

int UvcServerPlugin::Init() {
  LOGI << "UvcServerPlugin Init";
  if (0 != ParseConfig(config_file_)) {
    return -1;
  }

  uvc_server_ = UvcServer::GetInstance();
  HOBOT_CHECK(uvc_server_);
  if (uvc_server_->Init(this)) {
    LOGE << "UwsPlugin Init uWS server failed";
    return -1;
  }

  config_ = std::make_shared<UvcConfig>(config_file_);
  if (!config_ || !config_->LoadConfig()) {
    LOGE << "failed to load config file";
    return -1;
  }

  RegisterMsg(TYPE_IMAGE_MESSAGE, std::bind(&UvcServerPlugin::FeedVideoMsg,
                                            this, std::placeholders::_1));
  RegisterMsg(TYPE_DROP_IMAGE_MESSAGE,
              std::bind(&UvcServerPlugin::FeedVideoDropMsg, this,
                        std::placeholders::_1));
  std::lock_guard<std::mutex> lk(video_send_mutex_);
  video_sended_without_recv_count_ = 0;
  encode_thread_.CreatThread(1);
  auto print_timestamp_str = getenv("uvc_print_timestamp");
  if (print_timestamp_str && !strcmp(print_timestamp_str, "ON")) {
    print_timestamp_ = true;
  }
  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();
  return 0;
}

int UvcServerPlugin::DeInit() {
  LOGD << "uvc plugin deinit...";
  if (uvc_server_) {
    uvc_server_->DeInit();
  }

  LOGD << "uvc server deinit done";
  return XPluginAsync::DeInit();
}

int UvcServerPlugin::Reset() {
  LOGI << __FUNCTION__ << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
  return 0;
}

int UvcServerPlugin::Start() {
  LOGD << "uvc start";
  if (run_flag_) {
    return 0;
  }
  run_flag_ = true;
  if (uvc_server_) {
    uvc_server_->Start();
  }
  return 0;
}

int UvcServerPlugin::Stop() {
  if (!run_flag_) {
    return 0;
  }
  LOGI << "UvcServerPlugin::Stop()";
  run_flag_ = false;
  if (uvc_server_) {
    uvc_server_->Stop();
  }
  LOGI << "UvcServerPlugin stop done";
  return 0;
}

int UvcServerPlugin::FeedVideoMsg(XProtoMessagePtr msg) {
  encode_thread_.PostTask(std::bind(&UvcServerPlugin::FeedVideo, this, msg));
  return 0;
}

int UvcServerPlugin::FeedVideoDropMsg(XProtoMessagePtr msg) {
  encode_thread_.PostTask(
      std::bind(&UvcServerPlugin::FeedVideoDrop, this, msg));
  return 0;
}

int UvcServerPlugin::FeedVideo(XProtoMessagePtr msg) {
  if (!run_flag_) {
    return 0;
  }
  if (false == IsUvcStreamOn()) {
    return 0;
  }

  SetEncoderRunning(true);
  LOGD << "UvcServerPlugin Feedvideo";
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  auto vio_msg = std::dynamic_pointer_cast<VioMessage>(msg);
  int level = config_->layer_;

  auto pym_image = vio_msg->image_[0];
  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
  pstFrame.stVFrame.height = pym_image->img_.down_scale[level].height;
  pstFrame.stVFrame.width = pym_image->img_.down_scale[level].width;
  pstFrame.stVFrame.size = pym_image->img_.down_scale[level].height *
                           pym_image->img_.down_scale[level].width * 3 / 2;
  pstFrame.stVFrame.phy_ptr[0] =
      (uint32_t)pym_image->img_.down_scale[level].y_paddr;
  pstFrame.stVFrame.phy_ptr[1] =
      (uint32_t)pym_image->img_.down_scale[level].c_paddr;
  pstFrame.stVFrame.vir_ptr[0] =
      reinterpret_cast<char *>(pym_image->img_.down_scale[level].y_vaddr);
  pstFrame.stVFrame.vir_ptr[1] =
      reinterpret_cast<char *>(pym_image->img_.down_scale[level].c_vaddr);
  pstFrame.stVFrame.pts = vio_msg->time_stamp_;

  origin_image_width_ = pym_image->img_.down_scale[0].width;
  origin_image_height_ = pym_image->img_.down_scale[0].height;
  dst_image_width_ = pym_image->img_.down_scale[level].width;
  dst_image_height_ = pym_image->img_.down_scale[level].height;

  //  if (!RingQueue<VIDEO_STREAM_S>::Instance().IsValid()) {
  // SetEncoderRunning(false);
  //  return 0;
  // }
  if (print_timestamp_) {
    LOGW << "FeedVideo time_stamp:" << vio_msg->time_stamp_;
  }
  if (IsNv12On()) {
    UvcVideoData vstream;
    memset(&vstream, 0, sizeof(UvcVideoData));
    auto buffer_size = pstFrame.stVFrame.size;
    vstream.v_buffer_ = reinterpret_cast<char *>(calloc(1, buffer_size));
    vstream.v_size_ = buffer_size;
    memcpy(vstream.v_buffer_, pstFrame.stVFrame.vir_ptr[0],
           pstFrame.stVFrame.height * pstFrame.stVFrame.width);
    memcpy(
        vstream.v_buffer_ + pstFrame.stVFrame.height * pstFrame.stVFrame.width,
        pstFrame.stVFrame.vir_ptr[1],
        pstFrame.stVFrame.height * pstFrame.stVFrame.width / 2);
    uvc_server_->SendFrame(vstream);  // 默认有uvc_server管理v_buffer内存释放
    SetEncoderRunning(false);
    return 0;
  }
  auto ts0 = Timer::current_time_stamp();

  std::string user_data = ((UvcConfig::VIDEO_MJPEG == config_->video_type_ ||
                            UvcConfig::VIDEO_JPG == config_->video_type_)
                               ? ""
                               : "dc45e9bd-e6d948b7-962cd820-d923eeef+") +
                          std::to_string(vio_msg->time_stamp_);
  LOGD << "venc insert user_data:" << user_data;
  if (HB_VENC_InserUserData(
          0, reinterpret_cast<uint8_t *>(const_cast<char *>(user_data.data())),
          user_data.length()) != 0) {
    LOGE << "HB_VENC_InserUserData fail";
  }

  int ret = HB_VENC_SendFrame(0, &pstFrame, 0);
  if (ret < 0) {
    LOGE << "HB_VENC_SendFrame 0 error!!!ret " << ret;
    SetEncoderRunning(false);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    LOGW << "video_sended_without_recv_count: "
         << video_sended_without_recv_count_;
    return 0;
  } else {
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    ++video_sended_without_recv_count_;
  }

  VIDEO_STREAM_S vstream;
  memset(&vstream, 0, sizeof(VIDEO_STREAM_S));
  ret = HB_VENC_GetStream(0, &vstream, 2000);
  if (config_->h264_encode_time_ == 1) {
    auto ts1 = Timer::current_time_stamp();
    LOGI << "******Encode yuv to h264 cost: " << ts1 - ts0 << "ms";
  }
  if (ret < 0) {
    LOGE << "HB_VENC_GetStream timeout: " << ret;
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    LOGW << "video_sended_without_recv_count: "
         << video_sended_without_recv_count_;
  } else {
    UvcVideoData video_buffer;
    memset(&video_buffer, 0, sizeof(UvcVideoData));
    auto buffer_size = vstream.pstPack.size;
    HOBOT_CHECK(buffer_size > 5) << "encode bitstream too small";
    int nal_type = -1;
    if ((0 == static_cast<int>(vstream.pstPack.vir_ptr[0])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[1])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[2])) &&
        (1 == static_cast<int>(vstream.pstPack.vir_ptr[3]))) {
      nal_type = static_cast<int>(vstream.pstPack.vir_ptr[4] & 0x1F);
    }
    LOGD << "nal type is " << nal_type;
    video_buffer.v_buffer_ = reinterpret_cast<char *>(calloc(1, buffer_size));
    if (video_buffer.v_buffer_) {
      memcpy(video_buffer.v_buffer_, vstream.pstPack.vir_ptr, buffer_size);
      video_buffer.v_size_ = buffer_size;
      uvc_server_->SendFrame(
          video_buffer);  // 默认有uvc_server管理v_buffer内存释放
    }
    HB_VENC_ReleaseStream(0, &vstream);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    --video_sended_without_recv_count_;
  }

  SetEncoderRunning(false);
  return 0;
}

int UvcServerPlugin::FeedVideoDrop(XProtoMessagePtr msg) {
  if (!run_flag_) {
    return 0;
  }
  if (!IsUvcStreamOn()) {
    return 0;
  }
  SetEncoderRunning(true);
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  auto vio_msg = std::dynamic_pointer_cast<VioMessage>(msg);
  int level = config_->layer_;

  auto pym_image = vio_msg->image_[0];
  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
  pstFrame.stVFrame.height = pym_image->img_.down_scale[level].height;
  pstFrame.stVFrame.width = pym_image->img_.down_scale[level].width;
  pstFrame.stVFrame.size = pym_image->img_.down_scale[level].height *
                           pym_image->img_.down_scale[level].width * 3 / 2;
  pstFrame.stVFrame.phy_ptr[0] =
      (uint32_t)pym_image->img_.down_scale[level].y_paddr;
  pstFrame.stVFrame.phy_ptr[1] =
      (uint32_t)pym_image->img_.down_scale[level].c_paddr;
  pstFrame.stVFrame.vir_ptr[0] =
      reinterpret_cast<char *>(pym_image->img_.down_scale[level].y_vaddr);
  pstFrame.stVFrame.vir_ptr[1] =
      reinterpret_cast<char *>(pym_image->img_.down_scale[level].c_vaddr);
  pstFrame.stVFrame.pts = vio_msg->time_stamp_;

  origin_image_width_ = pym_image->img_.down_scale[0].width;
  origin_image_height_ = pym_image->img_.down_scale[0].height;
  dst_image_width_ = pym_image->img_.down_scale[level].width;
  dst_image_height_ = pym_image->img_.down_scale[level].height;

  if (print_timestamp_) {
    LOGW << "FeedVideoDrop time_stamp:" << vio_msg->time_stamp_;
  }

  if (IsNv12On()) {
    UvcVideoData vstream;
    memset(&vstream, 0, sizeof(UvcVideoData));
    auto buffer_size = pstFrame.stVFrame.size;
    vstream.v_buffer_ = reinterpret_cast<char *>(calloc(1, buffer_size));
    vstream.v_size_ = buffer_size;
    memcpy(vstream.v_buffer_, pstFrame.stVFrame.vir_ptr[0],
           pstFrame.stVFrame.height * pstFrame.stVFrame.width);
    memcpy(
        vstream.v_buffer_ + pstFrame.stVFrame.height * pstFrame.stVFrame.width,
        pstFrame.stVFrame.vir_ptr[1],
        pstFrame.stVFrame.height * pstFrame.stVFrame.width / 2);
    uvc_server_->SendFrame(vstream);  // 默认有uvc_server管理v_buffer内存释放
    return 0;
  }

  // For H264 and H265 encoder format, userdata format is uuid + user defined
  // data (string) For jpg and mjpeg encoder format, userdata format is only
  // user defined data (string)
  std::string user_data = ((UvcConfig::VIDEO_MJPEG == config_->video_type_ ||
                            UvcConfig::VIDEO_JPG == config_->video_type_)
                               ? ""
                               : "dc45e9bd-e6d948b7-962cd820-d923eeef+") +
                          std::to_string(vio_msg->time_stamp_);
  LOGD << "venc insert user_data:" << user_data;
  if (HB_VENC_InserUserData(
          0, reinterpret_cast<uint8_t *>(const_cast<char *>(user_data.data())),
          user_data.length()) != 0) {
    LOGE << "HB_VENC_InserUserData fail";
  }

  int ret = HB_VENC_SendFrame(0, &pstFrame, 0);
  if (ret < 0) {
    LOGE << "HB_VENC_SendFrame 0 error!!!ret " << ret;
    SetEncoderRunning(false);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    LOGW << "video_sended_without_recv_count: "
         << video_sended_without_recv_count_;
    return 0;
  } else {
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    ++video_sended_without_recv_count_;
  }

  VIDEO_STREAM_S vstream;
  memset(&vstream, 0, sizeof(VIDEO_STREAM_S));
  ret = HB_VENC_GetStream(0, &vstream, 2000);
  if (ret < 0) {
    LOGE << "HB_VENC_GetStream timeout: " << ret;
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    LOGW << "video_sended_without_recv_count: "
         << video_sended_without_recv_count_;
  } else {
    UvcVideoData video_buffer;
    memset(&video_buffer, 0, sizeof(UvcVideoData));
    auto buffer_size = vstream.pstPack.size;
    HOBOT_CHECK(buffer_size > 5) << "encode bitstream too small";
    int nal_type = -1;
    if ((0 == static_cast<int>(vstream.pstPack.vir_ptr[0])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[1])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[2])) &&
        (1 == static_cast<int>(vstream.pstPack.vir_ptr[3]))) {
      nal_type = static_cast<int>(vstream.pstPack.vir_ptr[4] & 0x1F);
    }
    LOGD << "nal type is " << nal_type;
    video_buffer.v_buffer_ = reinterpret_cast<char *>(calloc(1, buffer_size));
    video_buffer.v_size_ = buffer_size;
    if (video_buffer.v_buffer_) {
      memcpy(video_buffer.v_buffer_, vstream.pstPack.vir_ptr, buffer_size);
      uvc_server_->SendFrame(
          video_buffer);  // 默认有uvc_server管理v_buffer内存释放
    }
    HB_VENC_ReleaseStream(0, &vstream);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    --video_sended_without_recv_count_;
  }
  SetEncoderRunning(false);
  return 0;
}

int UvcServerPlugin::ReInit() {
  // uvc_server_ = std::make_shared<UvcServer>();
  if (uvc_server_) {
    uvc_server_->Init(this);
  }
  return 0;
}

int UvcServerPlugin::ReStart() {
  run_flag_ = true;
  if (uvc_server_) {
    uvc_server_->Start();
  }
  return 0;
}

void UvcServerPlugin::OnUvcEvent(UvcEvent event_type, void *data,
                                 int data_len) {
  LOGW << "UvcServerPlugin::OnUvcEvent event type: " << event_type;
  switch (event_type) {
    case uvccomponent::UVC_STREAM_OFF:
      /* code */
      SetUvcStreamOn(0);
      usleep(10000);
      while (true) {
        if (!IsEncoderRunning()) {
          break;
        }
        usleep(5000);
      }
      DeinitCodecManager(chn_);
      break;
    case uvccomponent::UVC_STREAM_ON: {
      StreamParams *event_data = reinterpret_cast<StreamParams *>(data);
      if (!event_data) {
        break;
      }
      vencParam param;
      param.width = event_data->width_;
      param.height = event_data->height_;
      param.type = event_data->video_type_;  // 已经转换成了目标
      param.veChn = 0;
      param.bitrate = 5000;
      int request_height = event_data->height_;
      if (request_height == RES_1080P_HEIGHT &&
          -1 != config_->res_1080p_layer_) {
        config_->layer_ = config_->res_1080p_layer_;
      } else if (request_height == RES_2160P_HEIGHT &&
                 -1 != config_->res_2160p_layer_) {
        config_->layer_ = config_->res_2160p_layer_;
      } else if (request_height == RES_720P_HEIGHT &&
                 -1 != config_->res_720p_layer_) {
        config_->layer_ = config_->res_720p_layer_;
      } else {
        config_->layer_ = DEFAULT_1080P_LAYER;
        param.width = RES_1080P_WIDTH;
        param.height = RES_1080P_HEIGHT;
      }
      LOGW << "StreaOn: " << param.width << ", " << param.height << ", "
           << param.type << ", layer = " << config_->layer_;
      SetUvcStreamOn(0);
      usleep(10000);

      while (true) {
        if (!IsEncoderRunning()) {  // 判断是否还有编码动作没完成
          break;
        }
        usleep(5000);
      }
      if (param.type == uvccomponent::CODEC_NV12) {
        SetNv12IsOn(true);
        SetEncoderRunning(false);
        SetUvcStreamOn(1);
      } else {
        SetNv12IsOn(false);
        InitCodecManager(&param);
        usleep(10000);
        SetUvcStreamOn(1);
      }
    } break;
    case uvccomponent::UVC_ADD:
      ReInit();
      ReStart();
      break;
    case uvccomponent::UVC_REMOVE:
      if (uvc_server_) {
        uvc_server_->DeInit();
      }
      break;
    default:
      break;
  }
}

int UvcServerPlugin::InitCodecManager(vencParam *vencParam) {
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
  LOGW << "pic_width = " << pic_width << " pic_height = " << pic_height;
  int frame_buf_depth = 3;
  int is_cbr = config_->is_cbr_;
  int bitrate = config_->bitrate_;
  LOGW << "is_cbr = " << is_cbr << " bitrate = " << bitrate;

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

int UvcServerPlugin::DeinitCodecManager(int chn) {
  LOGI << "DeinitCodecManager";
  MediaCodecManager &manager = MediaCodecManager::Get();
  manager.EncodeChnStop(chn);
  manager.EncodeChnDeInit(chn);
  // manager.ModuleDeInit();
  return 0;
}

}  // namespace xproto
