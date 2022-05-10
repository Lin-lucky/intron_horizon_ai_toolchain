/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     uvcplugin.cpp
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2020.5.12
 * \Brief    implement of api file
 */

#include "uvc_display_plugin/uvc_display_plugin.h"

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include "./hid_manager.h"
#include "./ring_queue.h"
#include "./rndis_manager.h"
#include "./smart_manager.h"
#include "./uvc_server.h"
#include "./uvcplugin_config.h"
#include "hobotlog/hobotlog.hpp"
#include "smart_message/smart_message.h"
#include "transport_message/uvc_message.h"
#include "xproto/message/msg_registry.h"
#ifdef USE_MC
#include "transport_message/monitor_control_message.h"
#endif
#include "utils/time_helper.h"
#include "xproto/msg_type//vio_message.h"

namespace xproto {
using xproto::XPluginErrorCode;
using xproto::message::SmartMessage;
using xproto::message::TransportMessage;
using xproto::message::VioMessage;
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_UVC_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_TRANSPORT_MESSAGE)
#ifdef USE_MC
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_APIMAGE_MESSAGE)
#endif
using std::chrono::milliseconds;
using hobot::Timer;
#define UEVENT_MSG_LEN 4096
#define EVENT_KEYBOARD_QUIT 1
#define EVENT_KOBJECT_UVC_ADD 2
#define EVENT_KOBJECT_UVC_REMOVE 3

UvcPlugin::UvcPlugin(std::string config_file) {
  config_file_ = config_file;
  LOGI << "UvcPlugin smart config file:" << config_file_;
  run_flag_ = false;
  dwc3_thread_ = nullptr;
  monitor_thread_ = nullptr;
  Reset();
}

UvcPlugin::~UvcPlugin() {
  //  config_ = nullptr;
}

int UvcPlugin::ParseConfig(std::string config_file) {
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
  if (json_obj.isMember("smart_transfer_mode")) {
    smart_transfer_mode_ = json_obj["smart_transfer_mode"].asUInt();
    LOGD << "smart transfer mode = " << smart_transfer_mode_;
  } else {
    LOGE << config_file << " should set smart_transfer mode";
    return -1;
  }
  return 0;
}

int UvcPlugin::Init() {
  LOGI << "UvcPlugin Init";
  RingQueue<VIDEO_STREAM_S>::Instance().Init(8, [](VIDEO_STREAM_S &elem) {
    if (elem.pstPack.vir_ptr) {
      free(elem.pstPack.vir_ptr);
      elem.pstPack.vir_ptr = nullptr;
    }
  });
  memset(&h264_sps_frame_, 0, sizeof(VIDEO_STREAM_S));

  if (0 != ParseConfig(config_file_)) {
    return -1;
  }

  uvc_server_ = std::make_shared<UvcServer>();
  if (uvc_server_->Init(config_file_)) {
    LOGE << "UwsPlugin Init uWS server failed";
    return -1;
  }

  if (smart_transfer_mode_ == 1) {
    smart_manager_ = std::make_shared<RndisManager>(config_file_);
  } else if (smart_transfer_mode_ == 0) {
    smart_manager_ = std::make_shared<HidManager>(config_file_);
  }
  else{
    LOGE << "not support this transefer mode";
    return -1;
  }

  if (smart_manager_->Init()) {
    LOGW << "UvcPlugin Init smartManager failed";
    //   return -1;
  }
  HOBOT_CHECK(uvc_server_ && smart_manager_);

  RegisterMsg(TYPE_IMAGE_MESSAGE,
  std::bind(&UvcPlugin::FeedVideoMsg, this, std::placeholders::_1));
  RegisterMsg(TYPE_DROP_IMAGE_MESSAGE, std::bind(&UvcPlugin::FeedVideoDropMsg,
                                                 this, std::placeholders::_1));
#ifdef USE_MC
  RegisterMsg(
          TYPE_MC_UPSTREAM_MESSAGE,
          std::bind(&UvcPlugin::FeedMc, this, std::placeholders::_1));
#endif
/*
  RegisterMsg(TYPE_SMART_MESSAGE,
              std::bind(&UvcPlugin::FeedSmart, this, std::placeholders::_1));
              */
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

int UvcPlugin::DeInit() {
  LOGD << "uvc plugin deinit...";
  if (uvc_server_) {
    uvc_server_->DeInit();
  }

  LOGD << "uvc server deinit done";
  if (smart_manager_) {
    smart_manager_->DeInit();
  }

  LOGD << "uvc hid_manager_ deinit done";
  return XPluginAsync::DeInit();
}

int UvcPlugin::Reset() {
  LOGI << __FUNCTION__ << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
  return 0;
}

int UvcPlugin::Start() {
  LOGD << "uvc start";
  if (run_flag_) {
    return 0;
  }
  run_flag_ = true;

  uvc_server_->Start();

  if (smart_manager_->Start() < 0) {
    LOGE << "start hid manager fail";
    return -1;
  }

  if (smart_manager_ && smart_manager_->IsApMode()
     && thread_process_ap_ == nullptr) {
    thread_process_ap_ = std::make_shared<std::thread>([=](){
        while (run_flag_) {
          std::string proto;
          if (smart_manager_->pb_ap2cp_info_queue_.try_pop(
                  &proto, std::chrono::microseconds(10 * 1000))) {
            auto msg = std::make_shared<TransportMessage>(proto);
            LOGD << "send msg " << msg->type_;// << " " << msg->proto_;
            PushMsg(msg);
          }
        }
    });
  }

  efd_ = eventfd(0, 0);
  monitor_flag_ = 1;

  if (dwc3_thread_ == nullptr) {
    dwc3_thread_ =
      std::make_shared<std::thread>(&UvcPlugin::Dwc3UeventMonitor, this);
  }

  if (monitor_thread_ == nullptr) {
    monitor_thread_ = std::make_shared<std::thread>(&UvcPlugin::MonitorEvent, this);
  }

  return 0;
}

int UvcPlugin::Stop() {
  if (!run_flag_) {
    return 0;
  }
  LOGI << "UvcPlugin::Stop()";
  run_flag_ = false;
  if (thread_process_ap_) {
    thread_process_ap_->join();
    thread_process_ap_ = nullptr;
  }
  smart_manager_->Stop();
  LOGI << "UvcPlugin hid stop done";
  if (uvc_server_) {
    uvc_server_->Stop();
  }
  LOGI << "UvcPlugin stop done";
  efd_ = 0;
  monitor_flag_ = 0;

  if (dwc3_thread_ != nullptr) {
    dwc3_thread_->join();
    dwc3_thread_ = nullptr;
  }

  if (monitor_thread_ != nullptr) {
    monitor_thread_->join();
    monitor_thread_ = nullptr;
  }

  return 0;
}

int UvcPlugin::FeedMc(XProtoMessagePtr msg) {
  if (!run_flag_) {
    return 0;
  }

  return smart_manager_ == nullptr ? 0 : smart_manager_->FeedInfo(msg);
}

int UvcPlugin::FeedSmart(XProtoMessagePtr msg) {
  LOGI << "recv smart";
  if (!run_flag_) {
    return 0;
  }
  if (print_timestamp_) {
    auto smart_msg = std::static_pointer_cast<SmartMessage>(msg);
    LOGW << "UvcPlugin::FeedSmart time_stamp: " << smart_msg->time_stamp_;
  }
  return smart_manager_->FeedSmart(msg, origin_image_width_, origin_image_height_,
                                 dst_image_width_, dst_image_height_);
}

int UvcPlugin::FeedVideoMsg(XProtoMessagePtr msg) {
  encode_thread_.PostTask(
      std::bind(&UvcPlugin::FeedVideo, this, msg));
  return 0;
}

int UvcPlugin::FeedVideoDropMsg(XProtoMessagePtr msg) {
  encode_thread_.PostTask(
      std::bind(&UvcPlugin::FeedVideoDrop, this, msg));
  return 0;
}


int UvcPlugin::FeedVideo(XProtoMessagePtr msg) {
  if (!run_flag_) {
    return 0;
  }
  if (UvcServer::IsUvcStreamOn() == 0) {
    return 0;
  }

  UvcServer::SetEncoderRunning(true);
  LOGD << "UvcPlugin Feedvideo";
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  auto vio_msg = std::dynamic_pointer_cast<VioMessage>(msg);
  int level = UvcServer::config_->layer_;

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

  if (!RingQueue<VIDEO_STREAM_S>::Instance().IsValid()) {
    UvcServer::SetEncoderRunning(false);
    return 0;
  }
  if (print_timestamp_) {
    LOGW << "FeedVideo time_stamp:" << vio_msg->time_stamp_;
  }
  if (UvcServer::IsNv12On()) {
    VIDEO_STREAM_S vstream;
    memset(&vstream, 0, sizeof(VIDEO_STREAM_S));
    auto buffer_size = pstFrame.stVFrame.size;
    vstream.pstPack.vir_ptr = (char *)calloc(1, buffer_size);
    vstream.pstPack.size = buffer_size;
    memcpy(vstream.pstPack.vir_ptr, pstFrame.stVFrame.vir_ptr[0],
                pstFrame.stVFrame.height * pstFrame.stVFrame.width);
    memcpy(vstream.pstPack.vir_ptr + pstFrame.stVFrame.height * pstFrame.stVFrame.width,
                pstFrame.stVFrame.vir_ptr[1],
                pstFrame.stVFrame.height * pstFrame.stVFrame.width / 2);
    RingQueue<VIDEO_STREAM_S>::Instance().Push(vstream);
    return 0;
  }
  auto ts0 = Timer::current_time_stamp();

  std::string user_data =
          ((UvcConfig::VIDEO_MJPEG == UvcServer::config_->video_type_ ||
            UvcConfig::VIDEO_JPG == UvcServer::config_->video_type_) ?
           "" : "dc45e9bd-e6d948b7-962cd820-d923eeef+") + std::to_string(vio_msg->time_stamp_);
  LOGD << "venc insert user_data:" << user_data;
  if (HB_VENC_InserUserData(0,
                            reinterpret_cast<uint8_t*>(const_cast<char*>
                            (user_data.data())), user_data.length()) != 0) {
    LOGE << "HB_VENC_InserUserData fail";
  }

  int ret = HB_VENC_SendFrame(0, &pstFrame, 0);
  if (ret < 0) {
    LOGE << "HB_VENC_SendFrame 0 error!!!ret " << ret;
    UvcServer::SetEncoderRunning(false);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    LOGW << "video_sended_without_recv_count: " << video_sended_without_recv_count_;
    return 0;
  } else {
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    ++video_sended_without_recv_count_;
  }

  VIDEO_STREAM_S vstream;
  memset(&vstream, 0, sizeof(VIDEO_STREAM_S));
  ret = HB_VENC_GetStream(0, &vstream, 2000);
  if (UvcServer::config_->h264_encode_time_ == 1) {
    auto ts1 = Timer::current_time_stamp();
    LOGI << "******Encode yuv to h264 cost: " << ts1 - ts0 << "ms";
  }
  if (ret < 0) {
    LOGE << "HB_VENC_GetStream timeout: " << ret;
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    LOGW << "video_sended_without_recv_count: " << video_sended_without_recv_count_;
  } else {
    auto video_buffer = vstream;
    auto buffer_size = video_buffer.pstPack.size;
    HOBOT_CHECK(buffer_size > 5) << "encode bitstream too small";
    int nal_type = -1;
    if ((0 == static_cast<int>(vstream.pstPack.vir_ptr[0])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[1])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[2])) &&
        (1 == static_cast<int>(vstream.pstPack.vir_ptr[3]))) {
      nal_type = static_cast<int>(vstream.pstPack.vir_ptr[4] & 0x1F);
    }
    LOGD << "nal type is " << nal_type;
    video_buffer.pstPack.vir_ptr = (char *)calloc(1, buffer_size);
    if (video_buffer.pstPack.vir_ptr) {
      memcpy(video_buffer.pstPack.vir_ptr, vstream.pstPack.vir_ptr,
          buffer_size);
      RingQueue<VIDEO_STREAM_S>::Instance().Push(video_buffer);
    }
    HB_VENC_ReleaseStream(0, &vstream);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    --video_sended_without_recv_count_;
  }

  UvcServer::SetEncoderRunning(false);
  return 0;
}

int UvcPlugin::FeedVideoDrop(XProtoMessagePtr msg) {
  if (!run_flag_) {
    return 0;
  }
  {
    auto vio_msg = std::dynamic_pointer_cast<VioMessage>(msg);
    if (vio_msg != nullptr) {
      smart_manager_->FeedDropSmart(vio_msg->sequence_id_);
    }
  }
  if (!uvc_server_->IsUvcStreamOn()) {
    return 0;
  }
  UvcServer::SetEncoderRunning(true);
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  auto vio_msg = std::dynamic_pointer_cast<VioMessage>(msg);
  int level = UvcServer::config_->layer_;
  
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

  if (!RingQueue<VIDEO_STREAM_S>::Instance().IsValid()) {
    UvcServer::SetEncoderRunning(false);
    return 0;
  }
  if (print_timestamp_) {
    LOGW << "FeedVideoDrop time_stamp:" << vio_msg->time_stamp_;
  }

  if (UvcServer::IsNv12On()) {
    VIDEO_STREAM_S vstream;
    memset(&vstream, 0, sizeof(VIDEO_STREAM_S));
    auto buffer_size = pstFrame.stVFrame.size;
    vstream.pstPack.vir_ptr = (char *)calloc(1, buffer_size);
    vstream.pstPack.size = buffer_size;
    memcpy(vstream.pstPack.vir_ptr, pstFrame.stVFrame.vir_ptr[0],
                pstFrame.stVFrame.height * pstFrame.stVFrame.width);
    memcpy(vstream.pstPack.vir_ptr + pstFrame.stVFrame.height * pstFrame.stVFrame.width,
                pstFrame.stVFrame.vir_ptr[1],
                pstFrame.stVFrame.height * pstFrame.stVFrame.width / 2);
    RingQueue<VIDEO_STREAM_S>::Instance().Push(vstream);
    return 0;
  }

  // For H264 and H265 encoder format, userdata format is uuid + user defined data (string)
  // For jpg and mjpeg encoder format, userdata format is only user defined data (string)
  std::string user_data =
          ((UvcConfig::VIDEO_MJPEG == UvcServer::config_->video_type_ ||
           UvcConfig::VIDEO_JPG == UvcServer::config_->video_type_) ?
          "" : "dc45e9bd-e6d948b7-962cd820-d923eeef+") + std::to_string(vio_msg->time_stamp_);
  LOGD << "venc insert user_data:" << user_data;
  if (HB_VENC_InserUserData(0,
                            reinterpret_cast<uint8_t*>(const_cast<char*>
                            (user_data.data())), user_data.length()) != 0) {
    LOGE << "HB_VENC_InserUserData fail";
  }

  int ret = HB_VENC_SendFrame(0, &pstFrame, 0);
  if (ret < 0) {
    LOGE << "HB_VENC_SendFrame 0 error!!!ret " << ret;
    UvcServer::SetEncoderRunning(false);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    LOGW << "video_sended_without_recv_count: " << video_sended_without_recv_count_;
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
    LOGW << "video_sended_without_recv_count: " << video_sended_without_recv_count_;
  } else {
    auto video_buffer = vstream;
    auto buffer_size = video_buffer.pstPack.size;
    HOBOT_CHECK(buffer_size > 5) << "encode bitstream too small";
    int nal_type = -1;
    if ((0 == static_cast<int>(vstream.pstPack.vir_ptr[0])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[1])) &&
        (0 == static_cast<int>(vstream.pstPack.vir_ptr[2])) &&
        (1 == static_cast<int>(vstream.pstPack.vir_ptr[3]))) {
      nal_type = static_cast<int>(vstream.pstPack.vir_ptr[4] & 0x1F);
    }
    LOGD << "nal type is " << nal_type;
    video_buffer.pstPack.vir_ptr = (char *)calloc(1, buffer_size);
    if (video_buffer.pstPack.vir_ptr) {
      memcpy(video_buffer.pstPack.vir_ptr, vstream.pstPack.vir_ptr,
          buffer_size);
      RingQueue<VIDEO_STREAM_S>::Instance().Push(video_buffer);
    }
    HB_VENC_ReleaseStream(0, &vstream);
    std::lock_guard<std::mutex> lk(video_send_mutex_);
    --video_sended_without_recv_count_;
  }
  UvcServer::SetEncoderRunning(false);
  return 0;
}

void UvcPlugin::Dwc3UeventMonitor() {
  struct uevent uevent;
  struct timeval timeout = {1, 0};  // 1s timeout
  char msg[UEVENT_MSG_LEN+2];
  uint64_t event;
  int sock, n;

  if (!efd_)
    return;

  sock = open_uevent_socket();
  if (sock < 0)
    return;

  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
    (const char *)&timeout, sizeof(timeout)) < 0) {
    fprintf(stderr, "setsockopt for socket receive timeout failed\n");
    return;
  }

  do {
    n = recv(sock, msg, UEVENT_MSG_LEN, 0);

    if (n < 0 || n > UEVENT_MSG_LEN)
      continue;

    msg[n] = '\0';
    msg[n+1] = '\0';

    parse_uevent(msg, &uevent);

    if (strncmp(uevent.subsystem, "video4linux", 11) == 0 &&
      strncmp(uevent.devname, "video", 5) == 0 &&
      strstr(uevent.path, "gadget")) {
      if (strncmp(uevent.action, "add", 3) == 0) {
        event = EVENT_KOBJECT_UVC_ADD;
        if (efd_)
          write(efd_, &event, sizeof(uint64_t));
      } else if (strncmp(uevent.action,
        "remove", 5) == 0) {
        event = EVENT_KOBJECT_UVC_REMOVE;
        if (efd_)
          write(efd_, &event, sizeof(uint64_t));
      }
    }
  } while (efd_); /* re-use efd as thread quit flag */

  close_uevent_socket(sock);

  return;
}

void UvcPlugin::MonitorEvent() {
  struct pollfd pfd;
  uint64_t event;
  ssize_t sz;

  pfd.fd = efd_;
  pfd.events = POLLIN;
  while (monitor_flag_) {
    if (poll(&pfd, 1, 0) == 1) {
      sz = read(efd_, &event, sizeof(uint64_t));
      if (sz != sizeof(uint64_t)) {
        fprintf(stderr, "eventfd read abnormal. sz(%lu)\n",
          sz);
        continue;
      }

      LOGD << "receive event " << event;
      switch (event) {
      case EVENT_KOBJECT_UVC_ADD:
        LOGD << "recv uvc kobject add event";
        ReInit();
        ReStart();
        break;
      case EVENT_KOBJECT_UVC_REMOVE:
        LOGD << "recv uvc kobject remove event";
        StopUvcHid();
        DeInit();
        break;
      default:
        break;
      }
    }
    usleep(300 * 1000);
  }
  return ;
}

int UvcPlugin::ReInit() {
  uvc_server_ = std::make_shared<UvcServer>();
  uvc_server_->Init(config_file_);

  if (smart_transfer_mode_ == 1) {
    smart_manager_ = std::make_shared<RndisManager>(config_file_);
  } else if (smart_transfer_mode_ == 0) {
    smart_manager_ = std::make_shared<HidManager>(config_file_);
  }
  else{
    LOGE << "not support this transefer mode";
    return -1;
  }

  if (smart_manager_->Init()) {
    LOGE << "UvcPlugin Init smartManager failed";
    return -1;
  }
  return 0;
}

int UvcPlugin::ReStart() {
  run_flag_ = true;

  uvc_server_->Start();
  if (smart_manager_->Start() < 0) {
    LOGE << "start hid manager fail";
    return -1;
  }

  if (smart_manager_ && smart_manager_->IsApMode()
     && thread_process_ap_ == nullptr) {
    thread_process_ap_ = std::make_shared<std::thread>([=](){
        while (run_flag_) {
          std::string proto;
          if (smart_manager_->pb_ap2cp_info_queue_.try_pop(
                  &proto, std::chrono::microseconds(10 * 1000))) {
            auto msg = std::make_shared<TransportMessage>(proto);
            LOGD << "send msg " << msg->type_;// << " " << msg->proto_;
            PushMsg(msg);
          }
        }
    });
  }
  return 0;
}

int UvcPlugin::StopUvcHid() {
  if (!run_flag_) {
    return 0;
  }
  LOGI << "UvcPlugin::Stop()";
  run_flag_ = false;
  if (thread_process_ap_) {
    thread_process_ap_->join();
    thread_process_ap_ = nullptr;
  }
  smart_manager_->Stop();
  LOGI << "UvcPlugin hid stop done";
  if (uvc_server_) {
    uvc_server_->Stop();
  }
  LOGI << "UvcPlugin stop done";
  return 0;
}

}  // namespace xproto
