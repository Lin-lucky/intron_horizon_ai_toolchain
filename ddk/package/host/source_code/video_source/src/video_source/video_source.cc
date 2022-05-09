/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-08
 *  @Version: v0.0.1
 *  @Brief: implemenation of video source system.
 */

#include "video_source/video_source.h"
#include <cerrno>
#include <cstdint>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <ostream>
#include <string>
#include <memory>
#include <fstream>
#include <functional>
#include "hobotlog/hobotlog.hpp"
#include "utils/json_cfg_wrapper.h"
#include "video_source/vps/vps_module.h"
#include "video_source/vps/vps_manager.h"
#include "video_source/vps/vps_data_type.h"
#ifdef X3
#include "video_source/vin/mipi_cam/x3/mipi_cam_vin_module_hapi.h"
#include "video_source/vin/mipi_cam/j3/mipi_cam_vin_module_vapi.h"
#endif
#include "video_source/vin/feedback/feedback_vin_module.h"
#include "video_source/vin/rtsp_client/rtsp_client_vin_module.h"
#include "video_source/vin/usb_cam/usb_cam_vin_module.h"
#include "video_source/version.h"

namespace videosource {

// video source construct
VideoSource::VideoSource(const int &channel_id,
    const std::string &config_file) {
  int ret = -1;
  int chret = 0;
  channel_id_ = channel_id;
  config_file_ = config_file;
  HOBOT_CHECK(channel_id >= 0)
    << "channel id must set greater than or equal 0, "
    << " channel_id: " << channel_id;
  ret = LoadConfig(config_file);
  HOBOT_CHECK(ret == 0);

  if (vin_en_ == false && vps_en_ == false) {
    chret = -1;
  }
  HOBOT_CHECK(chret == 0)
    << "video source vin and vps module not enable,"
    << " vin_en: " << vin_en_ << " vps_en: " << vps_en_;

  VpsManager &manager = VpsManager::Get();
  ret = manager.SetPlatformType(platform_type_);
  HOBOT_CHECK(ret == 0) << "vps manager set platform type failed.";
  if (vps_en_ == true) {
    // get group_id value and create vps module
    ret = manager.CreateVpsModule(channel_id_, group_id_, vps_module_);
    HOBOT_CHECK(ret == 0) << "create vps module failed, ret: " << ret;
    // video source vps module default async mode
    vps_module_->SetVpsSyncStatus(false);
  } else {
    // get group_id value
    ret = manager.GetGroupId(channel_id_, group_id_);
    HOBOT_CHECK(ret == 0) << "get group id failed, ret: " << ret;
  }

  if (vin_en_ == true) {
    if (data_source_ == "mipi_cam") {
      // create mipi camera video source device
      if (platform_type_ == "x3") {
        vin_module_ = std::make_shared<MipiCamVinModule>(
            channel_id_, group_id_);
      } else if (platform_type_ == "j3") {
        vin_module_ = std::make_shared<J3MipiCamVinModule>(
            channel_id_, group_id_);
      }
      source_type_ = kSOURCE_TYPE_MIPI_CAM;
    } else if (data_source_ == "usb_cam") {
      // create usb camera video source device
      vin_module_ = std::make_shared<UsbCamVinModule>(channel_id_, group_id_);
      source_type_ = kSOURCE_TYPE_USB_CAM;
    } else if (data_source_ == "feedback") {
      // create feedback video source device
      vin_module_ = std::make_shared<FeedbackVinModule>(channel_id_, group_id_);
      source_type_ = kSOURCE_TYPE_FEEDBACK;
    } else if (data_source_ == "rtsp_client") {
      // create rtsp video source device
      vin_module_ = std::make_shared<RtspClientVinModule>(
          channel_id_, group_id_);
      source_type_ = kSOURCE_TYPE_RTSP_CLIENT;
    } else {
      LOGE << "unsupported mode:" << data_source_;
      chret = -1;
    }
    HOBOT_CHECK(chret == 0) << "create vin module failed";
    // video source vin module default async mode
    vin_module_->SetVinSyncStatus(false);
    // if vps disable set vin output fps log
    if (vps_en_ == false) {
      vin_module_->SetVinFpsOutput(true);
    }
  }
  // set sync mode
  if (is_sync_mode_) {
    chret = SetSyncMode();
    HOBOT_CHECK(chret == 0) << "set sync module failed";
  }
}

// image source construct
VideoSource::VideoSource(const std::string &config_file) {
  int ret = -1;
  int chret = 0;
  channel_id_ = -1;  // image source ,channel_id is equal -1
  config_file_ = config_file;
  ret = LoadConfig(config_file);
  HOBOT_CHECK(ret == 0);
  is_sync_mode_ = true;  // image source, only support sync mode

  VpsManager &manager = VpsManager::Get();
  manager.SetPlatformType(platform_type_);
  /* 1. creat vps module if vps enable and get group_id value */
  if (vps_en_ == true) {
    ret = manager.CreateVpsModule(channel_id_, group_id_, vps_module_);
    HOBOT_CHECK(ret == 0) << "create vps module failed, ret: " << ret;
    // set vps sync mode
    vps_module_->SetVpsSyncStatus(true);
  } else {
    // get group_id value
    ret = manager.GetGroupId(channel_id_, group_id_);
    HOBOT_CHECK(ret == 0) << "get group id failed, ret: " << ret;
  }

  /* 2. creat vin module if vin enable*/
  if (vin_en_ == true) {
    if (data_source_ == "feedback") {
      // create static image feedback source device
      vin_module_ = std::make_shared<FeedbackVinModule>(channel_id_, group_id_);
      source_type_ = kSOURCE_TYPE_FEEDBACK;
    } else {
      LOGE << "image video source unsupported mode:" << data_source_;
      chret = -1;
    }
    HOBOT_CHECK(chret == 0) << "create vin module failed";
    // set vin sync mode
    vin_module_->SetVinSyncStatus(true);
    // if vps disable set vin output fps log
    if (vps_en_ == false) {
      vin_module_->SetVinFpsOutput(true);
    }
  }

  ret = this->Init();
  HOBOT_CHECK(ret == 0) << "init video source failed, ret: " << ret;
  ret = this->Start();
  HOBOT_CHECK(ret == 0) << "start video source failed, ret: " << ret;
}

VideoSource::~VideoSource()  {
  int ret = -1;
  LOGD << "enter video source deconstruct, "
    << " channel_id: " << channel_id_;

  if (channel_id_ == -1) {  // image video source deinit
    ret = this->Stop();
    if (ret) {
      LOGE << "image video source stop failed, ret: " << ret;
    }
    this->DeInit();
    if (ret) {
      LOGE << "image video source deinit failed, ret: " << ret;
    }
  }
  if (vin_module_) {
    vin_module_ = nullptr;
  }
  VpsManager &manager = VpsManager::Get();
  if (vps_module_) {
    ret = manager.DestroyVpsModule(vps_module_);
    if (ret) {
      LOGE << "destroy vps module failed, ret: " << ret;
    }
    vps_module_ = nullptr;
  } else {
    ret = manager.FreeGroupId(group_id_);
    if (ret) {
      LOGE << "free group id failed, ret: " << ret;
    }
  }
  LOGW << "video source channel_id: " << channel_id_
    << " deconstruct success...";
}

int VideoSource::LoadX3Config(
    const std::shared_ptr<JsonConfigWrapper> &json_cfg) {
  std::string default_value = "";
  if (json_cfg == nullptr) {
    LOGE << "json cfg is nullptr";
    return -1;
  }
  if (json_cfg->HasMember("channel_id")) {
    auto chn_id = json_cfg->GetIntValue("channel_id");
    if (chn_id != channel_id_) {
      LOGE << "video source construct channel_id: " << channel_id_
        << " is not same video_source_config_file channel_id: " << chn_id;
      return -kERROR_CODE_ILLEAGL_PARAMETER;
    }
  }
  if (json_cfg->HasMember("is_sync_mode")) {
    is_sync_mode_ = json_cfg->GetBoolValue("is_sync_mode");
  }
  if (json_cfg->HasMember("vin_en")) {
    vin_en_ = json_cfg->GetBoolValue("vin_en");
  }
  if (json_cfg->HasMember("vin_out_en")) {
    vin_out_en_ = json_cfg->GetBoolValue("vin_out_en");
  }
  if (json_cfg->HasMember("vin_frame_depth")) {
    vin_frame_depth_ = json_cfg->GetIntValue("vin_frame_depth");
  }
  if (json_cfg->HasMember("vps_en")) {
    vps_en_ = json_cfg->GetBoolValue("vps_en");
  }

  auto cfg_file = json_cfg->GetSubConfig("config_file");
  if (cfg_file == nullptr) {
    LOGE << "config file parameter is not exit!";
    return -kERROR_CODE_NULL_POINTER;
  }
  // static image source has not vin config file
  if (channel_id_ >= 0) {
    vin_config_file_ = cfg_file->GetSTDStringValue("vin_config_file");
    if (vin_config_file_ == default_value) {
      LOGE << "vin config file parameter is not exit!";
      return -kERROR_CODE_ILLEAGL_PARAMETER;
    }
  }
  vps_config_file_list_ = cfg_file->GetSTDStringArray("vps_config_file");
  if (vps_config_file_list_.size() == 0) {
    LOGE << "vps config file parameter is not exit!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  return 0;
}

int VideoSource::LoadJ3Config(
    const std::shared_ptr<JsonConfigWrapper> &json_cfg) {
  if (json_cfg == nullptr) {
    LOGE << "json cfg is nullptr";
    return -1;
  }

  // 1. parser vin config
  auto vin_cfg_json = json_cfg->GetSubConfig("vin_config");
  HOBOT_CHECK(vin_cfg_json);
  if (data_source_ == "mipi_cam") {
    j3_mipi_cam_vin_cfg_json_ = vin_cfg_json->GetSubConfig("mipi_cam");
    HOBOT_CHECK(j3_mipi_cam_vin_cfg_json_);
  } else {
    LOGE << "Unsupport data source: " << data_source_;
    return -1;
  }
  // 2. parser vps config
  j3_vps_cfg_json_ = json_cfg->GetSubConfig("vps_config");
  HOBOT_CHECK(j3_vps_cfg_json_);
  return 0;
}

int VideoSource::LoadConfig(const std::string &input_file) {
  int ret = -1;
  Json::Value cfg_jv;
  std::string default_value = "";
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;

  int Log_level = GetLogLevel();
  if (Log_level == 7) {  // HOBOT_LOG_NULL
    SetLogLevel(HOBOT_LOG_WARN);
    LOGW << "set default log level: [warn] ";
  } else {
    LOGW << "log level has been set, log_level: " << Log_level;
  }
  LOGI << "Load video source config file:" << input_file;
  std::ifstream infile(input_file);
  if (!infile) {
    LOGE << "error!!! video source config file is not exist"
      <<" input_file: " << input_file;
    return -kERROR_CODE_FILE_NO_EXIST;
  }
  infile >> cfg_jv;
  json_cfg.reset(new JsonConfigWrapper(cfg_jv));

  data_source_ = json_cfg->GetSTDStringValue("data_source");
  if (data_source_ == default_value) {
    LOGE << "data source parameter is not exit!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (json_cfg->HasMember("platform_type")) {
    platform_type_ = json_cfg->GetSTDStringValue("platform_type");
  } else {
    platform_type_ = "x3";
  }
  if (platform_type_ == "x3") {
    LOGI << "platform type is x3";
    ret = LoadX3Config(json_cfg);
    return ret;
  } else if (platform_type_ == "j3") {
    LOGI << "platform type is j3";
    ret = LoadJ3Config(json_cfg);
    return ret;
  } else {
    LOGI << "Unsupport platform type: " << platform_type_;
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  return 0;
}

int VideoSource::SetSyncMode() {
  // force set vin and vps run sync mode
  if (!vin_module_ && !vps_module_) {
    LOGE << "vin module and vps module is nullptr";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (init_flag_ == true) {
    LOGW << "video source has been init: " << init_flag_
      << " set sync mode must be before video source init";
    return 0;
  }
  // only support set videosource run sync mode in feedback source
  if (data_source_ == "feedback") {
    if (vin_module_) {
      if (vin_out_en_ == false) {
        LOGE << "vin_out_en must enable in sync mode";
        return -kERROR_CODE_ILLEAGL_PARAMETER;
      }
      vin_module_->SetVinSyncStatus(true);
    }
    if (vps_module_) {
      vps_module_->SetVpsSyncStatus(true);
    }
  }
  return 0;
}

void VideoSource::SetLoggingLevel(VideoSourceLogLevel &log_level) {
  switch (log_level) {
    case kLOG_LEVEL_FATAL:
      SetLogLevel(HOBOT_LOG_FATAL);
      break;
    case kLOG_LEVEL_ERROR:
      SetLogLevel(HOBOT_LOG_ERROR);
      break;
    case kLOG_LEVEL_WARN:
      SetLogLevel(HOBOT_LOG_WARN);
      break;
    case kLOG_LEVEL_INFO:
      SetLogLevel(HOBOT_LOG_INFO);
      break;
    case kLOG_LEVEL_DEBUG:
      SetLogLevel(HOBOT_LOG_DEBUG);
      break;
    case kLOG_LEVEL_VERBOSE:
      SetLogLevel(HOBOT_LOG_VERBOSE);
      break;
    default:
      SetLogLevel(HOBOT_LOG_INFO);
  }
}

void VideoSource::PrintVersion()
{
  static bool is_print = false;
  if (is_print == true) return;
  LOGW << "=========================================================";
  LOGW << "VideoSource VERSION: "      <<  VIDEO_SOURCE_VERSION_TIME;
  LOGW << "=========================================================";
  is_print = true;
}

int VideoSource::J3Init() {
  int ret = -1;
  if (!vin_module_ || !vps_module_) {
    LOGE << "vin_module or vps_module is nullptr";
    return -kERROR_CODE_NULL_POINTER;
  }

  if (data_source_ == "mipi_cam") {
    ret = vin_module_->Init(j3_mipi_cam_vin_cfg_json_);
    HOBOT_CHECK(ret == 0) << "vin module init failed!";
  }
  ret = vps_module_->Init(j3_vps_cfg_json_);
  HOBOT_CHECK(ret == 0) << "vps module init failed!";
  is_vps_init_ = true;
  return 0;
}

int VideoSource::X3Init() {
  int ret = -1;
  int bind_chn_id = -1;

  // 1. set vin frame depth and vin module init
  if (vin_module_) {
    vin_module_->SetOutputEnable(vin_out_en_);
    vin_module_->SetVinFrameDepth(vin_frame_depth_);
    ret = vin_module_->Init(vin_config_file_);
    HOBOT_CHECK(ret == 0) << "vin module init failed!";
    bind_chn_id = vin_module_->GetBindChannelId();
    LOGI << "video source chn_id: " << channel_id_
      << " bind_chn_id: " << bind_chn_id;
  }
  // 2. set vin and vps register data callback
  if (vin_module_ && vps_module_) {
    if ((channel_id_ == -1) || (is_sync_mode_ == false)) {
      if (data_source_ == "mipi_cam" && bind_chn_id < 0) {
        LOGI << "vin to vps bind by SetVinBindVps in mipi camera mode...";
      } else {
        // 2.1 register callback function for send nv12 frame
        // to vps in vin module
      vin_module_->SetDataHandleCb(
          std::bind(&VideoSource::SendFrameToVps,
            this, std::placeholders::_1));
      // 2.2 register callback function for set frame state
      // to vin in vps module
      vps_module_->SetFrameIdCb(
          std::bind(&VideoSource::SetVinFrameCompleted,
            this, std::placeholders::_1));
      }
    }
  }
  // 3. set vin register manager callback
  if (vin_module_) {
    vin_module_->SetManagerHandleCb(
        std::bind(&VideoSource::ManagerDeviceStatus,
          this, std::placeholders::_1));
  }

  /**
   * a) mipi camera module init in VideoSource Init
   * b) other module, such usb camra, feedback, rtsp_client
   * will dymamic init and start after recieve first video frame,
   * because of the different resolution
   *
   */
  if (data_source_ == "mipi_cam" && bind_chn_id < 0) {
    HOBOT_CHECK(vin_module_);
    if (vin_module_ && vps_module_) {
      int image_height, image_width;
      vin_module_->GetVinOutputSize(image_width, image_height);
      vps_module_->SetVpsInputSize(image_width, image_height);
      ret = vps_module_->Init(vps_config_file_list_);
      HOBOT_CHECK(ret == 0) << "vps module init failed!";
      is_vps_init_ = true;
      int group_id = vps_module_->GetGroupId();
      ret = vin_module_->SetVinBindVps(group_id);
      if (ret) {
        LOGE << "set vin vps bind failed!";
        return -kERROR_CODE_SET_PARAMETER_FAILED;
      }
    }
  }
  return 0;
}

int VideoSource::Init() {
  int ret = -1;
  int vps_config_index = 0;
  if (init_flag_ == true) {
    LOGW << "video source has been init, init flag: " << init_flag_
      << " channel_id: " << channel_id_;
    return 0;
  }
  PrintVersion();
  LOGI << "Enter video source init, platform_type: " << platform_type_;
  LOGI << "data_source: " << data_source_;
  if (platform_type_ == "x3") {
    LOGI << "x3 platform channel_id: " << channel_id_;
    LOGI << "is_sync_mode: " << is_sync_mode_;
    LOGI << "vin_enable: " << vin_en_;
    LOGI << "vin_out_enable: " << vin_out_en_;
    LOGI << "vin_frame_depth: " << vin_frame_depth_;
    LOGI << "vps_enable: " << vps_en_;
    LOGI << "vin_config_file: " << vin_config_file_;
    LOGI << "vps config file list: ";
    for (auto vps_config_file : vps_config_file_list_) {
      LOGI << "vps_config_file_index_" << vps_config_index++
        << " : " << vps_config_file;
    }
    ret = X3Init();
  } else if (platform_type_ == "j3") {
    ret = J3Init();
  }
  if (ret) {
    LOGE << "platform_type: " << platform_type_
      << " init failed, ret: " << ret;
    return ret;
  }
  init_flag_ = true;
  LOGD << "video source platform_type: " << platform_type_
    << " init success...";
  return 0;
}

int VideoSource::DynamicLoadVps(int width, int height) {
  int ret = -1;
  int curr_width, curr_height;
  bool update_flag = false;

  LOGD << "enter DynamicLoadVps."
    << " width: " << width << " height: " << height
    << " data_source: " << data_source_;

  // only support feedback, usb_cam, rtsp module
  HOBOT_CHECK(vps_module_);
  vps_module_->GetVpsInputSize(curr_width, curr_height);
  LOGI << "curr_width: " << curr_width
    << " curr_height: " << curr_height
    << " width: " << width << " height: " << height;
  if (curr_width != width || curr_height != height) {
    update_flag = true;
  }
  // reload vps module
  while (update_flag == true && start_flag_ == true) {
    LOGW << "start dynamic load vps module, chn_id: " << channel_id_;
    auto start_time = std::chrono::system_clock::now();
    if (is_vps_init_ == false || is_vps_start_ == false) {
      // init and start vps module
      if (is_vps_init_ == false) {
        vps_module_->SetVpsInputSize(width, height);
        ret = vps_module_->Init(vps_config_file_list_);
        HOBOT_CHECK(ret == 0) << "vps module init failed!";
        is_vps_init_ = true;
      }
      if (is_vps_start_ == false) {
        ret = vps_module_->Start();
        HOBOT_CHECK(ret == 0) << "vps module start failed!";
        std::unique_lock<std::mutex> lock(vps_mutex_);
        is_vps_start_ = true;
      }
      update_flag = false;
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - start_time).count();
      LOGW << "update vpsmodule success, cost_time: " << cost_time << "ms"
        << " image width from: " << curr_width << " to " << width
        << " image height from: " << curr_width << " to " << height;
      vps_condition_.notify_all();
    } else {
      // stop and deinit vps module
      if (is_vps_start_ == true) {
        ret = vps_module_->Stop();
        HOBOT_CHECK(ret == 0) << "vps module stop failed!";
        is_vps_start_ = false;
      }
      if (is_vps_init_ == true) {
        ret = vps_module_->DeInit();
        HOBOT_CHECK(ret == 0) << "vps module init failed!";
        is_vps_init_ = false;
      }
      LOGW << "deinit vpsmodule success for change vps image resolution";
    }
  }
  return 0;
}

int VideoSource::SetVinFrameCompleted(const uint64_t &frame_id) {
  int ret = -1;

  if (vin_module_ == nullptr) {
    LOGE << "vin module has not create!";
    return -kERROR_CODE_NULL_POINTER;
  }
  ret = vin_module_->SetFrameFromWorkedToCompleted(frame_id);
  if (ret) {
    LOGE << "set frame from worked to completed failed, ret: " << ret;
    return -kERROR_CODE_SET_PARAMETER_FAILED;
  }
  return 0;
}

int VideoSource::ManagerDeviceStatus(const std::atomic_bool &status) {
  if (status == true) {
    LOGW << "vin module is end of stream, video source need stop";
    is_source_stop_ = true;
  }
  return 0;
}

int VideoSource::SendFrameToVps(const std::shared_ptr<ImageFrame> &nv12_image) {
  int ret = -1;
  void *pvio_image = nullptr;
  LOGD << "Enter SendFrameToVps, vps_en: " << vps_en_
    << " is_vps_start: " << is_vps_start_;

  if (start_flag_ == false) {
    LOGE << "video source has not start!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vps_en_ == false) {
    LOGE << "vps module has not enable!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vps_module_ == nullptr) {
    LOGE << "vps module has not create!";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (nv12_image == nullptr) {
    LOGE << "nv12_image is nullptr!";
    return -kERROR_CODE_NULL_POINTER;
  }
  /**
   * dymamic load(init and start) vps module once
   * according to different resolution
   */
  if (is_vps_start_ == false) {
    int image_width = nv12_image->src_info_.width;
    int image_height = nv12_image->src_info_.height;
    ret = DynamicLoadVps(image_width, image_height);
    if (ret) {
      LOGE << "dymamic load vps module failed";
      return -kERROR_CODE_DYNAMIC_LOAD_VPS_FAILED;
    }
  }
  pvio_image = vps_module_->CreatePymAddrInfo();
  if (pvio_image == nullptr) {
    LOGE << "vps module create pym addr failed";
    return -kERROR_CODE_NULL_POINTER;
  }
  ret = vps_module_->ConvertVioBufInfo(nv12_image, pvio_image);
  if (ret) {
    LOGE << "vps module convert vio buf info failed!";
    return -kERROR_CODE_NULL_POINTER;
  }
  ret = vps_module_->SendFrame(pvio_image);
  if (ret) {
    LOGE << "vps module send frame failed!";
    return -kERROR_CODE_SEND_FRAME_FAILED;
  }
  if (pvio_image) {
    std::free(pvio_image);
    pvio_image = nullptr;
  }
  return 0;
}

int VideoSource::DeInit() {
  int ret = -1;
  LOGI << "Enter video source deinit...";

  if (init_flag_ == false) {
    LOGE << "video source has not init!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vin_module_) {
    ret = vin_module_->DeInit();
    if (ret) {
      LOGE << "vin module deinit failed!";
      return -kERROR_CODE_DEINIT_FAILED;
    }
  }
  if (vps_module_ && is_vps_init_) {
    ret = vps_module_->DeInit();
    if (ret) {
      LOGE << "vps module deinit failed!";
      return -kERROR_CODE_DEINIT_FAILED;
    }
  }
  is_vps_init_ = false;
  init_flag_ = false;
  LOGI << "video source deinit success...";
  return 0;
}

int VideoSource::Start() {
  int ret = -1;
  int bind_chn_id = -1;
  LOGI << "Enter video source start...";

  if (init_flag_ == false) {
    LOGE << "video source has not init, start failed";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }

  // only start mipi camera vps module
  if (vin_module_) {
    bind_chn_id = vin_module_->GetBindChannelId();
  }
  if (data_source_ == "mipi_cam" && bind_chn_id < 0) {
    if (vin_module_ && vps_module_) {
      ret = vps_module_->Start();
      HOBOT_CHECK(ret == 0) << "vps module start failed!";
      is_vps_start_ = true;
    }
  }
  // start vin module
  if (vin_module_) {
    ret = vin_module_->Start();
    HOBOT_CHECK(ret == 0) << "vin module start failed!";
    /**
     * image source channel_id is -1
     * image source vin module will not start until ReadImage interface called
     * video source channel_id >= 0, vin module must wait start flag
     */
    if (channel_id_ >= 0 && vin_module_->GetVinIsStart() == false) {
      LOGE << "vin module start timeout, chn_id: " << channel_id_;
      return -kERROR_CODE_TIMEOUT;
    }
  }

  start_flag_ = true;
  return 0;
}

int VideoSource::Stop() {
  int ret = -1;
  LOGI << "Enter video source stop...";

  if (start_flag_ == false) {
    LOGE << "video source has not start!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vin_module_) {
    ret = vin_module_->Stop();
    if (ret) {
      LOGE << "vin module stop failed!";
      return -kERROR_CODE_STOP_FAILED;
    }
  }
  if (vps_en_ && is_vps_start_ && vps_module_) {
    ret = vps_module_->Stop();
    if (ret) {
      LOGE << "vps module stop failed!";
      return -kERROR_CODE_STOP_FAILED;
    }
  }
  start_flag_ = false;
  is_vps_start_ = false;
  return 0;
}

int VideoSource::GetVinImageFrame(std::shared_ptr<ImageFrame> &nv12_image) {
  int ret = -1;
  if (start_flag_ == false) {
    LOGE << "video source has not start!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vin_module_ == nullptr) {
    LOGE << "vin module is not enable!";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (is_source_stop_ == true) {
    LOGW << "no data available, video source stop send data";
    return -kERROR_CODE_SOURCE_IS_STOP;
  }
  if (vin_module_->GetVinIsStart() == false) {
    LOGE << "vin module start timeout, chn_id: " << channel_id_;
    return -kERROR_CODE_TIMEOUT;
  }
  auto image_frame_ptr = std::make_shared<ImageFrame>();
  ret = vin_module_->GetFrame(image_frame_ptr);
  if (ret) {
    LOGE << "vin module get frame failed!";
    return -kERROR_CODE_GET_FRAME_FAILED;
  }
  nv12_image = image_frame_ptr;
  LOGI << "get vin image frame success, chn_id: " << channel_id_
    << " frame_id: " << nv12_image->frame_id_;
  return 0;
}

int VideoSource::FreeVinImageFrame(std::shared_ptr<ImageFrame> &nv12_image) {
  int ret = -1;
  if (nv12_image == nullptr) {
    LOGE << "nv12 image is nullptr";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (vin_module_ == nullptr) {
    LOGE << "vin module is not enable!";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (start_flag_ == false) {
    LOGE << "video source has not start!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  ret = vin_module_->FreeFrame(nv12_image);
  if (ret) {
    LOGE << "vin module free frame failed!";
    return -kERROR_CODE_FREE_FRAME_FAILED;
  }
  LOGD << "free vin image frame success, chn_id: " << channel_id_
    << " frame_id: " << nv12_image->frame_id_;
  nv12_image = nullptr;
  return 0;
}

int VideoSource::GetVpsImageFrame(std::vector<std::shared_ptr<ImageFrame>>
    &nv12_image_list) {
  int ret = -1;
  int chret = 0;
  void* pvio_image = nullptr;
  void* src_ctx = nullptr;
  std::vector<void*> pvio_images;
  std::vector<void*> src_ctxs;
  std::chrono::milliseconds timeout(10000);

  if (start_flag_ == false) {
    LOGE << "video source has not start!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vps_module_ == nullptr) {
    LOGE << "vps module is not enable!";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (is_source_stop_ == true) {
    LOGW << "no data available, video source stop send data";
    return -kERROR_CODE_SOURCE_IS_STOP;
  }
  if (VpsStartTimeoutWait(timeout) == false) {
    LOGE << "vps start timeout";
    return -kERROR_CODE_TIMEOUT;
  }
  if (nv12_image_list.size()) {
    LOGW << "nv12 image list size:" << nv12_image_list.size() << " need clear";
    nv12_image_list.clear();
  }
  std::vector<int> vps_chn_list;
  ret = vps_module_->GetVpsIpuChn(vps_chn_list);
  if (ret) {
    LOGE << "get vps chn failed, ret: " << ret;
    return -kERROR_CODE_GET_PARAMETER_FAILED;
  }
  for (size_t i = 0; i < vps_chn_list.size(); i++) {
    pvio_image = vps_module_->CreateSrcAddrInfo();
    if (pvio_image == nullptr) {
      LOGE << "create src addr failed";
      chret = -kERROR_CODE_NULL_POINTER;
      goto err;
    }
    pvio_images.push_back(pvio_image);
    int vps_chn = vps_chn_list[i];
    auto output = std::make_shared<ImageFrame>();
    ret = vps_module_->GetFrame(vps_chn, pvio_image);
    if (ret) {
      LOGE << "vps module get frame failed!";
      chret = -kERROR_CODE_GET_FRAME_FAILED;
      goto err;
    }
    ret = vps_module_->ConvertSrcInfo(pvio_image, output);
    if (ret) {
      LOGE << "vps module convert src info failed!";
      chret = -kERROR_CODE_NULL_POINTER;
      goto err;
    }
    src_ctx = vps_module_->CreateSrcContext(vps_chn, pvio_image);
    if (src_ctx == nullptr) {
      LOGE << "create src context failed";
      chret = -kERROR_CODE_NULL_POINTER;
      goto err;
    }
    src_ctxs.push_back(src_ctx);
    output->src_context_ = src_ctx;
    nv12_image_list.push_back(output);
    LOGI << "get vps image frame success, chn_id: " << channel_id_
      << " frame_id: " << output->frame_id_
      << " ipu_chn: " << vps_chn;
  }
  return 0;

err:
  // 1. clear pvio images
  for (size_t i = 0; i < pvio_images.size(); i++) {
    pvio_image = pvio_images[i];
    if (pvio_image) {
      std::free(pvio_image);
      pvio_image = nullptr;
    }
  }
  pvio_images.clear();
  // 2. clear src contexts
  for (size_t i = 0; i < src_ctxs.size(); i++) {
    src_ctx = src_ctxs[i];
    if (src_ctx) {
      std::free(src_ctx);
      src_ctx = nullptr;
    }
  }
  src_ctxs.clear();
  return chret;
}

int VideoSource::FreeVpsImageFrame(
    std::vector<std::shared_ptr<ImageFrame>> &nv12_image_list) {
  int ret = -1;
  int chret = 0;
  int vps_chn = -1;
  void *pvio_image = nullptr;
  std::vector<void*> pvio_images;
  std::vector<int> vps_chn_list;

  if (start_flag_ == false) {
    LOGE << "video source has not start!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vps_module_ == nullptr) {
    LOGE << "vps module is not enable!";
    return -kERROR_CODE_NULL_POINTER;
  }

  // 1. get pvio_images and vps_chn_list
  size_t vps_chn_num = nv12_image_list.size();
  for (size_t i = 0; i < vps_chn_num; i++) {
    std::shared_ptr<ImageFrame> nv12_image = nv12_image_list[i];
    if (nv12_image == nullptr) {
      LOGE << "nv12_image is nullptr!";
      chret = -kERROR_CODE_NULL_POINTER;
      continue;
    }
    auto src_ctx = nv12_image->src_context_;
    if (src_ctx == nullptr) {
      LOGE << "src_ctx pointer is nullptr";
      chret = -kERROR_CODE_NULL_POINTER;
      continue;
    }
    ret = vps_module_->GetSrcContext(src_ctx, vps_chn, pvio_image);
    if (ret) {
      LOGE << "get src context failed, ret: " << ret;
      chret = -kERROR_CODE_GET_PARAMETER_FAILED;
      if (src_ctx) {
        std::free(src_ctx);
      }
      continue;
    }
    if (src_ctx) {
      std::free(src_ctx);
    }
    pvio_images.push_back(pvio_image);
    vps_chn_list.push_back(vps_chn);
  }
  auto vio_num = pvio_images.size();
  if (vio_num != vps_chn_num) {
    LOGE << "vps chn output num not equal vio image num";
    chret = -kERROR_CODE_GET_PARAMETER_FAILED;
    goto err;
  }

  // 2. free vps module frame
  for (size_t i = 0; i < vps_chn_num; i++) {
    vps_chn = vps_chn_list[i];
    pvio_image = pvio_images[i];
    ret = vps_module_->FreeFrame(vps_chn, pvio_image);
    if (ret) {
      chret = -kERROR_CODE_FREE_FRAME_FAILED;
      LOGE << "free vps image frame success, failed: " << channel_id_
        << " vps_chn: " << vps_chn;
      goto err;
    }
    if (pvio_image) {
      std::free(pvio_image);
    }
    LOGD << "free vps image frame success, chn_id: " << channel_id_
      << " vps_chn: " << vps_chn;
  }
  nv12_image_list.clear();
  return 0;

err:
  // clear pvio images
  for (size_t i = 0; i < pvio_images.size(); i++) {
    pvio_image = pvio_images[i];
    if (pvio_image) {
      std::free(pvio_image);
      pvio_image = nullptr;
    }
  }
  pvio_images.clear();
  return chret;
}

bool VideoSource::VpsStartTimeoutWait(
    const std::chrono::milliseconds &timeout) {
  std::unique_lock<std::mutex> lock(vps_mutex_);
  if (!vps_condition_.wait_for(lock, timeout,
        [this] { return (is_vps_start_ == true); })) {
    return false;
  }
  return true;
}

int VideoSource::GetPyramidFrame(std::shared_ptr<PyramidFrame> &pym_image) {
  int ret =  -1;
  int vps_pym_chn;
  std::shared_ptr<PyramidFrame> output = nullptr;
  void *pvio_image = nullptr;
  void *src_image = nullptr;

  if (start_flag_ == false) {
    LOGE << "video source has not start!";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vps_module_ == nullptr) {
    LOGE << "vps module is not enable!";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (is_source_stop_ == true) {
    LOGW << "no data available, video source stop send data";
    return -kERROR_CODE_SOURCE_IS_STOP;
  }
  std::chrono::milliseconds timeout(20000);
  if (VpsStartTimeoutWait(timeout) == false) {
    LOGE << "vps start timeout";
    return -kERROR_CODE_TIMEOUT;
  }
  std::vector<int> vps_pym_list;
  ret = vps_module_->GetVpsPymChn(vps_pym_list);
  if (ret < 0) {
    LOGE << "get vps pym chn failed, ret: " << ret;
    return -kERROR_CODE_GET_PARAMETER_FAILED;
  }
  size_t pym_num = vps_pym_list.size();
  if (pym_num == 0) {
    return 0;  // pym chn has not enable by user
  }
  if (pym_num == 1) {
    vps_pym_chn = vps_pym_list[0];
  } else {
    LOGE << "vps pym chn only support max a pym chn in a vps group";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }

  output = std::make_shared<PyramidFrame>();
  pvio_image = vps_module_->CreatePymAddrInfo();
  if (pvio_image == nullptr) {
    LOGE << "vps module create pym addr failed";
    return -kERROR_CODE_NULL_POINTER;
  }
  if (platform_type_ == "j3") {
    src_image = vps_module_->CreateSrcAddrInfo();
    if (src_image == nullptr) {
      LOGE << "vps module create src addr failed";
      return -kERROR_CODE_NULL_POINTER;
    }
  }
  ret = vps_module_->GetPyramidFrame(vps_pym_chn, src_image, pvio_image);
  if (ret) {
    LOGE << "vps module get pyramid failed!";
    goto err;
  }
  output->pym_context_ = pvio_image;
  output->src_context_ = src_image;
  ret = vps_module_->ConvertPymInfo(pvio_image, output);
  if (ret) {
    LOGE << "vps module convert src info failed!";
    goto err;
  }
  pym_image = output;
  LOGI << "get pyramid frame success,   chn_id: " << channel_id_
    << " frame_id: " << pym_image->frame_id_
    << " pym_chn: " << vps_pym_chn;
  return 0;

err:
  if (pvio_image) {
    std::free(pvio_image);
    pvio_image = nullptr;
  }
  if (src_image) {
    std::free(src_image);
    src_image = nullptr;
  }
  return -kERROR_CODE_GET_FRAME_FAILED;
}

int VideoSource::FreePyramidFrame(std::shared_ptr<PyramidFrame> &pym_image) {
  int ret = -1;
  int vps_pym_chn;

  if (start_flag_ == false) {
    LOGE << "video source has not start, channel_id: " << channel_id_;
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (vps_module_ == nullptr) {
    LOGE << "vps module is not enable, channel_id: " << channel_id_;
    return -kERROR_CODE_NULL_POINTER;
  }
  std::vector<int> vps_pym_list;
  ret = vps_module_->GetVpsPymChn(vps_pym_list);
  if (ret < 0) {
    LOGE << "get vps pym chn failed, ret: " << ret;
    return -kERROR_CODE_GET_PARAMETER_FAILED;
  }
  size_t pym_num = vps_pym_list.size();
  if (pym_num == 0) {
    return 0;  // pym chn has not enable by user
  }
  if (pym_num == 1) {
    vps_pym_chn = vps_pym_list[0];
  } else {
    LOGE << "vps pym chn only support max a pym chn in a vps group";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (pym_image == nullptr) {
    LOGE << "pym image is nullptr!";  // check input pym_image
    return -kERROR_CODE_NULL_POINTER;
  }

  void* &pvio_image = pym_image->pym_context_;
  void* &src_image = pym_image->src_context_;
  if (pvio_image == nullptr) {
    LOGE << "pvio_image pointer is nullptr";
    return -kERROR_CODE_NULL_POINTER;
  }
  ret = vps_module_->FreePyramidFrame(vps_pym_chn,
      src_image, pvio_image);
  if (ret) {
    LOGE << "vps module free pyramid failed!";
    return -kERROR_CODE_FREE_FRAME_FAILED;
  }
  if (pvio_image) {
    std::free(pvio_image);
    pvio_image = nullptr;
  }
  if (src_image) {
    std::free(src_image);
    src_image = nullptr;
  }
  LOGD << "free pyramid frame success, chn_id: " << channel_id_
    << " frame_id: " << pym_image->frame_id_;
  return 0;
}

int VideoSource::ReadImage(const std::string &path,
    const uint32_t &width, const uint32_t &height,
    const HorizonVisionPixelFormat &pixel_format) {
  int ret = 0;
  int len;
  char *data = nullptr;

  if (channel_id_ != -1) {
    LOGE << "please create image source, this channel id is equal -1";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }
  if (access(path.c_str(), F_OK) != 0) {
    LOGE << "File not exist: " << path;
    return -kERROR_CODE_FILE_NO_EXIST;
  }
  std::ifstream ifs(path, std::ios::in | std::ios::binary);
  if (!ifs) {
    LOGE << "Failed load image: " << path;
    return -kERROR_CODE_FILE_OPERATION_FAILED;
  }
  ifs.seekg(0, std::ios::end);
  len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  data = new char[len];
  ifs.read(data, len);

  // vin module read image from user data
  if (vin_module_) {
    ret = vin_module_->ReadImage(data, len, width, height, pixel_format);
    if (ret) {
      LOGE << "read image from data failed, ret: " << ret;
      ret = -kERROR_CODE_READ_IMAGE_FAILED;
    }
  }
  delete[] data;
  ifs.close();
  return ret;
}

int VideoSource::ReadImage(char* data, const uint32_t &len,
    const uint32_t &width, const uint32_t &height,
    const HorizonVisionPixelFormat &pixel_format) {
  int ret = -1;
  if (channel_id_ != -1) {
    LOGE << "please create image source, this channel id is equal -1";
    return -kERROR_CODE_ILLEAGL_PARAMETER;
  }

  // vin module read image from user data
  if (vin_module_) {
    ret = vin_module_->ReadImage(data, len, width, height, pixel_format);
    if (ret) {
      LOGE << "read image from data failed, ret: " << ret;
      return -kERROR_CODE_READ_IMAGE_FAILED;
    }
  }
  return 0;
}

}  // namespace videosource
