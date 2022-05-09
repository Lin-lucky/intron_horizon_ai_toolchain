/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2021-04-25
 * @Version: v0.0.1
 * @Brief: usb camera vin module for video source system.
 */
#include "video_source/vin/usb_cam/usb_cam_vin_module.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#include "hobotlog/hobotlog.hpp"

namespace videosource {

int UsbCamVinModule::LoadConfig(const std::string &config_file) {
  int ret = -1;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;

  ret = LoadConfigFile(config_file, json_cfg);
  if (ret) {
    LOGE << "load config file failed, ret: " << ret;
    return ret;
  }

  vin_cfg_ = std::make_shared<UsbCamVinConfig>();
  vin_cfg_->usb_dev_name = json_cfg->GetSTDStringValue("dev_name");
  vin_cfg_->usb_format = json_cfg->GetSTDStringValue("format");
  vin_cfg_->cached_buf_count = json_cfg->GetIntValue("cached_buf_count");
  vin_cfg_->width = json_cfg->GetIntValue("width");
  vin_cfg_->height = json_cfg->GetIntValue("height");

  LOGI << "usb_dev_name: " << vin_cfg_->usb_dev_name;
  LOGI << "usb_format: " << vin_cfg_->usb_format;
  LOGI << "cached_buf_count: " << vin_cfg_->cached_buf_count;
  LOGI << "width: " << vin_cfg_->width;
  LOGI << "height: " << vin_cfg_->height;

  return 0;
}

int UsbCamVinModule::Init(const std::string &config_file) {
  LOGD << "Enter UsbCamVinModule Init...";
  int ret = -1;

  if (init_flag_ == true) {
    LOGW << "usb camera vin module has been init!!!";
    return 0;
  }
  ret = LoadConfig(config_file);
  if (ret) {
    LOGE << "UsbCamVinModule LoadConfig failed!";
    return ret;
  }
  // set vin cached buffer count
  SetVinCachedBufCount(vin_cfg_->cached_buf_count);

  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  vin_buf_cfg.max_queue_len = vin_frame_depth_;
  vin_buf_cfg.use_vb = true;
  vin_buf_cfg.raw_pixel_len = 0;

  if (vin_cfg_->usb_format == "mjpeg") {
    fcc_ = FCC_MJPEG;
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatMJPEG;
    vin_buf_cfg.decode_en = true;
  } else if (vin_cfg_->usb_format == "yuy2") {
    fcc_ = FCC_YUY2;
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatYUY2;
    vin_buf_cfg.decode_en = false;
  } else {
    LOGE << "UnSupport usb format: " << vin_cfg_->usb_format;
    return -1;
  }
  this->SetVinBuf(vin_buf_cfg, false);

  ret = UsbCamInit();
  if (ret) {
    LOGE << "usb camera init failed, ret: " << ret;
    return ret;
  }

  init_flag_ = true;
  LOGI << "UsbCamVinModule Init success...";
  return 0;
}

int UsbCamVinModule::DeInit() {
  int ret = -1;

  if (init_flag_ == false) {
    LOGW << "feedback vin module has not init!!!";
    return 0;
  }

  ret = UsbCamDeInit();
  if (ret) {
    LOGE << "usb camera deinit failed, ret: " << ret;
    return ret;
  }

  ret = VinModule::VinDeInit();
  if (ret) {
    LOGE << "vin module deinit failed, ret: " << ret;
    return ret;
  }

  init_flag_ = false;
  LOGI << "UsbCamVinModule DeInit success...";
  return 0;
}

int UsbCamVinModule::Start() {
  int ret = -1;

  void *user_args = this;
  ret = camera_start_streaming(usb_cam_, got_frame_handler, user_args);
  if (ret < 0) {
    LOGE << "camera start streaming failed";
    camera_close(usb_cam_);
    return ret;
  }
  LOGI << "UsbCamVinModule Start success...";
  return 0;
}

int UsbCamVinModule::Stop() {
  int ret = -1;

  if (camera_stop_streaming(usb_cam_) < 0) {
    LOGE << "camera stop streaming failed";
  }

  ret = VinModule::VinStop();
  if (ret) {
    LOGE << "vin module stop failed, ret: " << ret;
    return ret;
  }
  LOGI << "UsbCamVinModule Stop success...";
  return 0;
}

int UsbCamVinModule::UsbCamInit() {
  int ret = -1;
  format_enums fmt_enums;
  std::string v4l2_devname = vin_cfg_->usb_dev_name;
  int width = vin_cfg_->width;
  int height = vin_cfg_->height;

  LOGI << "v4l2_devname: " << v4l2_devname;
  usb_cam_ = camera_open(v4l2_devname.data());
  if (!usb_cam_) {
    LOGW << "camer_open " << v4l2_devname << " failed, try to open /dev/video0";
    v4l2_devname = "/dev/video0";
    usb_cam_ = camera_open(v4l2_devname.data());
    if (!usb_cam_) {
      LOGW << "camer_open " << v4l2_devname
           << " failed, try to open /dev/video8";
      v4l2_devname = "/dev/video8";
      usb_cam_ = camera_open(v4l2_devname.data());
    }
  }
  if (!usb_cam_) {
    LOGE << "camera_open failed";
    return -1;
  }

  ret = camera_enum_format(usb_cam_, &fmt_enums, 0);
  if (ret < 0) {
    LOGE << "camera enum format failed";
    camera_close(usb_cam_);
    return ret;
  }
  camera_show_format(usb_cam_);

  camera_param_t params;
  params.fcc = fcc_;
  params.width = width;
  params.height = height;
  params.fps = 30;

  ret = camera_set_params(usb_cam_, &params);
  if (ret < 0) {
    LOGE << "camera set format failed";
    camera_close(usb_cam_);
    return ret;
  }

  return 0;
}

int UsbCamVinModule::UsbCamDeInit() {
  if (!usb_cam_) {
    return -1;
  }
  camera_close(usb_cam_);
  return 0;
}

void UsbCamVinModule::got_frame_handler(
    struct video_frame *data, void *user_args) {
  int ret = -1;
  UsbCamVinModule *vin_module = reinterpret_cast<UsbCamVinModule*>(user_args);
  std::shared_ptr<UsbCamVinConfig> vin_cfg = vin_module->vin_cfg_;
  fcc_format fcc = vin_module->fcc_;
  std::string usb_format = vin_cfg->usb_format;
  int width = vin_cfg->width;
  int height = vin_cfg->height;

  HOBOT_CHECK(width == data->width);
  HOBOT_CHECK(height == data->height);
  HOBOT_CHECK(fcc == data->fcc);

  // update max width and height
  int image_width = data->width;
  int image_height = data->height;
  int vin_image_width, vin_image_height;
  vin_module->GetVinOutputSize(vin_image_width, vin_image_height);
  VinBufferConfig vin_buf_cfg = { 0 };
  vin_module->GetVinBuf(vin_buf_cfg);
  HorizonVisionPixelFormat buf_fmt = vin_buf_cfg.format;
  if (vin_image_width != image_width
      || vin_image_height != image_height) {
    vin_image_width = image_width;
    vin_image_height = image_height;
    vin_module->SetVinOutputSize(vin_image_width, vin_image_height);
    vin_buf_cfg.max_width = ALIGN(vin_image_width, 16);
    vin_buf_cfg.max_height = ALIGN(vin_image_height, 16);
    vin_module->SetVinBuf(vin_buf_cfg, true);
  }

  // Input frame data to vin module
  VinFrame vin_frame = { 0 };
  vin_frame.format = buf_fmt;
  vin_frame.frame_id = vin_module->frame_id_++;
  vin_frame.plane_count = 1;
  vin_frame.vaddr[0] = data->mem;
  vin_frame.vaddr[1] = 0;
  vin_frame.paddr[0] = 0;
  vin_frame.paddr[1] = 0;
  vin_frame.width = data->width;
  vin_frame.height = data->height;
  vin_frame.stride = data->width;
  vin_frame.size = data->length;
  ret = vin_module->InputData(vin_frame);
  if (ret) {
    LOGE << "input data frame failed, ret: " << ret;
  }

  return;
}

}  // namespace videosource
