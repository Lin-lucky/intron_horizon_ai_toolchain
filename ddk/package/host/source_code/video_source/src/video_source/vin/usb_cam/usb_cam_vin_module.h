/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIN_USB_CAM_VIN_MODULE_H_
#define VIDEO_SOURCE_VIN_USB_CAM_VIN_MODULE_H_
#include <string>
#include <memory>
#include <vector>
#include "video_source/vin/vin_module.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "uvc/camera.h"
#ifdef __cplusplus
}
#endif

namespace videosource {

struct UsbCamVinConfig {
  UsbCamVinConfig() {}
  ~UsbCamVinConfig() {}
  std::string usb_dev_name;
  std::string usb_format;
  int cached_buf_count;
  int width = 1920;
  int height = 1080;
};

class UsbCamVinModule : public VinModule {
 public:
  UsbCamVinModule() = delete;
  explicit UsbCamVinModule(const int &channel_id, const int &group_id)
    : VinModule("usb_cam_vin_module", channel_id, group_id) {}
  ~UsbCamVinModule() {}

  int Init(const std::string &config_file) override;
  int DeInit() override;
  int Start() override;
  int Stop() override;

 private:
  int LoadConfig(const std::string &config_file);
  int UsbCamInit();
  int UsbCamDeInit();
  static void got_frame_handler(struct video_frame *frame, void *user_args);

 private:
  std::shared_ptr<UsbCamVinConfig> vin_cfg_ = nullptr;
  bool init_flag_ = false;
  camera_t *usb_cam_ = nullptr;
  fcc_format fcc_ = FCC_MJPEG;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIN_USB_CAM_VIN_MODULE_H_
