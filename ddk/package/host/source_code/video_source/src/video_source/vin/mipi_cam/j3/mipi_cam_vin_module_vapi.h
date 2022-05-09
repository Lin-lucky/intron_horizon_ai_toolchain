/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIN_J3_MIPI_CAM_VIN_MODULE_VAPI_H_
#define VIDEO_SOURCE_VIN_J3_MIPI_CAM_VIN_MODULE_VAPI_H_
#include <string>
#include <memory>
// #include <atomic>
#include "utils/json_cfg_wrapper.h"
#include "video_source/vin/vin_module.h"

namespace videosource {

class J3MipiCamVinModule : public VinModule {
 public:
  J3MipiCamVinModule() = delete;
  explicit J3MipiCamVinModule(const int &channel_id, const int &group_id)
    : VinModule("j3_mipi_camera_vin_module", channel_id, group_id) {}
  ~J3MipiCamVinModule() {}

  int Init(const std::shared_ptr<JsonConfigWrapper> &cfg_json) override;
  int DeInit() override;
  int Start() override;
  int Stop() override;

 private:
  int LoadConfig(const std::shared_ptr<JsonConfigWrapper> &vin_cfg_json);
  static int GetVinTaskNum() { return task_num_; }
  static void AddVinTask() { task_num_++; }
  static void FreeVinTask() { task_num_--; }

 private:
  int cam_index_ = -1;
  std::string cam_cfg_file_;
  bool init_flag_ = false;
  static std::once_flag cam_init_flag_;
  static std::atomic_int task_num_;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIN_J3_MIPI_CAM_VIN_MODULE_VAPI_H_
