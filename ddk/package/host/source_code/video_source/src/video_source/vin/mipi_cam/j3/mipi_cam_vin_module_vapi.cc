/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2021-10-12
 * @Version: v0.0.1
 * @Brief: mipi camera vin module for j3 video source system.
 */
#include "video_source/vin/mipi_cam/j3/mipi_cam_vin_module_vapi.h"
#include "hobotlog/hobotlog.hpp"
#include "cam/hb_cam_interface.h"

namespace videosource {

std::once_flag J3MipiCamVinModule::cam_init_flag_;
std::atomic_int J3MipiCamVinModule::task_num_{0};

int J3MipiCamVinModule::Init(
    const std::shared_ptr<JsonConfigWrapper> &cfg_json) {
  int ret = -1;
  LOGI << "Enter J3MipiCamVinModule init, group_id: " << group_id_;

  if (init_flag_ == true) {
    LOGE << "J3MipiCamVinModule has been init, init_flag: " << init_flag_
      << " group_id: " << group_id_;
    return 0;
  }

  ret = LoadConfig(cfg_json);
  if (ret) {
    LOGE << "MipiCamVinModule LoadConfig failed!";
    return ret;
  }

  AddVinTask();
  std::call_once(cam_init_flag_,
      [&] () {
      // camera init all device
      LOGI << "Enter j3 hb_cam_init , group_id: " << group_id_;
      auto start_time = std::chrono::system_clock::now();
      ret = hb_cam_init(cam_index_, cam_cfg_file_.c_str());
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time).count();
      HOBOT_CHECK(ret == 0) << "hb_cam_init failed, ret: " << ret
      << " cam_index: " << cam_index_
      << " cam_cfg_file: " << cam_cfg_file_;
      LOGI << "j3 hb_cam_init cost_time: " << cost_time << "ms";
    });

  init_flag_ = true;
  LOGI << "j3MipiCamVinModule init success."
    << " group_id: " << group_id_
    << " cam_index: " << cam_index_
    << " cam_cfg_file: " << cam_cfg_file_;
  return 0;
}

int J3MipiCamVinModule::DeInit() {
  int ret = -1;
  LOGI << "Enter J3MipiCamVinModule deinit, group_id: " << group_id_;

  if (init_flag_ == false) {
    LOGE << "J3MipiCamVinModule has not init, init_flag: " << init_flag_
      << " group_id: " << group_id_;
    return -1;
  }

  FreeVinTask();
  if (GetVinTaskNum() == 0) {
    // camera deinit all pipeline
    ret = hb_cam_deinit(cam_index_);
    HOBOT_CHECK(ret == 0) << "hb_cam_deinit failed, ret: " << ret
      << " cam_index: " << cam_index_;
  }

  init_flag_ = false;
  LOGD << "J3MipiCamVinModule deinit success...";
  return 0;
}

int J3MipiCamVinModule::Start() {
  int ret = -1;
  LOGI << "Enter J3MipiCamVinModule start, group_id: " << group_id_;
  ret = hb_cam_start(group_id_);
  HOBOT_CHECK(ret == 0) << "hb_cam_start failed, ret: " << ret
    << " group_id: " << group_id_;
  vin_start_flag_ = true;  // mipi-cam vin->vps auto bind
  LOGD << "J3MipiCamVinModule start success...";
  return 0;
}

int J3MipiCamVinModule::Stop() {
  int ret;
  LOGI << "Enter J3MipiCamVinModule stop, group_id: " << group_id_;
  ret = hb_cam_stop(group_id_);
  HOBOT_CHECK(ret == 0) << "hb_cam_stop failed, ret: " << ret
    << " group_id: " << group_id_;
  LOGD << "J3MipiCamVinModule stop success...";
  return 0;
}

int J3MipiCamVinModule::LoadConfig(
    const std::shared_ptr<JsonConfigWrapper> &vin_cfg_json) {
  if (vin_cfg_json == nullptr) {
    LOGE << "vin_json_cfg is nullptr";
    return -1;
  }
  if (vin_cfg_json->HasMember("cam_index")) {
    cam_index_ = vin_cfg_json->GetIntValue("cam_index");
  }
  if (vin_cfg_json->HasMember("cam_cfg_file")) {
    cam_cfg_file_ = vin_cfg_json->GetSTDStringValue("cam_cfg_file");
  }
  HOBOT_CHECK(cam_index_ >= 0);
  return 0;
}

}  // namespace videosource
