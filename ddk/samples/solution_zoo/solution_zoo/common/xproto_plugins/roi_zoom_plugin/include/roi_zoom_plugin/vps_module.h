/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_ROI_ZOOM_PLUGIN_VPS_MODULE_H_
#define INCLUDE_ROI_ZOOM_PLUGIN_VPS_MODULE_H_

#include <string.h>

#include <memory>
#include <thread>
#include <vector>

#include "vio/hb_vio_interface.h"
#include "roi_zoom_plugin/roi_zoom_common.h"

namespace xproto {
class VpsModule {
 public:
  VpsModule();
  ~VpsModule();
  int Init();
  int Start();
  int Stop();
  int DeInit();

 public:
  int Input(std::shared_ptr<VideoRoiData> &in_data);

  int UpdateViewRoi(const RoiInfo &info);
  int UpdateTrackRoi(const RoiInfo &info, const RoiInfo *info_tmp,
                     const bool is_second = false);

  int OutputViewData(std::shared_ptr<VideoRoiData> &out_data);
  int OutputTrackData(std::shared_ptr<VideoRoiData> &out_data,
                      const bool is_second = false);

 public:
  int Process(const RoiInfo &info, std::shared_ptr<VideoRoiData> &data,
              const RoiInfo *info_tmp = nullptr, const bool is_second = false,
              const bool send_video = true);

  int Process(std::shared_ptr<VideoRoiData> &in_data,
              std::shared_ptr<VideoRoiData> &out_data, const RoiInfo &info,
              const RoiInfo *info_tmp = nullptr, const bool is_second = false,
              const bool send_video = true);

 private:
  int GetFrame(const int group, const int channel, hb_vio_buffer_t *gdc_buf,
               std::shared_ptr<VideoRoiData> &out_data);

 private:
  int SysInit();
  int SysUnit();

 private:
  bool init_ = false;
  uint32_t timeout_ = 2000;
  mutable std::mutex mutex_;
  hb_vio_buffer_t *buffer_ = nullptr;
  hb_vio_buffer_t *view_buf = nullptr;
  hb_vio_buffer_t *track1_buf = nullptr;
  hb_vio_buffer_t *track2_buf = nullptr;
};

}  // namespace xproto
#endif  // INCLUDE_ROI_ZOOM_PLUGIN_VPS_MODULE_H_
