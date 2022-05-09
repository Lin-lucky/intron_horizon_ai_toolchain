/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIDEO_SOURCE_H_
#define VIDEO_SOURCE_VIDEO_SOURCE_H_
#include <vector>
#include <string>
#include <memory>
#include "video_source/video_source_type.h"

#define IN
#define OUT
#define INOUT

namespace videosource {

class VinModule;
class VpsModule;

enum VideoSourceLogLevel {
  kLOG_LEVEL_FATAL = 0,
  kLOG_LEVEL_ERROR,
  kLOG_LEVEL_WARN,
  kLOG_LEVEL_INFO,
  kLOG_LEVEL_DEBUG,
  kLOG_LEVEL_VERBOSE
};

enum VideoSourceType {
  kSOURCE_TYPE_NONE = 0,
  kSOURCE_TYPE_MIPI_CAM,
  kSOURCE_TYPE_USB_CAM,
  kSOURCE_TYPE_FEEDBACK,
  kSOURCE_TYPE_RTSP,
};

class VideoSource {
 public:
  VideoSource() = delete;
  explicit VideoSource(const int &channel_id,
      const std::string &config_file);
  ~VideoSource();
  int Init();
  int DeInit();
  int Start();
  int Stop();

  VideoSourceType GetSourceType() { return source_type_; }
  void SetLoggingLevel(VideoSourceLogLevel &log_level);
  /**
   * @description: Get one nv12 image frame data from diffent vin(video_input)
   * module, such as mipi_cam, usb_cam, feedback, rtsp_stream and so on.
   * @param[in]  vin_nv12_image is nullptr
   * @param[out] vin_nv12_image is shared_ptr which including image data
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int GetVinImageFrame(OUT std::shared_ptr<ImageFrame> &nv12_image);
  /**
   * @description: Free one or more nv12 image frame data from vin module
   * @param[in]  vin_nv12_image is shared pointer being used by user
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int FreeVinImageFrame(IN std::shared_ptr<ImageFrame> &nv12_image);
  /**
   * @description: Get one or more nv12 images frame data from vps(video
   * processsing system) hardware module in xj3 or j5.If soc is xj3,
   * image frame data is derived from ipu multi-channel ouput in vps.
   * @param[in]  vps_nv12_image is nullptr
   * @param[out] vps_nv12_image is shared_ptr which including image data
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int GetVpsImageFrame(OUT std::vector<std::shared_ptr<ImageFrame>>
      &nv12_image_list);
  /**
   * @description: Free one or more nv12 image frame data from vps module
   * @param[in]  vps_nv12_image is shared pointer being used by user
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int FreeVpsImageFrame(IN std::vector<std::shared_ptr<ImageFrame>>
      &nv12_image_list);
  /**
   * @description: Get one pyramid image frame data from pyramid
   * hardware module in xj3 or j5.
   * @param[in]  pym_image is nullptr
   * @param[out] pym_image is shared_ptr which including pyramid data
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int GetPyramidFrame(OUT std::shared_ptr<PyramidFrame> &pym_image);
  /**
   * @description: Free pyramid frame data from pyramid module
   * @param[in]  pym_image is shared pointer being used by user
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int FreePyramidFrame(IN std::shared_ptr<PyramidFrame> &pym_image);

  bool GetVinOutEnable() { return vin_out_en_; }
  bool GetVpsEnable() { return vps_en_; }

 private:
  int LoadConfig(const std::string &input_file);
  int SendFrameToVps(const std::shared_ptr<ImageFrame> &nv12_image);

 private:
  int channel_id_;
  std::string config_file_;
  std::string data_source_;
  VideoSourceType source_type_ = kSOURCE_TYPE_NONE;
  std::string vin_config_file_;
  std::string vps_config_file_;
  bool vin_out_en_ = false;
  bool vps_en_ = false;
  bool init_flag_ = false;
  bool start_flag_ = false;
  std::shared_ptr<VinModule> vin_module_ = nullptr;
  std::shared_ptr<VpsModule> vps_module_ = nullptr;
  VideoSourceLogLevel log_level_ = kLOG_LEVEL_INFO;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIDEO_SOURCE_H_
