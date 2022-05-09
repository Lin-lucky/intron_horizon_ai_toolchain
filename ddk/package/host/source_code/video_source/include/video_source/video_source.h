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
#include <chrono>
#include <mutex>
#include <atomic>
#include <semaphore.h>
#include <condition_variable>
#include <thread>
#include "video_source/video_source_type.h"

#define IN
#define OUT
#define INOUT

namespace videosource {

class VinModule;
class VpsModule;
class JsonConfigWrapper;

enum VideoSourceErrorCode {
  kERROR_CODE_NULL_POINTER = 1000,
  kERROR_CODE_FILE_NO_EXIST,
  kERROR_CODE_FILE_OPERATION_FAILED,
  kERROR_CODE_ILLEAGL_PARAMETER,
  kERROR_CODE_SET_PARAMETER_FAILED,
  kERROR_CODE_GET_PARAMETER_FAILED,  // 5
  kERROR_CODE_STOP_FAILED,
  kERROR_CODE_DEINIT_FAILED,
  kERROR_CODE_SEND_FRAME_FAILED,
  kERROR_CODE_GET_FRAME_FAILED,  // 9
  kERROR_CODE_FREE_FRAME_FAILED,
  kERROR_CODE_DYNAMIC_LOAD_VPS_FAILED,
  kERROR_CODE_READ_IMAGE_FAILED,
  kERROR_CODE_TIMEOUT,
  kERROR_CODE_SOURCE_IS_STOP,  // 14
  kERROR_CODE_SOURCE_UNKNOWN,
};

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
  kSOURCE_TYPE_RTSP_CLIENT,
  kSOURCE_TYPE_MAX,
};

class VideoSource {
 public:
  VideoSource() = delete;
  /**
   * video source class. including
   * -- a) mipi camera
   * -- b) usb camera
   * -- c) feedback(nv12 image list, jpeg image list)
   * -- d) rtsp client
   */
  explicit VideoSource(const int &channel_id,
      const std::string &config_file);
  /**
   * video source class. only including
   * -- a) single image feedback(
   *  support single nv12 image or jpeg image in ddr or string path)
   */
  explicit VideoSource(const std::string &config_file);
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
  /**
   * @description: Read image into video source module,
   * support jpg and nv12 image format
   * @param[in]  image path
   * @param[in]  image width
   * @param[in]  image height
   * @param[in]  image format
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int ReadImage(IN const std::string &path,
      IN const uint32_t &width, IN const uint32_t &height,
      IN const HorizonVisionPixelFormat &pixel_format);
  /**
   * @description: Read image into video source module,
   * support jpg and nv12 image format
   * @param[in]  image data pointer
   * @param[in]  image data length
   * @param[in]  image width
   * @param[in]  image height
   * @param[in]  image format
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int ReadImage(IN char* data, IN const uint32_t &len,
      IN const uint32_t &width, IN const uint32_t &height,
      IN const HorizonVisionPixelFormat &pixel_format);
  /**
   * @description: get vin module is enable or not
   * @param[in]  null
   * @return The interface returns vin enable flag
   */
  bool GetVinEnable() { return vin_en_; }
  /**
   * @description: get vin module output is enable or not
   * @param[in]  null
   * @return The interface returns vin output enable flag
   */
  bool GetVinOutEnable() { return vin_out_en_; }
  /**
   * @description: get vps module is enable or not
   * @param[in]  null
   * @return The interface returns vps enable flag
   */
  bool GetVpsEnable() { return vps_en_; }
  /**
   * @description: get sync mode is enable or not
   * @param[in]  null
   * @return The interface returns sync mode enable flag
   */
  bool GetSyncMode() { return is_sync_mode_; }
  /**
   * @description: send frame to vps in sync mode
   * @param[in]  vin_nv12_image is shared pointer being used by user
   * @return The interface returns 0 to indicate that the function is
   * successful, otherwise it indicates that the return failed.
   */
  int SendFrameToVps(const std::shared_ptr<ImageFrame> &nv12_image);

 private:
  int LoadConfig(const std::string &input_file);
  int LoadX3Config(const std::shared_ptr<JsonConfigWrapper> &json_cfg);
  int LoadJ3Config(const std::shared_ptr<JsonConfigWrapper> &json_cfg);
  int SetVinFrameCompleted(const uint64_t &frame_id);
  int DynamicLoadVps(int width, int height);
  bool VpsStartTimeoutWait(const std::chrono::milliseconds &timeout);
  void PrintVersion();
  int SetSyncMode();
  int ManagerDeviceStatus(const std::atomic_bool &status);
  int X3Init();
  int J3Init();

 private:
  int channel_id_;
  std::string config_file_;
  std::string data_source_;
  int group_id_;
  VideoSourceType source_type_ = kSOURCE_TYPE_NONE;
  std::string vin_config_file_;
  std::string vps_config_file_;
  std::vector<std::string> vps_config_file_list_;
  bool vin_en_ = true;
  bool vin_out_en_ = false;
  bool vps_en_ = true;
  std::atomic_bool init_flag_{false};
  std::atomic_bool start_flag_{false};
  int vin_frame_depth_ = 8;
  std::shared_ptr<VinModule> vin_module_ = nullptr;
  std::shared_ptr<VpsModule> vps_module_ = nullptr;
  VideoSourceLogLevel log_level_ = kLOG_LEVEL_INFO;
  std::atomic_bool is_vps_init_{false};
  std::atomic_bool is_vps_start_{false};
  std::condition_variable vps_condition_;
  std::mutex vps_mutex_;
  sem_t vps_start_sem_;
  bool is_sync_mode_ = false;
  std::atomic_bool is_source_stop_{false};
  std::string platform_type_ = "x3";
  std::shared_ptr<JsonConfigWrapper> j3_mipi_cam_vin_cfg_json_;
  std::shared_ptr<JsonConfigWrapper> j3_vps_cfg_json_;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIDEO_SOURCE_H_
