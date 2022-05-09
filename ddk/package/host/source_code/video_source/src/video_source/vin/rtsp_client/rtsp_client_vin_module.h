/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIN_RTSP_VIN_MODULE_H_
#define VIDEO_SOURCE_VIN_RTSP_VIN_MODULE_H_
#include <string>
#include <memory>
#include "video_source/vin/vin_module.h"

namespace videosource {

struct RtspClientVinConfig {
  RtspClientVinConfig() {}
  ~RtspClientVinConfig() {}
  std::string rtsp_link;
  bool is_tcp = false;
  int frame_max_size = 200;  // default 200k
  int cached_buf_count = 30;
  int max_reconnect_count = 5;
  int max_reconnect_err_count = 0;
  bool is_save_stream = false;
  bool is_log_debug = false;
  std::string analyze_duration;
  std::string probe_size;
};

class RtspClientVinModule : public VinModule {
 public:
  RtspClientVinModule() = delete;
  explicit RtspClientVinModule(const int &channel_id, const int &group_id)
    : VinModule("rtsp_client_vin_module", channel_id, group_id) {}
  ~RtspClientVinModule() {}

  int Init(const std::string &config_file) override;
  int DeInit() override;
  int Start() override;
  int Stop() override;

 private:
  int LoadConfig(const std::string &config_file);
  int RtspClientStart();
  int RtspClientStop();
  void HbGetDataThread();
  int InitFFMPEG();
  int DeInitFFMPEG();
  int ParseVideoStreamInfo();
  int ConnectRtspLink();
  int OpenCodecContext(int *stream_idx,
      AVCodecContext **dec_ctx, AVFormatContext *fmt_ctx,
      enum AVMediaType type);
  std::string AVPixelFormatToStr(AVPixelFormat format_id);
  std::string AVCodecIDToStr(AVCodecID codec_id);
  int PushFrame(AVPacket &av_frame);

 private:
  std::shared_ptr<RtspClientVinConfig> vin_cfg_ = nullptr;
  std::shared_ptr<FFMPEGStreamParams> ffmpeg_stream_params_ = nullptr;
  bool init_flag_ = false;
  std::shared_ptr<std::thread> data_thread_ = nullptr;
  bool is_running_ = false;
  bool ffmpeg_init_flag_ = false;
  int image_width_;
  int image_height_;
  HorizonVisionPixelFormat buf_fmt_ =
    HorizonVisionPixelFormat::kHorizonVisionPixelFormatNone;
};



}  // namespace videosource
#endif  // VIDEO_SOURCE_VIN_RTSP_VIN_MODULE_H_
