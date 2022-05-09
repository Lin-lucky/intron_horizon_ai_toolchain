/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIN_FEEDBACK_VIN_MODULE_H_
#define VIDEO_SOURCE_VIN_FEEDBACK_VIN_MODULE_H_
#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include "video_source/vin/vin_module.h"

namespace videosource {

struct FeedbackVinConfig {
  FeedbackVinConfig() {}
  ~FeedbackVinConfig() {}
  std::string feedback_type;
  std::string file_list_path;
  int image_interval;
  int cached_buf_count;
  bool file_list_loop;
  int width;
  int height;
};

class FeedbackVinModule : public VinModule {
 public:
  FeedbackVinModule() = delete;
  explicit FeedbackVinModule(const int &channel_id, const int &group_id)
    : VinModule("feedback_vin_module", channel_id, group_id) {}
  ~FeedbackVinModule() {}

  int Init(const std::string &config_file) override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  int ReadImage(char* data, const uint32_t &len,
      const uint32_t &width, const uint32_t &height,
      const HorizonVisionPixelFormat &pixel_format) override;

 private:
  int LoadConfig(const std::string &config_file);
  int ParseImageListFile();
  int FillVIOImageByImagePath(const std::string &image_name);
  int VinCreateDataThread();
  int VinDestoryDataThread();
  // 1. image feedback
  int ImageSourceInit();
  // 2. image list feedback
  void ImageListDataThread();
  // 3. video feedback
  void VideoDataThread();
  int InitFFMPEG();
  int DeInitFFMPEG();
  int OpenCodecContext(int *stream_idx,
      AVCodecContext **dec_ctx, AVFormatContext *fmt_ctx,
      enum AVMediaType type);
  std::string AVPixelFormatToStr(AVPixelFormat format_id);
  std::string AVCodecIDToStr(AVCodecID codec_id);
  int PushFrame(AVPacket &av_frame);

 private:
  std::shared_ptr<FeedbackVinConfig> vin_cfg_ = nullptr;
  std::shared_ptr<FFMPEGStreamParams> ffmpeg_stream_params_ = nullptr;
  std::vector<std::string> image_source_list_;
  uint32_t all_img_count_ = 0;
  bool init_flag_ = false;
  bool is_running_ = false;
  std::shared_ptr<std::thread> data_thread_ = nullptr;
  int image_width_;
  int image_height_;
  uint64_t frame_id_ = 0;
  HorizonVisionPixelFormat buf_fmt_ =
    HorizonVisionPixelFormat::kHorizonVisionPixelFormatNone;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIN_FEEDBACK_VIN_MODULE_H_
