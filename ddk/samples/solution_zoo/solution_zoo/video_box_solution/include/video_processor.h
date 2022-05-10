/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_VIDEOPROCESSOR_H_
#define INCLUDE_VIDEOPROCESSOR_H_

#include <string.h>

#include <memory>
#include <mutex>
#include <vector>

#include "blocking_queue/blocking_queue.hpp"
#include "venc_module.h"
#include "video_box_common.h"
#include "vot_module.h"

namespace solution {
namespace video_box {
class VideoProcessor {
 public:
  VideoProcessor();
  ~VideoProcessor();

 public:
  int Init(const int channel_num, const int display_mode,
           const smart_vo_cfg_t& smart_vo_cfg, const bool encode_smart = false,
           const bool encode_1080p = false, const bool encode_720p = false,
           const bool display = true);
  int Start();
  int Input(std::shared_ptr<VideoData> video_data,
            const bool encode_720P = false, const bool transition = false);
  int Stop();
  int DeInit();

 private:
  int HandleData();
  int HandleData_720P();

 private:
  bool start_ = false;
  bool init_ = false;
  bool running_ = false;

  int channel_num_ = 0;
  int display_mode_ = 0;
  bool encode_smart_ = false;
  bool display_ = true;
  bool encode_720p_ = false;
  std::shared_ptr<solution::video_box::VotModule> vot_module_;

  std::shared_ptr<VencModule> venc_module_1080p_;
  std::shared_ptr<VencModule> venc_module_720p_;
  smart_vo_cfg_t vo_plot_cfg_;

  horizon::vision::BlockingQueue<std::shared_ptr<VideoData>> in_queue_;
  uint32_t in_queue_len_max_ = 10;
  // std::mutex cache_mtx_;
  std::thread plot_task_;

  horizon::vision::BlockingQueue<std::shared_ptr<VideoData>> in_queue_720p_;
  // std::mutex cache_mtx_720p_;
  std::thread plot_task_720p_;
  // plot task
  // uint32_t plot_task_num_ = 1;
  // std::vector<std::shared_ptr<std::thread>> plot_tasks_;
};

}  // namespace video_box
}  // namespace solution
#endif  // INCLUDE_VIDEOPROCESSOR_H_
