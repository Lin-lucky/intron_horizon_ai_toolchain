/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "video_processor.h"

#include "hobotlog/hobotlog.hpp"
#include "plot_smart_data.h"

namespace solution {
namespace video_box {

VideoProcessor::VideoProcessor() {
  memset(&vo_plot_cfg_, 0, sizeof(smart_vo_cfg_t));
}

VideoProcessor::~VideoProcessor() {
  Stop();
  DeInit();
}

int VideoProcessor::Init(const int channel_num, const int display_mode,
                         const smart_vo_cfg_t& smart_vo_cfg,
                         const bool encode_smart, const bool encode_1080p,
                         const bool encode_720p, const bool display) {
  if (init_) return 0;
  channel_num_ = channel_num;
  display_mode_ = display_mode;
  encode_smart_ = encode_smart;
  display_ = display;
  encode_720p_ = encode_720p;
  memcpy(&vo_plot_cfg_, &smart_vo_cfg, sizeof(smart_vo_cfg_t));

  if (display) {
    vot_module_ = std::make_shared<VotModule>();
    vot_module_->SetDisplayMode(display_mode_);
    vot_module_->SetChannelNum(channel_num_);
    vot_module_->Init(0, vo_plot_cfg_);
  }
  if (encode_1080p) {
    venc_module_1080p_ = std::make_shared<VencModule>();
    VencModuleInfo module_info_venc;
    module_info_venc.width = 1920;
    module_info_venc.height = 1080;
    module_info_venc.type = 1;
    module_info_venc.bits = 2000;
    venc_module_1080p_->Init(0, &module_info_venc, channel_num_, display_mode_);
  }

  if (encode_720p_) {
    venc_module_720p_ = std::make_shared<VencModule>();
    VencModuleInfo module_info_venc;
    module_info_venc.width = 1280;
    module_info_venc.height = 720;
    module_info_venc.type = 1;
    module_info_venc.bits = 2000;
    venc_module_720p_->Init(1, &module_info_venc, channel_num_, display_mode_);
  }

  init_ = true;
  return 0;
}

int VideoProcessor::Start() {
  if (start_) return 0;

  if (display_) {
    vot_module_->Start();
  }

  if (venc_module_1080p_) {
    venc_module_1080p_->Start();
  }

  if (encode_720p_) {
    venc_module_720p_->Start();
  }

  running_ = true;
  plot_task_ = std::thread(&VideoProcessor::HandleData, this);
  plot_task_720p_ = std::thread(&VideoProcessor::HandleData_720P, this);
  start_ = true;
  return 0;
}

int VideoProcessor::Input(std::shared_ptr<VideoData> video_data,
                          const bool encode_720P, const bool transition) {
  if (!start_) return 0;
  if (transition) {
    if (vot_module_) vot_module_->Input(video_data, true);
    return 0;
  }

  if (video_data->buffer == NULL) {
    return 0;
  }

  if (!encode_720P) {
    {
      // std::lock_guard<std::mutex> lk(cache_mtx_);
      if (in_queue_.size() >= in_queue_len_max_) {
        in_queue_.pop();
        LOGE << "vodeo processor queue is full";
      }
      // in_queue_.push(std::move(video_data));
      in_queue_.push(video_data);
    }
  } else {
    {
      // std::lock_guard<std::mutex> lk(cache_mtx_720p_);
      if (in_queue_720p_.size() >= in_queue_len_max_) {
        in_queue_720p_.pop();
        LOGE << "vodeo processor queue is full";
      }
      in_queue_720p_.push(video_data);
    }
  }
  return 0;
}

int VideoProcessor::Stop() {
  if (!start_) return 0;
  running_ = false;
  start_ = false;
  if (plot_task_.joinable()) {
    plot_task_.join();
  }
  if (plot_task_720p_.joinable()) {
    plot_task_720p_.join();
  }

  if (vot_module_) {
    vot_module_->Stop();
  }
  if (venc_module_1080p_) {
    venc_module_1080p_->Stop();
  }
  if (venc_module_720p_) {
    venc_module_720p_->Stop();
  }

  in_queue_.clear();
  in_queue_720p_.clear();

  return 0;
}

int VideoProcessor::DeInit() {
  if (!init_) {
    return 0;
  }

  if (vot_module_) {
    vot_module_->DeInit();
  }
  if (venc_module_1080p_) {
    venc_module_1080p_->DeInit();
  }
  if (venc_module_720p_) {
    venc_module_720p_->DeInit();
  }

  init_ = false;
  return 0;
}

int VideoProcessor::HandleData() {
  while (this->running_) {
    std::shared_ptr<VideoData> vot_data;
    {
      // std::lock_guard<std::mutex> lk(cache_mtx_);
      auto is_getitem =
          in_queue_.try_pop(&vot_data, std::chrono::microseconds(100));
      if (!is_getitem) {
        continue;
      }
    }

    if (encode_smart_ && vot_data->smart_frame) {
      PlotSmartData objplot;
      objplot.PlotData(vo_plot_cfg_, vot_data);
      vot_data->has_plot = true;
    }

    if (venc_module_1080p_) venc_module_1080p_->Input(vot_data, true);
    if (vot_module_ && !vo_plot_cfg_.transition_support)
      vot_module_->Input(vot_data);
  }
  return 0;
}

int VideoProcessor::HandleData_720P() {
  while (this->running_) {
    std::shared_ptr<VideoData> vot_data;
    {
      // std::lock_guard<std::mutex> lk(cache_mtx_720p_);
      auto is_getitem =
          in_queue_720p_.try_pop(&vot_data, std::chrono::microseconds(100));
      if (!is_getitem) {
        continue;
      }
    }

    if (encode_smart_) {
      PlotSmartData objplot;
      objplot.PlotData(vo_plot_cfg_, vot_data);
      vot_data->has_plot = true;
    }

    venc_module_720p_->Input(vot_data);
  }
  return 0;
}

}  // namespace video_box
}  // namespace solution
