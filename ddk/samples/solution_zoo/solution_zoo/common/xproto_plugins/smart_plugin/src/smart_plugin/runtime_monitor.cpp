/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 03:07:26
 * @Version: v0.0.1
 * @Brief: runtime monitor implementation
 * @Note:  extracted from repo xperson's global_config.cpp
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-22 23:46:17
 */

#include "smart_plugin/runtime_monitor.h"
#include "smart_plugin/utils/time_helper.h"
#include <memory>
#include <mutex>
#include "hobotlog/hobotlog.hpp"

namespace xproto {
using ImageFramePtr = std::shared_ptr<xstream::ImageFrame>;

void RuntimeMonitor::PushFrame(const SmartInput *input) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  HOBOT_CHECK(input) << "Null HorizonVisionImageFrame";
  auto frame_info = input->frame_info;
  HOBOT_CHECK(frame_info->num_ > 0);

  std::shared_ptr<xstream::PyramidImageFrame> image0 = frame_info->image_[0];
  uint64_t frame_id = image0->frame_id_;
  LOGI << "PushFrame frame_id = " << frame_id << std::endl;
  input_frames_[frame_id].image_num = frame_info->num_;
  input_frames_[frame_id].img = frame_info->image_;
  input_frames_[frame_id].context = input->context;
  input_frames_[frame_id].vio_msg = frame_info;
}

RuntimeMonitor::InputFrameData RuntimeMonitor::PopFrame(
    const int32_t &frame_id) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  InputFrameData input;
  auto itr = input_frames_.find(frame_id);
  if (itr != input_frames_.end()) {
    input = itr->second;
    LOGI << "Pop frame " << frame_id;
    input_frames_.erase(itr);
  }
  return input;
}

void RuntimeMonitor::FrameStatistic() {
    // 实际智能帧率计算
  static int fps = 0;
  // 耗时统计，ms
  static auto lastTime = xproto::Timer::tic();
  static int frameCount = 0;

  ++frameCount;

  auto curTime = xproto::Timer::toc(lastTime);
  // 统计数据发送帧率
  if (curTime > 1000) {
    fps = frameCount;
    frameCount = 0;
    lastTime = xproto::Timer::tic();
    LOGW << "Smart fps = " << fps;
    frame_fps_ = fps;
  }
}

RuntimeMonitor::RuntimeMonitor() { Reset(); }

bool RuntimeMonitor::Reset() { return true; }

}  // namespace xproto
