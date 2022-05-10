/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 02:31:30
 * @Version: v0.0.1
 * @Brief: smart runtime monitor
 * @Note: simplify code from repo xperson global_config.h
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 05:03:14
 */

#ifndef INCLUDE_SMARTPLUGIN_RUNTIME_MONITOR_H_
#define INCLUDE_SMARTPLUGIN_RUNTIME_MONITOR_H_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "vision/vision_type.hpp"
#include "xproto/msg_type/vio_message.h"
#include "xstream/xstream_world.h"

namespace solution {
namespace video_box {
using xproto::message::VioMessage;
using Time_Point = std::chrono::time_point<std::chrono::system_clock>;

struct SmartInput {
  std::shared_ptr<VioMessage> frame_info;
  void *context;
};

class RuntimeMonitor {
 public:
  RuntimeMonitor();

  struct InputFrameData {
    uint32_t image_num;
    std::vector<std::shared_ptr<xstream::PyramidImageFrame>> img;
    void *context = nullptr;
  };

  bool Reset();

  void PushFrame(const SmartInput *input);

  InputFrameData PopFrame(const uint64_t &frame_id, int channel_id = -1);

  uint64_t GetFrontFrame(uint8_t channel_id);

  void FrameStatistic(int channel);
  // void OnXStreamCallback(xstream::OutputDataPtr xstream_output);

 private:
  std::unordered_map<int32_t, InputFrameData> input_frames_;
  std::mutex map_mutex_;
  std::vector<std::shared_ptr<Time_Point>> TP;
  const int channel_ = 9;
  const uint8_t channel_frame_queue_limit_ = 50;
  std::vector<std::map<uint64_t, bool>> channel_frame_id_;
};

}  // namespace video_box
}  // namespace solution

#endif  //  INCLUDE_SMARTPLUGIN_RUNTIME_MONITOR_H_
