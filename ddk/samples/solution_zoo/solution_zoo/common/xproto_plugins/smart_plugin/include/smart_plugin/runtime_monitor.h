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

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "xproto/msg_type/vio_message.h"
#include "xstream/xstream_world.h"

namespace xproto {
using xproto::message::VioMessage;

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
    std::shared_ptr<VioMessage> vio_msg;  // origin vio message
    void *context = nullptr;
  };
  bool Reset();

  void PushFrame(const SmartInput *input);

  InputFrameData PopFrame(const int &frame_id);

  void FrameStatistic();
  int GetFrameFps() { return frame_fps_; }
  // void OnXStreamCallback(xstream::OutputDataPtr xstream_output);

 private:
  std::unordered_map<int32_t, InputFrameData> input_frames_;
  std::mutex map_mutex_;
  int frame_fps_ = 0;
};

}  // namespace xproto
#endif  //  INCLUDE_SMARTPLUGIN_RUNTIME_MONITOR_H_
