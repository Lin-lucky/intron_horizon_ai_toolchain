/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Qingpeng Liu
 * @Mail: qingpeng.liu@horizon.ai
 * @Date: 2021-03-15 14:00:22
 * @Version: v0.0.1
 * @Brief: FeatureFrameMessage declaration
 * @Last Modified by: Qingpeng Liu
 * @Last Modified time: 2021-03-15 15:45:01
 */
#ifndef VIDEO_BOX_INCLUDE_MESSGAGE_\
FEATURE_FRAME_MESSAGE_FEATURE_FRAME_MESSAGE_H_
#define VIDEO_BOX_INCLUDE_MESSGAGE_\
FEATURE_FRAME_MESSAGE_FEATURE_FRAME_MESSAGE_H_

#include <vector>

#include "feature_info.h"

namespace solution {
namespace video_box {

struct FeatureFrameMessage {
  //  消息错误码，指明此帧数据状态
  //  0：正常数据，1：capture drop数据，2:feature drop数据
  uint32_t error_code_;
  int ch_id_;
  std::vector<FeatureInfo> feature_infos_;  // 每帧多个人
};
}  // namespace video_box
}  // namespace solution

#endif  // VIDEO_BOX_INCLUDE_MESSGAGE_\
FEATURE_FRAME_MESSAGE_FEATURE_FRAME_MESSAGE_H_
