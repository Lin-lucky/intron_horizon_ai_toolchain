/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Qingpeng Liu
 * @Mail: qingpeng.liu@horizon.ai
 * @Date: 2021-03-15 14:00:22
 * @Version: v0.0.1
 * @Brief: FeatureInfo declaration
 * @Last Modified by: Qingpeng Liu
 * @Last Modified time: 2021-03-15 15:45:01
 */

#ifndef VIDEO_BOX_INCLUDE_MESSGAGE_\
FEATURE_FRAME_MESSAGE_FEATURE_INFO_H_
#define VIDEO_BOX_INCLUDE_MESSGAGE_\
FEATURE_FRAME_MESSAGE_FEATURE_INFO_H_

#include <string>
#include <vector>

#include "float_array.h"

namespace solution {
namespace video_box {

struct FeatureInfo {
  std::string type_ = "person";
  int64_t frame_id_;
  int64_t time_stamp_;
  uint32_t track_id_;
  std::vector<FloatArray> float_arrays_;  // 每个人多个抓拍特征
};

}  // namespace video_box
}  // namespace solution

#endif  // VIDEO_BOX_INCLUDE_MESSGAGE_FEATURE_FRAME_MESSAGE_FEATURE_INFO_H_
