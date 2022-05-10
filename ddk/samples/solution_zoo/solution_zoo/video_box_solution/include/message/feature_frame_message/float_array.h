/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Qingpeng Liu
 * @Mail: qingpeng.liu@horizon.ai
 * @Date: 2021-03-15 14:00:22
 * @Version: v0.0.1
 * @Brief: FloatArray declaration
 * @Last Modified by: Qingpeng Liu
 * @Last Modified time: 2021-03-15 15:45:01
 */
#ifndef VIDEO_BOX_INCLUDE_MESSGAGE_\
FEATURE_FRAME_MESSAGE_FEATURE_ARRAY_H_
#define VIDEO_BOX_INCLUDE_MESSGAGE_\
FEATURE_FRAME_MESSAGE_FEATURE_ARRAY_H_

#include <vector>

#include "xstream/vision_type.h"

namespace solution {
namespace video_box {

/**
 * @brief float array : use vision_type.h
 */
using FloatArray = xstream::DataArray_<float>;

}  // namespace video_box
}  // namespace solution

#endif  // VIDEO_BOX_INCLUDE_MESSGAGE_FEATURE_FRAME_MESSAGE_FEATURE_ARRAY_H_
