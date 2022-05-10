/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_BOX_INCLUDE_MESSAGE_COMMON_DATA_TYPE_USER_DATA_H_
#define VIDEO_BOX_INCLUDE_MESSAGE_COMMON_DATA_TYPE_USER_DATA_H_
#include <memory>
#include <string>
#include <vector>

#include "xstream/vision_type.h"

namespace solution {
namespace video_box {

/**
 * @brief timestamp, freq id
 */
struct UserData {
  int64_t frame_id_;
  int64_t timestamp_;
  xstream::BBox box_;
};

}  // namespace video_box
}  // namespace solution

#endif  // VIDEO_BOX_INCLUDE_MESSAGE_COMMON_DATA_TYPE_USER_DATA_H_
