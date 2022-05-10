/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_BOX_INCLUDE_MESSAGE_COMMON_DATA_TYPE_RECOG_RESULT_H_
#define VIDEO_BOX_INCLUDE_MESSAGE_COMMON_DATA_TYPE_RECOG_RESULT_H_
#include <memory>
#include <string>
#include <vector>

#include "xstream/vision_type.h"

namespace solution {
namespace video_box {

struct RecogResult {
  uint64_t ch_id;
  uint64_t track_id;
  uint32_t is_recognize;
  float similar;
  std::string record_id;
  std::string img_uri_list;
};

}  // namespace video_box
}  // namespace solution

#endif  // VIDEO_BOX_INCLUDE_MESSAGE_COMMON_DATA_TYPE_RECOG_RESULT_H_
