/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_VIDEOBOX_COMMON_H_
#define INCLUDE_VIDEOBOX_COMMON_H_

#include <memory>
#include <unordered_map>

#include "blocking_queue.hpp"
#include "message/common_data_type/recog_result.h"
#include "vision/vision_type.h"
#include "xstream/xstream_data.h"
#include "xstream/vision_type.h"
#include "vision/vision_msg.h"
#include "vision/vision_type_util.h"
#include "smart_message/smart_message.h"

using solution::video_box::RecogResult;

namespace solution {
namespace video_box {
struct VideoData {
  uint32_t channel = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint64_t timestamp = 0;
  char *buffer = nullptr;
  char *y_virtual_addr = nullptr;
  char *uv_virtual_addr = nullptr;
  int data_len = 0;
  bool has_plot = false;
  bool nv12 = false;
  HorizonVisionSmartFrame *smart_frame = nullptr;
  std::unordered_map<uint64_t, std::shared_ptr<RecogResult>> recog_cache;
  ~VideoData() {
    if (smart_frame != nullptr) {
      HorizonVisionFreeSmartFrame(smart_frame);
    }
    if (buffer != nullptr) {
      free(buffer);
    }
  }
};

typedef struct smart_vo_s {
  float box_face_thr = 0.95;
  float box_head_thr = 0.95;
  float box_body_thr = 0.95;
  float lmk_thr = 0.0;
  float kps_thr = 0.50;
  float box_veh_thr = 0.995;
  bool plot_fps = false;
  bool transition_support = false;
} smart_vo_cfg_t;

#define SMART_TYPE_NORMAL 0;
#define SMART_TYPE_FILTERED 1;
#define SMART_TYPE_DISAPPEARED 2;

typedef struct FaceQuality_ {
  /// \~Chinese 人脸清晰度
  xstream::BaseDataVector *blur = nullptr;
  /// \~Chinese 人脸亮度
  xstream::BaseDataVector *brightness = nullptr;
  /// \~Chinese 眼睛表情
  xstream::BaseDataVector *eye_abnormalities = nullptr;
  /// \~Chinese 嘴部
  xstream::BaseDataVector *mouth_abnormal = nullptr;
  /// \~Chinese 左眼
  xstream::BaseDataVector *left_eye = nullptr;
  /// \~Chinese 右眼
  xstream::BaseDataVector *right_eye = nullptr;
  /// \~Chinese 左眉毛
  xstream::BaseDataVector *left_brow = nullptr;
  /// \~Chinese 右眉毛
  xstream::BaseDataVector *right_brow = nullptr;
  /// \~Chinese 额头
  xstream::BaseDataVector *forehead = nullptr;
  /// \~Chinese 左脸颊
  xstream::BaseDataVector *left_cheek = nullptr;
  /// \~Chinese 右脸颊
  xstream::BaseDataVector *right_cheek = nullptr;
  /// \~Chinese 鼻子
  xstream::BaseDataVector *nose = nullptr;
  /// \~Chinese 嘴部
  xstream::BaseDataVector *mouth = nullptr;
  /// \~Chinese 下巴
  xstream::BaseDataVector *jaw = nullptr;
} FaceQuality;

struct SnapshotParam {
  uint32_t output_width = 128;
  uint32_t output_height = 128;
};

}  // namespace video_box
}  // namespace solution
#endif  // INCLUDE_VIDEOBOX_COMMON_H_
