/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief: roi_zoom_plugin impl based on xpp.
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#ifndef INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_COMMON_H_
#define INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_COMMON_H_
#include <future>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "xstream/xstream_world.h"
#include "utils/roi_vision_type_util.h"

namespace xproto {

struct RoiInfo {
  int32_t x;
  int32_t y;
  int32_t width;
  int32_t height;
};

struct VideoRoiData {
  uint32_t channel = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint64_t timestamp = 0;
  uint64_t frame_id = 0;
  bool roi_data = false;
  char *buffer = nullptr;
  int data_len = 0;
  char *y_virtual_addr = nullptr;
  char *uv_virtual_addr = nullptr;
  bool drop_frame = false;
  HorizonVisionSmartFrame *smart_frame = nullptr;
  ~VideoRoiData() {
    if (smart_frame != nullptr) {
      HorizonVisionFreeSmartFrame(smart_frame);
    }
    if (buffer != nullptr) {
      free(buffer);
    }
    if (y_virtual_addr != nullptr) {
      free(y_virtual_addr);
    }
    if (uv_virtual_addr != nullptr) {
      free(uv_virtual_addr);
    }
  }
};

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

}  // namespace xproto
#endif  // INCLUDE_ROI_ZOOM_PLUGIN_ROI_ZOOM_COMMON_H_
