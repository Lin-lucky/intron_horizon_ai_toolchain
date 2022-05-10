/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     snapshot_data_type header
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.16
 * @date      2019.04.22
 */

#ifndef COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
INCLUDE_SNAPSHOT_METHOD_SNAPSHOT_DATA_TYPE_H_
#define COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_\
INCLUDE_SNAPSHOT_METHOD_SNAPSHOT_DATA_TYPE_H_

#include <memory>
#include <string>
#include <vector>

#include "snapshot_method/snapshot_utils.h"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "json/json.h"

namespace xstream {

using xstream::Point;
using xstream::FloatPoints;

/// Wrap other data structures into derived classes of BaseData
template <typename Dtype>
struct XStreamData : public BaseData {
  Dtype value;
  XStreamData() {}
  explicit XStreamData(const Dtype &val) { value = val; }
};

typedef xstream::XStreamData<float> XStreamFloat;
typedef xstream::XStreamData<uint32_t> XStreamUint32;

// typedef xstream::XStreamData<BBox> XStreamBBox;
// typedef std::shared_ptr<XStreamBBox> XStreamBBoxPtr;

struct SnapshotState {
  SnapshotState()
      : id(-1),
        overall_quality(0),
        select_index(-1),
        snap_en(0),
        snap_stop(0),
        snap_repeat(0) {}
  /// \~Chinese 抓拍人脸ID
  int32_t id;
  /// \~Chinese 抓拍人脸总体打分
  float overall_quality;
  /// \~Chinese 抓拍人脸外框
  xstream::BBox box;
  /// \~Chinese 抓拍人脸缓冲替换序号
  int32_t select_index;
  /// \~Chinese 人脸抓拍有效标识
  uint8_t snap_en : 1;
  /// \~Chinese 优选抓拍结束标识
  uint8_t snap_stop : 1;
  /// \~Chinese 重复抓拍开始标识
  uint8_t snap_repeat : 1;
};

typedef std::shared_ptr<SnapshotState> SnapshotStatePtr;


typedef xstream::XStreamData<SnapshotStatePtr> XStreamSnapshotState;
typedef std::shared_ptr<XStreamSnapshotState> XStreamSnapshotStatePtr;

typedef xstream::SnapshotInfo_<BaseDataPtr> SnapshotInfoBaseData;
typedef std::shared_ptr<SnapshotInfoBaseData> SnapshotInfoBaseDataPtr;

typedef std::shared_ptr<BaseDataVector> BaseDataVectorPtr;

struct SelectSnapShotInfo : SnapshotInfoBaseData {
  Point snap_base_point;
  float wide_scale = 0;
  float height_scale = 0;
  FloatPoints PointsToSnap(const FloatPoints &in) override;
};
typedef std::shared_ptr<SelectSnapShotInfo> SelectSnapShotInfoPtr;

#define SET_SNAPSHOT_METHOD_PARAM(json_cfg, type, key)          \
  do {                                                          \
    if (json_cfg.isMember(#key) && json_cfg[#key].is##type()) { \
      key = json_cfg[#key].as##type();                          \
      config_jv[#key] = key;                                    \
    }                                                           \
  } while (0)

struct SnapShotParam : public InputParam {
 public:
  explicit SnapShotParam(const std::string &content)
      : InputParam("SnapShotMethod") {
    is_enable_this_method_ = true;
    is_json_format_ = true;
    unique_name_ = "SnapShotMethod";
  }

  virtual int UpdateParameter(const std::string &content);

  float scale_rate = 0;
  bool need_resize = true;
  unsigned output_width = 0;
  unsigned output_height = 0;
  bool snapshot_state_enable = false;
  bool save_original_image_frame = true;
  std::string snapshot_type = "select";
  Json::Value config_jv;
  std::string Format() override;
};

class SnapShotInfo {
 public:
  static SelectSnapShotInfoPtr GetSnapShotInfo(
      const ImageFramePtr &frame, const float &select_score,
      const xstream::BBoxPtr &pbbox, SnapShotParam *param,
      std::vector<BaseDataPtr> userdatas);

  static BaseDataVectorPtr GenerateSnapshotInfo(
      const std::vector<SelectSnapShotInfoPtr> &snap_infos,
      const int32_t &type);

  static BaseDataPtr GenerateWithoutSnapshot(const int32_t id);
};

}  // namespace xstream

#endif  // COMMON_XSTREAM_METHODS_SNAPSHOT_METHOD_INCLUDE_SNAPSHOT_METHOD_SNAPSHOT_DATA_TYPE_H_
