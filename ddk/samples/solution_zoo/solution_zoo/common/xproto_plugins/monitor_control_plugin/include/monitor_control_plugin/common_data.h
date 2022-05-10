/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Fei cheng
 * @Mail: fei.cheng@horizon.ai
 * @Date: 2019-09-14 20:38:52
 * @Version: v0.0.1
 * @Brief: common data on xpp.
 * @Last Modified by: Fei cheng
 * @Last Modified time: 2019-09-14 22:41:30
 */

#ifndef INCLUDE_COMMON_DATA_H_
#define INCLUDE_COMMON_DATA_H_
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "transport_message/monitor_control_message.h"

namespace xproto {
#define SYS_VER "system_version"
#define APP_VER "app_version"
#define MD_VER "model_version"

/// \~Chinese X2Solution 专属错误码，预留200
#ifndef kHorizonVisionX2SolutionOffset
#define kHorizonVisionX2SolutionOffset 1200
#endif

using xproto::message::BaseConfigData;

enum X2SolutionErrorCode {
  kHorizonX2SolutionJsonPath = -kHorizonVisionX2SolutionOffset - 1,
  kHorizonX2SolutionJsonParse = -kHorizonVisionX2SolutionOffset - 2,
  kHorizonX2SolutionJsonContent = -kHorizonVisionX2SolutionOffset - 3,
  kHorizonX2SolutionJsonEnd = -kHorizonVisionX2SolutionOffset - 10,
  kHorizonX2SolutionHbipcInit = -kHorizonVisionX2SolutionOffset - 11,
  kHorizonX2SolutionHbipcUninit = -kHorizonVisionX2SolutionOffset - 12,
  kHorizonX2SolutionHbipcConnect = -kHorizonVisionX2SolutionOffset - 13,
  kHorizonX2SolutionHbipcUnconnect = -kHorizonVisionX2SolutionOffset - 14,
  kHorizonX2SolutionHbipcSend = -kHorizonVisionX2SolutionOffset - 15,
  kHorizonX2SolutionHbipcRecv = -kHorizonVisionX2SolutionOffset - 16,
  kHorizonX2SolutionHbipcEnd = -kHorizonVisionX2SolutionOffset - 20,
  kHorizonX2SolutionAdapterInit = -kHorizonVisionX2SolutionOffset - 21,
  kHorizonX2SolutionAdapterUinit = -kHorizonVisionX2SolutionOffset - 22,
  kHorizonX2SolutionAdapterEnd = -kHorizonVisionX2SolutionOffset - 30,
  kHorizonX2SolutionVioInput = -kHorizonVisionX2SolutionOffset - 31,
  kHorizonX2SolutionVioOutput = -kHorizonVisionX2SolutionOffset - 32,
  kHorizonX2SolutionVioStart = -kHorizonVisionX2SolutionOffset - 33,
  kHorizonX2SolutionVioStop = -kHorizonVisionX2SolutionOffset - 34,
  kHorizonX2SolutionAlreadyStart = -kHorizonVisionX2SolutionOffset - 35,
  kHorizonX2SolutionAlreadyStop = -kHorizonVisionX2SolutionOffset - 36
};

#define VIDEO_WIDTH 1920.0
#define VIDEO_HEIGHT 1080.0

static std::map<int, int> HobotLogMap = { };

/* smarter config base data structure */
class SmarterBaseData {
 public:
  SmarterBaseData() = default;
  virtual ~SmarterBaseData() = default;
  std::string name_;
};

/* model capability data structure */
class CPVersion : public SmarterBaseData {
 public:
  CPVersion() { name_ = "CPVersion"; }
  ~CPVersion() = default;

  std::string system_version;
  std::string app_version;
  std::string model_version;
};

/* CP status data structure */
class CPStatus : public SmarterBaseData {
 public:
  explicit CPStatus(int type, int value) { status_[type] = value; }
  ~CPStatus() = default;

  std::map<int, int> status_;
};

const std::list<std::string> ModelCapabilityList = {
    "face_det", "head_det", "body_det", "kps", "reid",
    "mask", "age", "sex", "s3d_pose", "lmk",
    "anti_spf", "clarity", "occlusion", "face_feature", "body_snap"};

/* model capability switch */
typedef struct model_cap_config {
  bool face_det;
  bool head_det;
  bool body_det;
  bool kps;
  bool reid;
  bool mask;
  bool age;
  bool sex;
  bool s3d_pose;
  bool lmk;
  bool anti_sfp;
  bool clarity;
  bool occlusion;
  bool face_feature;
} model_cap_config_t;

const std::list<std::string> StrategyList = {
    "face_trk", "head_trk", "body_trk",
    "face_snap", "body_snap", "attr_smooth",
    "skip_cnn", "close_feat_jump", "open_frame_skip",
    "clear_feat_queue", "big_face_mode", "snap_face_sel_mode"};

/* strategy capability switch */
typedef struct strategy_config {
  bool face_trk;
  bool head_trk;
  bool body_trk;
  bool face_snap;
  bool body_snap;
  bool big_face_mode;
  bool snap_queue_clear;
} strategy_config_t;

class CapabilityCfg : public SmarterBaseData {
 public:
  CapabilityCfg() { name_ = "CapabilityCfg"; }
  ~CapabilityCfg() = default;

  std::map<std::string, bool> model_cap_;
  std::map<std::string, bool> stratege_cap_;
};

const std::list<std::string> ZoneList = {"valid_zone", "invalid_zone"};

/* snap rectangle length setting */
typedef struct {
  int left;
  int top;
  int right;
  int bottom;
} box_t;

const std::list<std::string> TrackCfgList = {
    "face_box_score_thr", "head_box_score_thr", "body_box_score_thr",
    "trk_lost_frame_cnt", "frame_period_time", "face_num_limit"};

/* track module config data structure */
class TrackCfg : public SmarterBaseData {
 public:
  TrackCfg() { name_ = "TrackCfg"; }
  ~TrackCfg() = default;

  std::map<std::string, float> track_cfg_;
  std::map<std::string, std::vector<box_t>> det_zone_cfg_;
};

/* vio mode config data structure */
class VIOModeCfg : public SmarterBaseData {
 public:
  VIOModeCfg() { name_ = "VIOModeCfg"; }
  ~VIOModeCfg() = default;

  std::map<std::string, std::string> vio_mode_cfg_;
};

const std::list<std::string> SnapCfgParamList = {
    "min_size_thr", "frontal_thr", "stop_frame_cnt", "resnap_frame_cnt",
    "slot_cnt", "scale", "bound_thr_w", "bound_thr_h",
    "pose_step", "pic_format", "crop_img_en", "crop_img_max_len"};

/* snapshot module config data structure */
class SnapshotCfg : public SmarterBaseData {
 public:
  SnapshotCfg() { name_ = "SnapshotCfg"; }
  ~SnapshotCfg() = default;

  std::map<std::string, float> snap_param_cfg_;
  std::map<std::string, std::vector<box_t>> snap_zone_cfg_;
};

const std::list<std::string> BodySnapCfgParamList = {
    "det_thres", "det_width_thres", "det_height_thres",
    "kps_cond1_up_body_thres", "kps_cond1_low_body_thres",
    "kps_cond2_head_thres", "kps_cond2_up_body",
    "kps_cond3_all_thres", "kps_cond4_body_thres",
    "size_min", "size_max", "size_inflexion", "size_weight",
    "oritation_weight", "blur_weight", "kps_weight",
    "bound_w_thres", "bound_h_thres", "snap_min_score",
    "delete_frame_interval", "post_frame_thres", "min_tracklet_len"};

class BodySnapCfg : public SmarterBaseData {
 public:
  BodySnapCfg() { name_ = "BodysnapCfg"; }
  ~BodySnapCfg() = default;

  std::map<std::string, float> body_snap_cfg_;
};

const std::list<std::string> FeatureCfgList =
    {"feature_queue_max_len"};

/* snapshot module config data structure */
class FeatureCfg : public SmarterBaseData {
 public:
  FeatureCfg() { name_ = "FeatureCfg"; }
  ~FeatureCfg() = default;

  std::map<std::string, std::vector<uint32_t>> feature_action_;
  std::map<std::string, float> feature_cfg_;
};

const std::list<std::string> FrameCfgList = {
    "frame_fps", "det_jump_frame", "attr_jump_frame", "feature_jump_frame"};

/* snapshot module config data structure */
class FrameCfg : public SmarterBaseData {
 public:
  FrameCfg() { name_ = "FrameCfg"; }
  ~FrameCfg() = default;

  std::map<std::string, float> frame_cfg_;
};

/* image frame data structure */
class FrameImage : public SmarterBaseData {
 public:
  FrameImage() { name_ = "FrameImage"; }
  ~FrameImage() = default;

  std::string buf_;
  std::string type_;
  uint32_t width_;
  uint32_t height_;
};

/* smarter plugin config data structure */
using SmartBasePtr = std::shared_ptr<SmarterBaseData>;
class SmarterConfigData : public BaseConfigData {
 public:
  SmarterConfigData() = default;
  explicit SmarterConfigData(bool status) : status_(status) {}
  explicit SmarterConfigData(std::string name, std::string type)
      : BaseConfigData(name, type) {}
  ~SmarterConfigData() = default;

  std::string Serialize() { return "no need serialize"; }
  std::vector<SmartBasePtr> data_;
  bool status_;
};

}  // namespace xproto
#endif  // INCLUDE_COMMON_DATA_H_
