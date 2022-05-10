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
#ifndef INCLUDE_ROI_ZOOM_PLUGIN_ROI_CALC_H_
#define INCLUDE_ROI_ZOOM_PLUGIN_ROI_CALC_H_

#include <algorithm>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "roi_zoom_plugin/roi_zoom_common.h"

namespace xproto {

struct RoiCalcConfig {
  // 智能取景框更新阈值
  float view_roi_update_thr = 0.85;
  // 智能取景框外扩系数
  float view_expansion_coefficient = 1.3;
  // 智能取景框过度速度系数
  int view_transition_coefficient = 16;

  // 智能追踪框更新阈值
  float track_roi_update_thr = 0.75;
  // 智能追踪框外扩系数
  float track_expansion_coefficient = 1.3;
  // 智能追踪框显示个数
  uint32_t track_roi_display = 2;
  // 返回框是否保持原图比例
  bool keep_ratio = true;
};

class SmartCalcRoi {
 public:
  SmartCalcRoi();
  ~SmartCalcRoi() = default;

  int UpdateConfig(const RoiCalcConfig &config);

  int AddHeadBox(const HorizonVisionBBox &box);
  int AddHeadBox(const std::vector<HorizonVisionBBox> &boxs);
  RoiInfo GetViewRoiBox();
  int ResetHeadBox();

  int AddHandGestureMuteBox(const HorizonVisionBBox &box);
  int AddHandGestureMuteBox(const std::vector<HorizonVisionBBox> &boxs);
  std::vector<std::vector<RoiInfo>> GetTrackRoiBox();
  int ResetHandGestureMuteBox();

  RoiInfo GetViewRoiBox(const std::vector<HorizonVisionBBox> &boxs);
  std::vector<std::vector<RoiInfo>> GetTrackRoiBox(
      const std::vector<HorizonVisionBBox> &hand_boxs,
      const std::vector<HorizonVisionBBox> &body_boxs);

 private:
  HorizonVisionBBox MaxBox(const std::vector<HorizonVisionBBox> &boxs);
  float BoxIou(const HorizonVisionBBox head_box,
               const HorizonVisionBBox hand_box);
  float BoxIou(const RoiInfo boxA, const RoiInfo boxB);
  void RoiBoxAdaptation(RoiInfo &roi, int min_w, int min_h, int max_w,
                        int max_h);
  void RoiBoxExpansion(RoiInfo &roi, float coefficient);
  std::vector<RoiInfo> BoxAdaptationToRoi(const HorizonVisionBBox &box);

  std::vector<HorizonVisionBBox> head_boxs_;
  std::vector<HorizonVisionBBox> gesture_mute_boxs_;

  RoiInfo target_roi_;
  RoiInfo current_roi_;
  std::vector<HorizonVisionBBox> hand_boxs_;
  std::vector<std::vector<RoiInfo>> track_boxs_;

  // 原图宽
  int frame_width_ = 1920;
  // 原图高
  int frame_height_ = 1080;
  // 显示区域宽
  int display_width_ = 960;
  // 显示区域高
  int display_height_ = 540;
  // 放大区域最小宽
  int upscale_min_width_ = 640;
  // 放大区域最小高
  int upscale_min_height_ = 360;

  // 智能取景框更新阈值
  float view_roi_update_thr_ = 0.85;
  // 智能取景框外扩系数
  float view_expansion_coefficient_ = 1.3;
  // 智能取景框过度速度系数
  int view_transition_coefficient_ = 16;

  // 智能追踪框更新阈值
  float track_roi_update_thr_ = 0.75;
  // 智能追踪框外扩系数
  float track_expansion_coefficient_ = 1.3;
  // 智能追踪框显示个数
  uint32_t track_roi_display_ = 2;

  // 返回框比例是否和原图一致
  bool keep_ratio_ = true;
};

}  // namespace xproto
#endif  // INCLUDE_ROI_ZOOM_PLUGIN_ROI_CALC_H_
