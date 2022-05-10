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

#include "roi_zoom_plugin/roi_calc.h"

#include "hobotlog/hobotlog.hpp"

namespace xproto {

SmartCalcRoi::SmartCalcRoi() {
  target_roi_.x = 0;
  target_roi_.y = 0;
  target_roi_.width = frame_width_;
  target_roi_.height = frame_height_;
  current_roi_.x = 0;
  current_roi_.y = 0;
  current_roi_.width = frame_width_;
  current_roi_.height = frame_height_;
}

int SmartCalcRoi::UpdateConfig(const RoiCalcConfig &config) {
  if (config.view_roi_update_thr >= 0 && config.view_roi_update_thr <= 1)
    view_roi_update_thr_ = config.view_roi_update_thr;
  if (config.view_expansion_coefficient >= 1 &&
      config.view_expansion_coefficient <= 2)
    view_expansion_coefficient_ = config.view_expansion_coefficient;
  if (config.view_transition_coefficient >= 4 &&
      config.view_transition_coefficient <= 64)
    view_transition_coefficient_ = config.view_transition_coefficient;
  if (config.track_roi_update_thr >= 0 && config.track_roi_update_thr <= 1)
    track_roi_update_thr_ = config.track_roi_update_thr;
  if (config.track_expansion_coefficient >= 1 &&
      config.track_expansion_coefficient <= 2)
    track_expansion_coefficient_ = config.track_expansion_coefficient;
  if (config.track_roi_display >= 1 && config.track_expansion_coefficient <= 3)
    track_roi_display_ = config.track_roi_display;
  keep_ratio_ = config.keep_ratio;
  return 0;
}

HorizonVisionBBox SmartCalcRoi::MaxBox(
    const std::vector<HorizonVisionBBox> &boxs) {
  float x_min(4096), y_min(4096);
  float x_max(0), y_max(0);
  float threshold(0);

  for (auto box : boxs) {
    if (box.score > threshold) {
      if (box.x1 < x_min) x_min = box.x1;
      if (box.y1 < y_min) y_min = box.y1;
      if (box.x2 > x_max) x_max = box.x2;
      if (box.y2 > y_max) y_max = box.y2;
    }
  }

  if (keep_ratio_) {
    int w = x_max - x_min;
    int h = y_max - y_min;

    if (w > h * 16 / 9) {
      int nh = w * 9 / 16;
      y_min -= nh / 2 - h / 2;
      y_max += nh / 2 - h / 2;
    } else {
      int nw = h * 16 / 9;
      x_min -= nw / 2 - w / 2;
      x_max += nw / 2 - w / 2;
    }
  }

  return HorizonVisionBBox{x_min, y_min, x_max, y_max};
}

int SmartCalcRoi::AddHeadBox(const HorizonVisionBBox &box) {
  head_boxs_.push_back(box);
  return 0;
}

int SmartCalcRoi::AddHeadBox(const std::vector<HorizonVisionBBox> &boxs) {
  head_boxs_.insert(head_boxs_.end(), boxs.begin(), boxs.end());
  return 0;
}

RoiInfo SmartCalcRoi::GetViewRoiBox() { return GetViewRoiBox(head_boxs_); }

int SmartCalcRoi::ResetHeadBox() {
  head_boxs_.clear();
  return 0;
}

int SmartCalcRoi::AddHandGestureMuteBox(const HorizonVisionBBox &box) {
  gesture_mute_boxs_.push_back(box);
  return 0;
}

int SmartCalcRoi::AddHandGestureMuteBox(
    const std::vector<HorizonVisionBBox> &boxs) {
  gesture_mute_boxs_.insert(gesture_mute_boxs_.end(), boxs.begin(), boxs.end());
  return 0;
}

std::vector<std::vector<RoiInfo>> SmartCalcRoi::GetTrackRoiBox() {
  return GetTrackRoiBox(gesture_mute_boxs_, head_boxs_);
}

int SmartCalcRoi::ResetHandGestureMuteBox() {
  gesture_mute_boxs_.clear();
  return 0;
}

float SmartCalcRoi::BoxIou(const HorizonVisionBBox head_box,
                           const HorizonVisionBBox hand_box) {
  float l_inter = std::max(head_box.x1, hand_box.x1);
  float r_inter = std::min(head_box.x2, hand_box.x2);
  if (l_inter >= r_inter) {
    return -1;
  }
  float t_inter = std::max(head_box.y1, hand_box.y1);
  float b_inter = std::min(head_box.y2, hand_box.y2);
  if (t_inter >= b_inter) {
    return -1;
  }
  float w_inter = r_inter - l_inter;
  float h_inter = b_inter - t_inter;
  float area_inter = w_inter * h_inter;
  float area_hand_box =
      (hand_box.x2 - hand_box.x1) * (hand_box.y2 - hand_box.y1);
  float iou = area_inter / area_hand_box;
  return iou;
}

float SmartCalcRoi::BoxIou(const RoiInfo boxA, const RoiInfo boxB) {
  float l_inter = std::max(boxA.x, boxB.x);
  float r_inter = std::min(boxA.x + boxA.width, boxB.x + boxB.width);
  if (l_inter >= r_inter) {
    return 0;
  }
  float t_inter = std::max(boxA.y, boxB.y);
  float b_inter = std::min(boxA.y + boxA.height, boxB.y + boxB.height);
  if (t_inter >= b_inter) {
    return 0;
  }
  float w_inter = r_inter - l_inter;
  float h_inter = b_inter - t_inter;
  float area_inter = w_inter * h_inter;
  float area_union =
      boxA.width * boxA.height + boxB.width * boxB.height - area_inter;
  float iou = area_inter / area_union;
  return iou;
}

void SmartCalcRoi::RoiBoxAdaptation(RoiInfo &roi, int min_w, int min_h,
                                    int max_w, int max_h) {
  int roi_x = roi.x;
  int roi_y = roi.y;
  int roi_w = roi.width;
  int roi_h = roi.height;

  if (roi_w < min_w) {
    roi_w = min_w;
    roi_x = roi.x + roi.width / 2 - roi_w / 2;
  }

  if (roi_h < min_h) {
    roi_h = min_h;
    roi_y = roi.y + roi.height / 2 - roi_h / 2;
  }

  if (roi_x < 0) roi_x = 0;
  if (roi_y < 0) roi_y = 0;
  if (roi_x + roi_w > max_w) roi_x = max_w - roi_w;
  if (roi_y + roi_h > max_h) roi_y = max_h - roi_h;

  roi_x &= ~0x1;
  roi_y &= ~0x1;

  roi.x = roi_x;
  roi.y = roi_y;
  roi.width = roi_w;
  roi.height = roi_h;
}

void SmartCalcRoi::RoiBoxExpansion(RoiInfo &roi, float coefficient) {
  int roi_w = roi.width * coefficient;
  int roi_h = roi.height * coefficient;

  if (roi_w > frame_width_ || roi_h > frame_height_) {
    roi.x = 0;
    roi.y = 0;
    roi.width = frame_width_;
    roi.height = frame_height_;
  } else {
    RoiBoxAdaptation(roi, roi_w, roi_h, frame_width_, frame_height_);
  }
}

RoiInfo SmartCalcRoi::GetViewRoiBox(
    const std::vector<HorizonVisionBBox> &boxs) {
  RoiInfo roi{0, 0, frame_width_, frame_height_};
  if (!boxs.empty()) {
    auto box_max = MaxBox(boxs);
    roi.x = box_max.x1;
    roi.y = box_max.y1;
    roi.width = box_max.x2 - box_max.x1;
    roi.height = box_max.y2 - box_max.y1;
    RoiBoxExpansion(roi, view_expansion_coefficient_);
    RoiBoxAdaptation(roi, display_width_, display_height_, frame_width_,
                     frame_height_);
  }

  auto similar = BoxIou(roi, target_roi_);
  // 更新target框
  if (similar < view_roi_update_thr_) {
    target_roi_ = roi;
  }

  similar = BoxIou(target_roi_, current_roi_);
  // 更新roi框
  if (similar < view_roi_update_thr_) {
    current_roi_.x +=
        (target_roi_.x - current_roi_.x) / view_transition_coefficient_;
    current_roi_.y +=
        (target_roi_.y - current_roi_.y) / view_transition_coefficient_;
    current_roi_.width +=
        (target_roi_.width - current_roi_.width) / view_transition_coefficient_;
    current_roi_.height += (target_roi_.height - current_roi_.height) /
                           view_transition_coefficient_;

    RoiBoxAdaptation(current_roi_, display_width_, display_height_,
                     frame_width_, frame_height_);
  }

  return current_roi_;
}

std::vector<RoiInfo> SmartCalcRoi::BoxAdaptationToRoi(
    const HorizonVisionBBox &box) {
  std::vector<RoiInfo> info;
  int box_x = box.x1;
  int box_y = box.y1;
  int box_width = box.x2 - box.x1;
  int box_height = box.y2 - box.y1;

  RoiInfo roi_track{box_x, box_y, box_width, box_height};
  RoiBoxExpansion(roi_track, track_expansion_coefficient_);
  RoiInfo roi_rsz = roi_track;

  if (roi_track.width < display_width_ && roi_track.height < display_height_) {
    RoiBoxAdaptation(roi_rsz, display_width_, display_height_, frame_width_,
                     frame_height_);
    roi_track.x -= roi_rsz.x;
    roi_track.y -= roi_rsz.y;
    if (roi_track.width < upscale_min_width_ ||
        roi_track.height < upscale_min_height_) {
      RoiBoxAdaptation(roi_track, upscale_min_width_, upscale_min_height_,
                       roi_rsz.width, roi_rsz.height);
    }
    roi_track.x &= ~0x1;
    roi_track.y &= ~0x1;
  } else {
    roi_track.x = 0;
    roi_track.y = 0;
    roi_track.width = display_width_;
    roi_track.height = display_height_;
    RoiBoxAdaptation(roi_rsz, display_width_, display_height_, frame_width_,
                     frame_height_);
  }
  info.push_back(roi_track);
  info.push_back(roi_rsz);
  return info;
}

RoiInfo GetRealRoiBox(const std::vector<RoiInfo> &roi) {
  RoiInfo box;

  if (roi.empty()) {
    return box;
  }
  auto track_box = roi[0];

  if (roi.size() > 1) {
    auto rsz_box = roi[1];
    box.x = track_box.x + rsz_box.x;
    box.y = track_box.y + rsz_box.y;
  } else {
    box.x = track_box.x;
    box.y = track_box.y;
  }
  box.width = track_box.width;
  box.height = track_box.height;
  return box;
}

std::vector<std::vector<RoiInfo>> SmartCalcRoi::GetTrackRoiBox(
    const std::vector<HorizonVisionBBox> &hand_boxs,
    const std::vector<HorizonVisionBBox> &body_boxs) {
  if (track_boxs_.empty()) {
    RoiInfo pym_roi{0, 0, display_width_, display_height_};
    RoiInfo full_roi{0, 0, frame_width_, frame_height_};
    std::vector<RoiInfo> full;
    full.push_back(pym_roi);
    full.push_back(full_roi);
    track_boxs_.resize(track_roi_display_, full);
  }

  if (hand_boxs.empty() || body_boxs.empty()) {
    return track_boxs_;
  }

  std::vector<std::vector<RoiInfo>> track_boxs;
  for (auto hand_box : hand_boxs) {
    std::vector<HorizonVisionBBox> boxs;
    if (body_boxs.empty()) {
      boxs.push_back(hand_box);
    } else {
      std::vector<float> score_head;
      for (auto head_box : body_boxs) {
        auto iou_score = BoxIou(head_box, hand_box);
        score_head.push_back(iou_score);
      }

      auto largest = std::max_element(score_head.begin(), score_head.end());
      if (*largest < 0) {
        LOGE << "max score " << *largest;
        continue;
      }
      auto pos = std::distance(std::begin(score_head), largest);
      boxs.push_back(body_boxs[pos]);
      // boxs.push_back(hand_box);
    }
    auto box_max = MaxBox(boxs);
    auto track_box = BoxAdaptationToRoi(box_max);
    track_boxs.push_back(track_box);
  }

  if (!track_boxs.empty()) {
    std::vector<bool> curr_update_flag(track_boxs.size(), false);
    std::vector<bool> track_update_flag(track_boxs_.size(), false);

    for (uint32_t i = 0; i < track_boxs.size(); ++i) {
      for (uint32_t j = 0; j < track_boxs_.size(); ++j) {
        // 计算2个结果的交并比
        auto iou_score =
            BoxIou(GetRealRoiBox(track_boxs[i]), GetRealRoiBox(track_boxs_[j]));
        if (iou_score > track_roi_update_thr_) {
          curr_update_flag.at(i) = true;
          track_update_flag.at(j) = true;
        }
      }
    }

    for (uint32_t i = 0; i < track_update_flag.size(); i++) {
      if (!track_update_flag[i]) {
        for (uint32_t j = 0; j < curr_update_flag.size(); j++) {
          if (!curr_update_flag[j]) {
            track_boxs_[i] = track_boxs[j];
            curr_update_flag[j] = true;
            break;
          }
        }
      }
    }
  }

  return track_boxs_;
}

}  // namespace xproto
