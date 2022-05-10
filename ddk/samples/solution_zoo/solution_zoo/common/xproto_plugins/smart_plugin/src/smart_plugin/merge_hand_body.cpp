/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief:
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#include "smart_plugin/merge_hand_body.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <vector>
#include "hobotlog/hobotlog.hpp"

namespace xproto {
void MergeHandBody::UpdateHandTrackID(xstream::OutputDataPtr xstream_out) {
  xstream::BaseDataVector *kps_list = nullptr;
  xstream::BaseDataVector *hand_lmk_list = nullptr;
  xstream::BaseDataVector *body_box_list = nullptr;
  xstream::BaseDataVector *hand_box_list = nullptr;
  xstream::BaseDataVector *gesture_list = nullptr;
  xstream::BaseDataVector *face_boxe_list = nullptr;
  auto name_prefix = [](const std::string name,
                        const char separator) -> std::string {
    auto pos = name.find(separator);
    if (pos == std::string::npos) return "";

    return name.substr(0, pos);
  };
  auto name_postfix = [](const std::string name,
                         const char separator) -> std::string {
    auto pos = name.rfind(separator);
    if (pos == std::string::npos) return "";

    return name.substr(pos + 1);
  };
  for (const auto &output : xstream_out->datas_) {
    auto real_output_name = name_prefix(output->name_, '|');
    LOGD << output->name_ << ", type is " << output->type_ << "real name is "
         << real_output_name;
    if (real_output_name.empty()) {
      real_output_name = output->name_;
    }
    auto prefix = name_prefix(real_output_name, '_');
    auto postfix = name_postfix(real_output_name, '_');
    if (real_output_name == "kps" ||
        real_output_name == "lowpassfilter_body_kps") {
      kps_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }

    if (real_output_name == "hand_lmk" ||
        real_output_name == "lowpassfilter_hand_lmk") {
      hand_lmk_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }
    if (real_output_name == "body_box" || prefix == "body") {
      body_box_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }
    if (prefix == "hand" && postfix == "box") {
      hand_box_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }

    if (real_output_name == "gesture_vote") {
      gesture_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }
    if (real_output_name == "face_bbox_list") {
      face_boxe_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    }
  }

  UpdateHandTrackID(body_box_list, kps_list, hand_box_list, hand_lmk_list,
                    face_boxe_list);
  FilterGesture(body_box_list, kps_list, hand_box_list, hand_lmk_list,
                gesture_list);
}

float Float_Min(float x, float y) {
  if (x > y && fabs(x - y) > 1e-6) {
    return y;
  } else {
    return x;
  }
}

float CrossProduct(const xstream::Point a, const xstream::Point b,
                   xstream::Point c) {
  float ab = sqrt((a.x_ - b.x_) * (a.x_ - b.x_) +
                  (a.y_ - b.y_) * (a.y_ - b.y_));
  float ac = sqrt((a.x_ - c.x_) * (a.x_ - c.x_) +
                  (a.y_ - c.y_) * (a.y_ - c.y_));
  float bc = sqrt((b.x_ - c.x_) * (b.x_ - c.x_) +
                  (b.y_ - c.y_) * (b.y_ - c.y_));
  float p = (ab + ac + bc) / 2;
  float s = sqrt(p * (p - ab) * (p - ac) * (p - bc));
  return s / ab;
}

bool MergeHandBody::UpdateHandID(
    xstream::BaseDataVector *face_boxes_list,
    std::shared_ptr<xstream::Landmarks> hand_lmk,
    int32_t &new_track_id) {
  int32_t old_track_id = new_track_id;
  const auto &point = hand_lmk->values_[0];
  for (size_t i = 0; i < face_boxes_list->datas_.size(); ++i) {
    auto face_box =
        std::static_pointer_cast<xstream::BBox>(
            face_boxes_list->datas_[i]);
    if (!face_box) continue;

    if (face_box->score_ <= 0.0) {
      continue;
    }

    if (point.x_ >= face_box->x1_ && point.x_ <= face_box->x2_ &&
        point.y_ >= face_box->y1_ && point.y_ <= face_box->y2_) {
      new_track_id = face_box->id_;
      LOGD << "update hand id hand key point in face box, old "
              "trackid:"
           << old_track_id << ", new trackid:" << new_track_id;
      return true;
    }
  }
  LOGD << "update hand id hand key point not in face box, old "
          "trackid:"
       << old_track_id << ", new trackid:" << new_track_id;
  return false;
}

bool MergeHandBody::IsHandOnOneLine(
    std::shared_ptr<xstream::Landmarks> hand_lmk,
    std::shared_ptr<xstream::Landmarks> lmk,
    int32_t body_trackid, const bool left_hand) {
  auto &point1 = lmk->values_[7];
  auto &point2 = lmk->values_[9];
  auto &point3 = lmk->values_[5];
  if (!left_hand) {
    point1 = lmk->values_[8];
    point2 = lmk->values_[10];
    point3 = lmk->values_[6];
  }

  const auto &hand_point = hand_lmk->values_[0];
  const auto &hand_point1 = hand_lmk->values_[9];
  constexpr float eps = 50.0;  // 1e-8;
  if (point1.score_ > 0) {
    float distance = CrossProduct(point1, point2, hand_point);
    if (distance > eps && fabs(distance - eps) > 1e-1) {
      return false;
    }
    LOGW << "update hand id body trackid:" << body_trackid
         << ", arm key point on line, distance:"
         << distance << ", hand key point x:"
         << hand_point.x_ << ", y:" << hand_point.y_
         << ", arm point x:" << point1.x_
         << ", y:" << point1.y_ << ",   x:" << point2.x_
         << ", y:" << point2.y_ << "  left hand flag:" << left_hand;
    distance = CrossProduct(point3, hand_point1, hand_point);
    if (distance > eps && fabs(distance - eps) > 1e-1) {
      return false;
    }

    LOGW << "update hand id body trackid:" << body_trackid
         << ", arm key point and hand "
            "point10, key point on line, "
            "distance:"
         << distance << ", arm point x:" << point3.x_ << ", y:" << point3.y_
         << ",  hand 10 x:" << hand_point1.x_ << ", y:" << hand_point1.y_
         << ", hand 1 x:" << hand_point.x_ << ", y:" << hand_point.y_
         << "  left hand flag:" << left_hand;
    distance = sqrt((point1.x_ - point2.x_) * (point1.x_ - point2.x_) +
                    (point1.y_ - point2.y_) * (point1.y_ - point3.y_));
    float tmp = sqrt((point1.x_ - hand_point.x_) *
                     (point1.x_ - hand_point.x_) +
                     (point1.y_ - hand_point.y_) *
                     (point1.y_ - hand_point.y_));
    if (tmp > distance && (tmp - distance) > 1e-3) {
      return true;  // The hand is in a straight state, and key points are in
                    // order
    }
  }  // end of point1 score > 0
  return false;
}

bool MergeHandBody::UpdateDistance(
    std::shared_ptr<xstream::Landmarks> hand_lmk,
    std::shared_ptr<xstream::Landmarks> kps,
    std::shared_ptr<xstream::BBox> body_box,
    float &min_distance, int32_t &new_track_id) {
  if (hand_lmk->values_.size() <= 0) {
    LOGE << "MergeHandBody recv hand lmk point size is 0";
    return false;
  }

  const auto &point9 = kps->values_[9];
  const auto &point10 = kps->values_[10];
  if (point9.score_ < 0 && point10.score_ < 0) {
    LOGW << "update hand id find key point score less than 0, return";
    return false;
  }

  bool update = false;
  const auto &point = hand_lmk->values_[0];
  float distance_9 = sqrt((point9.x_ - point.x_) * (point9.x_ - point.x_) +
                          (point9.y_ - point.y_) * (point9.y_- point.y_));
  float distance_10 = sqrt((point10.x_ - point.x_) * (point10.x_ - point.x_) +
                           (point10.y_ - point.y_) * (point10.y_ - point.y_));
  float tmp_distance = Float_Min(distance_9, distance_10);
  if (min_distance > tmp_distance &&
      fabs(tmp_distance - min_distance) > 1e-6) {
    min_distance = tmp_distance;
    new_track_id = body_box->id_;
    if (tmp_distance <= 100.0) update = true;
  }

  // if(point9.score > 0) {
  if (distance_9 < distance_10 && fabs(distance_9 - distance_10) < 1e-3) {
    if (IsHandOnOneLine(hand_lmk, kps, body_box->id_)) {  // on one line
      tmp_distance =
          CrossProduct(kps->values_[5], kps->values_[7], point9);
      if (tmp_distance < 50.0 &&
          fabs(tmp_distance - 50.0) > 1e-1) {  // one line
        if (CrossProduct(kps->values_[7],
           point9, kps->values_[11]) < 100.0)
          return true;
      }

      if (kps->values_[5].x_ <= kps->values_[7].x_ &&
          fabs(kps->values_[5].x_ - kps->values_[7].x_) < 1e-1) {
        // if (point9.x < point.x && fabs(point9.x - point.x) < 1e-1) {
        if (point9.x_ < hand_lmk->values_[9].x_ && update) {
          return true;
        }
        return false;
      } else {
        // if (point9.x > point.x && fabs(point9.x - point.x) < 1e-1) {
        if (point9.x_ > hand_lmk->values_[9].x_ && update) {
          return true;
        }
        return false;
      }
    } else {  // not on one line
      // todo
    }
  } else if (distance_10 < distance_9 &&
             fabs(distance_9 - distance_10) < 1e-3) {
    if (IsHandOnOneLine(hand_lmk, kps, body_box->id_, false)) {
      // on one line
      tmp_distance =
          CrossProduct(kps->values_[6], kps->values_[8], point10);
      if (tmp_distance < 50.0 &&
          fabs(tmp_distance - 50.0) > 1e-1) {  // one line
        if (CrossProduct(kps->values_[8],
           point10, kps->values_[12]) < 100.0)
          return true;
      }

      if (kps->values_[6].x_ <= kps->values_[8].x_ &&
          fabs(kps->values_[6].x_ - kps->values_[8].x_) < 1e-1) {
        // if (point10.x < point.x && fabs(point10.x - point.x) < 1e-1) {
        if (point10.x_ < hand_lmk->values_[9].x_ && update) {
          return true;
        }
        return false;
      } else {
        // if (point10.x > point.x && fabs(point10.x - point.x) < 1e-1) {
        if (point10.x_ > hand_lmk->values_[9].x_ && update) {
          return true;
        }
        return false;
      }
    } else {  // not on one line
      // todo
    }
  } else {
  }

  return false;
}

bool MergeHandBody::UpdateHandID(
    xstream::BaseDataVector *body_list, xstream::BaseDataVector *kps_list,
    std::shared_ptr<xstream::Landmarks> hand_lmk,
    int32_t &new_track_id) {
  int32_t old_track_id = new_track_id;
  float min_distance = 10000.0;
  for (size_t j = 0; j < kps_list->datas_.size(); ++j) {
    auto lmk = std::static_pointer_cast<
        xstream::Landmarks>(kps_list->datas_[j]);
    if (!lmk) continue;

    if (lmk->values_.size() < 17) {
      LOGE << "UpdateHandTrackID body kps size less than 17, error!!!";
      continue;
    }
    auto body_box =
        std::static_pointer_cast<xstream::BBox>(
            body_list->datas_[j]);
    if (!body_box) continue;
    if (body_box->id_ == -1) {
      continue;
    }

    bool same_hand = false;
    same_hand =
        UpdateDistance(hand_lmk, lmk, body_box, min_distance, new_track_id);
    if (same_hand) {
      new_track_id = body_box->id_;
      return true;
    }
  }  // end of for

  if (min_distance > 500.0) {
    LOGI << "update hand id find hand track:" << old_track_id
         << " the min key point distance more than 500, distance:"
         << min_distance << ",return false ";
    return false;
  }
  return true;
}

void MergeHandBody::UpdateHandTrackID(
    xstream::BaseDataVector *body_list,
    xstream::BaseDataVector *kps_list,
    xstream::BaseDataVector *hand_box_list,
    xstream::BaseDataVector *hand_lmk_list,
    xstream::BaseDataVector *face_boxe_list) {
  if (body_list == NULL || kps_list == NULL || hand_box_list == NULL ||
      hand_lmk_list == NULL) {
    LOGD << "UpdateHandTrackID recv null pointer";
    return;
  }

  if (hand_lmk_list->datas_.size() != hand_box_list->datas_.size()) {
    LOGE << "UpdateHandTrackID recv hand lmk list size != hand box list size";
    return;
  }
  if (body_list->datas_.size() != kps_list->datas_.size()) {
    LOGE << "UpdateHandTrackID recv body box list size != kps list size";
    return;
  }

  for (size_t i = 0; i < hand_lmk_list->datas_.size(); ++i) {
    auto hand_lmk = std::static_pointer_cast<xstream::Landmarks>(
        hand_lmk_list->datas_[i]);
    if (!hand_lmk) continue;

    // 查找对应的track_id
    auto hand_box =
        std::static_pointer_cast<xstream::BBox>(
            hand_box_list->datas_[i]);
    if (!hand_box || hand_box->id_ == -1) {
      continue;
    }

    int32_t new_track_id = hand_box->id_;
    if (UpdateHandID(body_list, kps_list, hand_lmk, new_track_id)) {
      LOGD << "update hand id to body id, old track:" << hand_box->id_
           << " new track:" << new_track_id;
      hand_box->id_ = new_track_id;
    } else {
      if (face_boxe_list) {
        if (UpdateHandID(face_boxe_list, hand_lmk, new_track_id)) {
          LOGD << "update hand id to face id, old track:" << hand_box->id_
               << " new track:" << new_track_id;
          hand_box->id_ = new_track_id;
        }
      }
    }  // end of else
  }    // end of for
}

void MergeHandBody::FilterGesture(xstream::BaseDataVector *body_list,
                                  xstream::BaseDataVector *kps_list,
                                  xstream::BaseDataVector *hand_box_list,
                                  xstream::BaseDataVector *hand_lmk_list,
                                  xstream::BaseDataVector *gesture_list) {
  if (body_list == NULL || kps_list == NULL || hand_box_list == NULL ||
      hand_lmk_list == NULL || gesture_list == NULL) {
    LOGD << "FilterGesture recv null pointer";
    return;
  }
  if (hand_lmk_list->datas_.size() != hand_box_list->datas_.size()) {
    LOGE << "FilterGesture recv hand lmk list size != hand box list "
            "size";
    return;
  }

  if (gesture_list->datas_.size() != hand_box_list->datas_.size()) {
    LOGE << "FilterGesture recv gesture list size != hand box list "
            "size";
    return;
  }

  if (body_list->datas_.size() != kps_list->datas_.size()) {
    LOGE << "FilterGesture recv body box list size != kps list size";
    return;
  }

  constexpr float eps = 100.0;
  for (size_t i = 0; i < hand_lmk_list->datas_.size(); ++i) {
    auto hand_lmk = std::static_pointer_cast<xstream::Landmarks>(
        hand_lmk_list->datas_[i]);
    if (!hand_lmk)
      continue;

    // 查找对应的track_id
    auto hand_box =
        std::static_pointer_cast<xstream::BBox>(
            hand_box_list->datas_[i]);
    if (!hand_box)
      continue;
    if (hand_box->id_ == -1) {
      continue;
    }
    const auto &point = hand_lmk->values_[0];
    int32_t track_id = hand_box->id_;
    for (size_t j = 0; j < body_list->datas_.size(); ++j) {
      auto body_box =
          std::static_pointer_cast<xstream::BBox>(
              body_list->datas_[j]);
      if (!body_box) {
        continue;
      }
      if (body_box->id_ != track_id) {
        continue;
      }

      auto lmk = std::static_pointer_cast<
          xstream::Landmarks>(kps_list->datas_[j]);
      if (!lmk)
       break;
      const auto &point11 = lmk->values_[11];
      const auto &point12 = lmk->values_[12];
      if (((point11.y_ < point.y_ + eps) &&
           fabs(point11.y_ - point.y_ - eps) > 1e-6) ||
          ((point12.y_ < point.y_ + eps) &&
           fabs(point12.y_ - point.y_ - eps) > 1e-6)) {
        auto gesture_vote = std::static_pointer_cast<
            xstream::Attribute_<int>>(
            gesture_list->datas_[i]);
        if (gesture_vote->state_ == xstream::DataState::VALID) {
          gesture_vote->value_ = 0;
        }
      }
      break;
    }
  }
}

}  // namespace xproto
