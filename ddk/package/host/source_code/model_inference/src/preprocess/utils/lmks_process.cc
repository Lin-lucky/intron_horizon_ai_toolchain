/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: lmks_process.cc
 * @Brief: implementation of LmksProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu May 20 2021 07:49:22
 */

#include "model_inference/preprocess/utils/lmks_process.h"

namespace inference {

void FeatureSequenceBuffer::Update(std::shared_ptr<xstream::BBox> box,
                                   std::shared_ptr<xstream::Landmarks> kps,
                                   uint64_t timestamp) {
  while (len_ > max_len_) {
    LOGD << "overflow, removing..., len: " << len_;
    timestamps_.erase(timestamps_.begin());
    feats_->datas_.erase(feats_->datas_.begin());
    boxes_->datas_.erase(boxes_->datas_.begin());
    len_ -= 1;
  }
  feats_->datas_.push_back(kps);
  boxes_->datas_.push_back(box);
  timestamps_.push_back(timestamp);
  len_ += 1;
}

void FeatureSequenceBuffer::GetClipFeatByEnd(
    std::shared_ptr<BaseDataVector> kpses,
    std::shared_ptr<BaseDataVector> boxes, int num, float stride, float margin,
    uint64_t end) {
  kpses->datas_.clear();
  boxes->datas_.clear();
  if (len_ < 1) {
    LOGW << "len_ < 1, return...";
    return;
  }

  std::vector<int> clip_idxs(num, -1);
  for (int frame_idx = 0; frame_idx < num; ++frame_idx) {
    // timestamp for expected candidate
    float curtime = end - frame_idx * stride;
    // time gap
    uint64_t restime = 1e10;
    uint64_t time_diff = 0;
    for (int idx = len_ - 1; idx >= 0; --idx) {
      time_diff = static_cast<uint64_t>(
              abs(static_cast<int>(timestamps_[idx] - curtime)));
      if (time_diff < restime) {
        restime = time_diff;
        if (restime < margin) {
          clip_idxs[num - 1 - frame_idx] = idx;
        }
      } else {
        break;
      }
    }
  }

  int maxValue = clip_idxs[0];
  int minValue = clip_idxs[0];
  for (auto& item : clip_idxs) {
    if (item < minValue) {
      minValue = item;
    }
    if (item > maxValue) {
      maxValue = item;
    }
  }
  if (maxValue >= len_ || minValue < 0) {
    LOGD << "Failed to get clip kps by end, invalid index. "
         << "max_idx: " << maxValue << " min_idx: " << minValue
         << " length: " << len_;
    return;
  }

  for (auto& idx : clip_idxs) {
    auto tmp_kps_value =
        std::static_pointer_cast<xstream::Landmarks>(feats_->datas_[idx])
            ->values_;
    auto p_tmp_kps = std::make_shared<xstream::Landmarks>();
    p_tmp_kps->values_ = tmp_kps_value;
    kpses->datas_.push_back(p_tmp_kps);
    boxes->datas_.push_back(boxes_->datas_[idx]);
  }
}

// =============================================================================

void LmksProcess::Update(std::shared_ptr<xstream::BBox> box,
                         std::shared_ptr<xstream::Landmarks> kps,
                         uint64_t timestamp) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  // auto p_box = std::static_pointer_cast<XStreamData<BBox>>(box);
  auto track_id = box->id_;
  if (track_buffers_.find(track_id) == track_buffers_.end()) {
    FeatureSequenceBuffer buffer(buff_len_);
    track_buffers_[track_id] = buffer;
  }
  auto tmp_kps = std::make_shared<xstream::Landmarks>();
  tmp_kps->values_ = kps->values_;
  track_buffers_[track_id].Update(box, tmp_kps, timestamp);
}

void LmksProcess::NormKps(std::shared_ptr<BaseDataVector> kpses,
                          std::shared_ptr<BaseDataVector> boxes) {
  HOBOT_CHECK(kpses->datas_.size() == static_cast<uint32_t>(seq_len_));
  auto first_kps =
      std::static_pointer_cast<xstream::Landmarks>(kpses->datas_[0]);
  xstream::Point center = first_kps->values_[9];
#if 0
  // fall detection
  Point left_hip = first_kps->value.values[11];
  Point right_hip = first_kps->value.values[12];
  float hip_x = left_hip.x * 0.5 + right_hip.x * 0.5;
  float hip_y = left_hip.y * 0.5 + right_hip.y * 0.5;
  center.x = hip_x;
  center.y = hip_y;
#endif
  LOGD << "center_x: " << center.x_ << ", center_y: " << center.y_;
  max_score_ = -1;
  for (auto p_kps : kpses->datas_) {
    auto kps = std::static_pointer_cast<xstream::Landmarks>(p_kps);
    for (int i = 0; i < num_kps_; ++i) {
      kps->values_[i].x_ -= center.x_;
      kps->values_[i].y_ -= center.y_;

      if (kps->values_[i].score_ > max_score_) {
        max_score_ = kps->values_[i].score_;
      }
    }
  }
  LOGD << "MAX SCORE " << max_score_;
  if (max_score_ >= 2.0 || norm_kps_conf_) {
    for (auto p_kps : kpses->datas_) {
      auto kps = std::static_pointer_cast<xstream::Landmarks>(p_kps);
      for (int i = 0; i < num_kps_; ++i) {
        kps->values_[i].score_ /= kps_norm_scale_;
      }
    }
  }
  float max_width = -1;
  float max_height = -1;
  for (auto p_box : boxes->datas_) {
    auto box = std::static_pointer_cast<xstream::BBox>(p_box);
    float cur_width = box->x2_ - box->x1_;
    float cur_height = box->y2_ - box->y1_;
    if (cur_width > max_width) {
      max_width = cur_width;
    }
    if (cur_height > max_height) {
      max_height = cur_height;
    }
  }
  LOGD << "max width: " << max_width << ", max_height: " << max_height;
  float max_border = std::max(max_width, max_height);
  for (auto p_kps : kpses->datas_) {
    auto kps = std::static_pointer_cast<xstream::Landmarks>(p_kps);
    for (int i = 0; i < num_kps_; ++i) {
      kps->values_[i].x_ /= max_border;
      kps->values_[i].y_ /= max_border;
    }
  }
}

void LmksProcess::GetClipKps(std::shared_ptr<BaseDataVector> kpses,
                             int track_id, uint64_t times_tamp) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  auto boxes = std::make_shared<BaseDataVector>();
  track_buffers_[track_id].GetClipFeatByEnd(kpses, boxes, seq_len_, stride_,
                                            max_gap_, times_tamp);
  if (kpses->datas_.size() < 1) {
    LOGD << "No clip kps get";
    return;
  }
  LOGD << "Got clip feat by end";
  NormKps(kpses, boxes);
}

void LmksProcess::Clean(std::shared_ptr<BaseDataVector> disappeared_track_ids) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  LOGD << "data preprocessor clean() called";
  for (size_t i = 0; i < disappeared_track_ids->datas_.size(); ++i) {
    auto disappeared_track_id =
        std::static_pointer_cast<xstream::Attribute_<uint32_t>>(
            disappeared_track_ids->datas_[i])
            ->value_;
    if (track_buffers_.find(disappeared_track_id) != track_buffers_.end()) {
      track_buffers_[disappeared_track_id].Clean();
      track_buffers_.erase(disappeared_track_id);
    }
  }
}

void LmksProcess::Execute(std::shared_ptr<xstream::BBox> box,
                          std::shared_ptr<xstream::Landmarks> kps,
                          std::shared_ptr<BaseDataVector> cached_kpses,
                          uint64_t timestamp) {
  Update(box, kps, timestamp);
  GetClipKps(cached_kpses, box->id_, timestamp);
}

}  // namespace inference
