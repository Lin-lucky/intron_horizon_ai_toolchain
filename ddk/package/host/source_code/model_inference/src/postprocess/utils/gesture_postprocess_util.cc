/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: act_postprocess_util.cc
 * @Brief: implementation of GesturePostProcessUtil
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu May 20 2021 23:20:06
 */

#include "model_inference/postprocess/utils/gesture_postprocess_util.h"
#include "xstream/vision_type.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

GesturePostProcessUtil* GesturePostProcessUtil::util_instance_ = nullptr;
std::mutex GesturePostProcessUtil::instance_mutex_;

GesturePostProcessUtil* GesturePostProcessUtil::GetInstance() {
  if (util_instance_ == nullptr) {
    std::lock_guard<std::mutex> guard(GesturePostProcessUtil::instance_mutex_);
    if (util_instance_ == nullptr) {
      util_instance_ = new (std::nothrow) GesturePostProcessUtil();
    }
  }
  return util_instance_;
}

int GesturePostProcessUtil::Init(float window_size, int score_size) {
  if (!inited_) {
    cached_scores_map_.clear();
    window_size_ = window_size;
    score_size_ = score_size;
    inited_ = true;
    LOGD << "act post util, window_size: " << window_size_
         << ", score_size: " << score_size_;
  } else {
    LOGE << "already inited with window_size: " << window_size_
         << ", score_size_: " << score_size_;
  }
  return 0;
}

std::vector<float> GesturePostProcessUtil::GetCachedAvgScore(
    float timestamp, int track_id, std::vector<float> cur_score) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  HOBOT_CHECK(cur_score.size() == static_cast<uint32_t>(score_size_))
      << "#scores mismatch!";
  // current score entry
  CachedScoreEntry cur_entry(timestamp);
  cur_entry.score_.assign(cur_score.begin(), cur_score.end());
  // get data for current track id
  auto iter = cached_scores_map_.find(track_id);
  if (iter == cached_scores_map_.end()) {
    std::deque<CachedScoreEntry> cached_scores;
    cached_scores.push_back(cur_entry);
    cached_scores_map_[track_id] = cached_scores;
    return cur_score;
  } else {
    auto cached_scores = iter->second;
    cached_scores.push_back(cur_entry);
    auto front_timestamp = cached_scores.front().timestamp_;
    while (timestamp - front_timestamp > window_size_) {
      cached_scores.pop_front();
      front_timestamp = cached_scores.front().timestamp_;
    }
    auto avg_score = CalcAvg(cached_scores);
    HOBOT_CHECK(avg_score.size() == static_cast<uint32_t>(score_size_))
        << "#scores mismatch!";
    return avg_score;
  }
}

void GesturePostProcessUtil::Clean(
    std::shared_ptr<BaseDataVector> disappeared_track_ids) {
  std::unique_lock<std::mutex> lock(map_mutex_);
  for (size_t i = 0; i < disappeared_track_ids->datas_.size(); ++i) {
    auto disappeared_track_id =
        std::static_pointer_cast<xstream::Attribute_<uint32_t>>(
            disappeared_track_ids->datas_[i])
            ->value_;
    if (cached_scores_map_.find(disappeared_track_id) !=
        cached_scores_map_.end()) {
      cached_scores_map_[disappeared_track_id].clear();
      cached_scores_map_.erase(disappeared_track_id);
    }
  }
}

std::vector<float> GesturePostProcessUtil::CalcAvg(
    std::deque<CachedScoreEntry> data) {
  std::vector<float> avg_score;
  for (int score_idx = 0; score_idx < score_size_; ++score_idx) {
    float sum = 0;
    for (size_t data_idx = 0; data_idx < data.size(); ++data_idx) {
      sum += data[data_idx].score_[score_idx];
    }
    float avg = sum / static_cast<float>(data.size());
    avg_score.push_back(avg);
  }
  HOBOT_CHECK(avg_score.size() == static_cast<uint32_t>(score_size_))
      << "#score mismatch";
  return avg_score;
}

}  // namespace inference
