/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: act_postprocess_util.h
 * @Brief: declaration of GesturePostProcessUtil
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu May 20 2021 23:05:32
 */

#ifndef ACT_POSTPROCESS_UTIL_H_
#define ACT_POSTPROCESS_UTIL_H_

#include <mutex>
#include <vector>
#include <deque>
#include <memory>
#include <unordered_map>
#include "xstream/xstream_world.h"

namespace inference {

using xstream::BaseDataVector;

class CachedScoreEntry {
 public:
  explicit CachedScoreEntry(float timestamp) : timestamp_(timestamp) {}
  void Clean() { score_.clear(); }
  std::vector<float> score_;
  float timestamp_;
};

class GesturePostProcessUtil {
 public:
  GesturePostProcessUtil() {
    inited_ = false;
  }

  static GesturePostProcessUtil *GetInstance();

  int Init(float window_size, int score_size);

  std::vector<float> GetCachedAvgScore(
    float timestamp, int track_id, std::vector<float> cur_score);

  void Clean(std::shared_ptr<BaseDataVector> disappeared_track_ids);

 private:
  std::vector<float> CalcAvg(std::deque<CachedScoreEntry> data);

 private:
  std::unordered_map<int, std::deque<CachedScoreEntry>> cached_scores_map_;
  float window_size_;
  int score_size_;
  bool inited_;
  std::mutex map_mutex_;
  static GesturePostProcessUtil *util_instance_;
  static std::mutex instance_mutex_;
};

}   // namespace inference

#endif  // ACT_POSTPROCESS_UTIL_H_
