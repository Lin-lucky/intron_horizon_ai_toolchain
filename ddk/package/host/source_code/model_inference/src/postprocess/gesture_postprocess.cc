/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: act_postprocess.cc
 * @Brief: implementation of GesturePostProcess
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu May 20 2021 22:13:09
 */

#include "model_inference/postprocess/gesture_postprocess.h"
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"
#include "xstream/vision_type.h"

namespace inference {

void GesturePostProcess::InitThresholds(std::string &thresholds_str) {
  size_t delimiter = thresholds_str.find(";");
  std::string group = "";
  while (delimiter != std::string::npos) {
    // one group
    group = thresholds_str.substr(0, delimiter);
    // remove current group from groups
    thresholds_str = thresholds_str.substr(delimiter + 1,
                                           thresholds_str.length() - delimiter);
    // string to float
    thresholds_.push_back(std::stof(group));
    delimiter = thresholds_str.find(";");
  }
  if (thresholds_str.length() != 0) {  // only one group
    thresholds_.push_back(std::stof(thresholds_str));
  }

  LOGD << "thresholds_.size():" << thresholds_.size();
  for (const auto &thr : thresholds_) {
    LOGD << "thr:" << thr;
  }
}

void GesturePostProcess::InitMergeGroups(std::string &merge_groups_str) {
  size_t delimiter = merge_groups_str.find(";");
  size_t sub_delimiter = std::string::npos;
  std::string group = "";
  std::vector<int> vec;
  while (delimiter != std::string::npos) {
    // one group
    group = merge_groups_str.substr(0, delimiter);
    // remove brackets
    group = group.substr(1, group.length() - 2);
    // remove current group from groups
    merge_groups_str = merge_groups_str.substr(
        delimiter + 1, merge_groups_str.length() - delimiter);
    // string to int
    sub_delimiter = group.find(",");
    while (sub_delimiter != std::string::npos) {
      int index = std::stoi(group.substr(0, sub_delimiter));
      vec.push_back(index);
      group = group.substr(sub_delimiter + 1, group.length() - sub_delimiter);
      sub_delimiter = group.find(",");
    }
    // [] without number is placeholder
    if (!group.empty()) {
      int index = std::stoi(group);
      vec.push_back(index);
    }
    merge_groups_.push_back(vec);
    delimiter = merge_groups_str.find(";");
    vec.clear();
  }
  if (merge_groups_str.length() != 0) {  // only one group
    group = merge_groups_str.substr(1, merge_groups_str.length() - 2);
    sub_delimiter = group.find(",");
    while (sub_delimiter != std::string::npos) {
      int index = std::stoi(group.substr(0, sub_delimiter));
      vec.push_back(index);
      group = group.substr(sub_delimiter + 1, group.length() - sub_delimiter);
      sub_delimiter = group.find(",");
    }
    if (!group.empty()) {
      int index = std::stoi(group);
      vec.push_back(index);
    }
    merge_groups_.push_back(vec);
    vec.clear();
  }
}

int GesturePostProcess::Init(const std::string &json_str) {
  LOGD << "GesturePostProcess Init";
  Json::Value cfg_jv;
  JSONCPP_STRING errs;
  Json::CharReaderBuilder reader_builder;
  std::unique_ptr<Json::CharReader> const json_reader(
      reader_builder.newCharReader());
  bool ret = json_reader->parse(
      json_str.c_str(), json_str.c_str() + json_str.size(), &cfg_jv, &errs);
  if (!ret || !errs.empty()) {
    LOGE << "Failed to parse config string: " << errs;
    return -1;
  }

  if (cfg_jv["threshold"].isDouble()) {
    threshold_ = cfg_jv["threshold"].asFloat();
  }
  if (cfg_jv["en_score_avg"].isInt()) {
    en_score_avg_ = cfg_jv["en_score_avg"].asInt();
  }
  std::string thresholds_str;
  std::string merge_groups_str;
  if (cfg_jv["thresholds"].isString()) {
    thresholds_str = cfg_jv["thresholds"].asString();
  }
  if (cfg_jv["merge_groups"].isString()) {
    merge_groups_str = cfg_jv["merge_groups"].asString();
  }
  if (!thresholds_str.empty()) {
    InitThresholds(thresholds_str);
  }
  if (!merge_groups_str.empty()) {
    InitMergeGroups(merge_groups_str);
  }

  HOBOT_CHECK(merge_groups_.size() != 0) << "Failed to parse merge groups";
  HOBOT_CHECK(merge_groups_[0].size() != 0) << "Empty group";

  if (en_score_avg_) {
    auto window_size =
        cfg_jv["window_size"].isNull() ? 0.0f : cfg_jv["window_size"].asFloat();
    GesturePostProcessUtil::GetInstance()->Init(window_size,
                                                merge_groups_.size());
  }

  LOGD << "threshold: " << threshold_ << ", en_score_avg: " << en_score_avg_;

  return 0;
}

int GesturePostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "GesturePostProcess Execute";
  auto act_results = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(act_results);

  // each task corresponds to one target
  for (size_t idx = 0; idx < tasks.size(); ++idx) {
    auto tensor_task =
        std::dynamic_pointer_cast<TensorInferenceEngineTask>(tasks[idx]);
    HOBOT_CHECK(tensor_task != nullptr) << "Failed to cast task " << idx;

    if (!tensor_task->float_tensors_.empty()) {
      auto infer_result =
          ActPostPro(tensor_task->float_tensors_, tensor_task->timestamp_,
                     tensor_task->track_id_);
      act_results->datas_.push_back(infer_result);
    } else {
      auto fake_result = std::make_shared<xstream::Attribute_<int32_t>>();
      fake_result->value_ = -1;
      fake_result->score_ = 0.0f;
      fake_result->state_ = xstream::DataState::INVALID;
      act_results->datas_.push_back(fake_result);
    }
  }
}

xstream::BaseDataPtr GesturePostProcess::ActPostPro(
    std::vector<FloatTensor> &float_tensors, uint64_t timestamp,
    uint64_t track_id) {
  LOGD << "GesturePostProcess ActPostPro";
  auto act_result = std::make_shared<xstream::Attribute_<int32_t>>();
  if (float_tensors.empty()) {
    LOGD << "no bpu output for this target, track_id: " << track_id
         << ", set INVALID";
    act_result->value_ = -1;
    act_result->score_ = 0.0f;
    act_result->state_ = xstream::DataState::INVALID;
  } else {
    auto mxnet_output = float_tensors[0].value.data();
    int output_size = float_tensors[0].dim[0] * float_tensors[0].dim[1] *
                      float_tensors[0].dim[2] * float_tensors[0].dim[3];
    std::vector<float> model_outs;
    model_outs.resize(output_size);
    float max_score = mxnet_output[0], sum_score = 0;
    // sofmax
    for (int i = 0; i < output_size; ++i) {
      model_outs[i] = mxnet_output[i];
      if (mxnet_output[i] > max_score) {
        max_score = mxnet_output[i];
      }
    }
    for (auto &item : model_outs) {
      item = std::exp(item - max_score);
      sum_score += item;
    }

    // merge by group
    std::vector<float> act_rets(merge_groups_.size(), 0);
    float max_group_score = 0;
    size_t max_group_index = -1;
    if (en_score_avg_ && timestamp) {
      std::vector<float> tmp_rets(merge_groups_.size(), 0);
      for (size_t g_idx = 0; g_idx < merge_groups_.size(); ++g_idx) {
        for (size_t idx = 0; idx < merge_groups_[g_idx].size(); ++idx) {
          tmp_rets[g_idx] += model_outs[merge_groups_[g_idx][idx]] / sum_score;
        }
      }
      auto avg_rets = GesturePostProcessUtil::GetInstance()->GetCachedAvgScore(
          static_cast<float>(timestamp), track_id, tmp_rets);
      for (size_t idx = 0; idx < avg_rets.size(); ++idx) {
        if (avg_rets[idx] > max_group_score) {
          max_group_score = avg_rets[idx];
          max_group_index = idx;
        }
      }
    } else {
      for (size_t g_idx = 0; g_idx < merge_groups_.size(); ++g_idx) {
        for (size_t idx = 0; idx < merge_groups_[g_idx].size(); ++idx) {
          act_rets[g_idx] += model_outs[merge_groups_[g_idx][idx]] / sum_score;
        }
        if (act_rets[g_idx] > max_group_score) {
          max_group_score = act_rets[g_idx];
          max_group_index = g_idx;
        }
      }
    }

    // currently only support gesture
    float threshold = threshold_;
    if (max_group_index >= 0 && max_group_index < thresholds_.size()) {
      threshold = thresholds_.at(max_group_index);
    }
    if (max_group_score >= threshold) {
      act_result->value_ = max_group_index;
      act_result->score_ = max_group_score;
    } else {
      act_result->value_ = 0;
      act_result->score_ = max_group_score;
    }
    LOGD << "act val: " << max_group_index << "  score: " << max_group_score
         << "  threshold: " << threshold;
  }
  return act_result;
}

}  // namespace inference
