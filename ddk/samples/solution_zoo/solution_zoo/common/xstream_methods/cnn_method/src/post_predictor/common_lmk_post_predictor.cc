/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CommonLmkPostPredictor.h
 * @Brief: declaration of the CommonLmkPostPredictor
 * @Author: fei.cheng
 * @Email: fei.cheng@horizon.ai
 * @Date: 2020-07-18 14:18:28
 * @Last Modified by: fei.cheng
 * @Last Modified time: 2019-07-18 15:13:07
 */

#include "cnn_method/post_predictor/common_lmk_post_predictor.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

namespace xstream {
int32_t CommonLmkPostPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  if (nullptr == config) {
    return -1;
  }
  model_name_ = config->GetSTDStringValue("model_name");
  lmk_num_ = config->GetIntValue("lmk_num", 21);
  post_fn_ = config->GetSTDStringValue("post_fn");

  feature_w_ = config->GetIntValue("feature_w", 32);
  feature_h_ = config->GetIntValue("feature_h", 32);
  i_o_stride_ = config->GetIntValue("i_o_stride", 4);
  output_slot_size_ = config->GetIntValue("output_size");
  vector_size_ = config->GetIntValue("vector_size", 32);
  HOBOT_CHECK(output_slot_size_ > 0);

  std::string s_norm_method = config->GetSTDStringValue("norm_method",
                                                        "norm_by_nothing");
  expand_scale_ = config->GetFloatValue("expand_scale", 1.0f);
  auto iter = g_norm_method_map.find(s_norm_method);
  HOBOT_CHECK(iter != g_norm_method_map.end())
  << "norm_method is unknown:" << s_norm_method;
  norm_type_ = iter->second;
  aspect_ratio_ = config->GetFloatValue("aspect_ratio", 1.0f);

  return 0;
}

void CommonLmkPostPredictor::Do(CNNMethodRunData *run_data) {
  {
    int dim_size = run_data->input_dim_size;
    auto &mxnet_output = run_data->mxnet_output;
    std::vector<BaseDataPtr> &batch_output = run_data->output;
    batch_output.resize(output_slot_size_);
    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }
    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");
      RUN_FPS_PROFILER(model_name_ + "_post");
      auto boxes = std::static_pointer_cast<BaseDataVector>(
          (*(run_data->input))[0]);

      for (int dim_idx = 0; dim_idx < dim_size; dim_idx++) {
        std::vector<BaseDataPtr> output;
        auto xstream_box =
            std::static_pointer_cast<xstream::BBox>(boxes->datas_[dim_idx]);
        HandleLmk(mxnet_output[dim_idx], *xstream_box,
                  run_data->real_nhwc, run_data->all_shift, &output);

        for (int i = 0; i < output_slot_size_; i++) {
          auto base_data_vector =
              std::static_pointer_cast<BaseDataVector>(batch_output[i]);
          base_data_vector->datas_.push_back(output[i]);
        }
      }
    }
  }
}

void CommonLmkPostPredictor::HandleLmk(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    const xstream::BBox &box,
    const std::vector<std::vector<uint32_t>> &nhwc,
    const std::vector<std::vector<uint32_t>> &shifts,
    std::vector<BaseDataPtr> *output) {
  if (mxnet_outs.size()) {
    BaseDataPtr lmk;
    if ("common_lmk" == post_fn_) {
      lmk = Lmk3Post(mxnet_outs, box, nhwc, shifts);
    } else if ("common_lmk_106pts" == post_fn_) {
      lmk = Lmk4Post(mxnet_outs, box);
    } else {
      lmk = std::make_shared<xstream::Landmarks>();
      lmk->state_ = DataState::INVALID;
    }
    output->push_back(lmk);
  } else {
    auto landmarks = std::make_shared<xstream::Landmarks>();
    landmarks->state_ = DataState::INVALID;
    output->push_back(std::static_pointer_cast<BaseData>(landmarks));
  }
}

int CommonLmkPostPredictor::CalIndex(int k, int i, int j) {
  auto index = i * feature_w_ * lmk_num_ + \
          j * lmk_num_ + k;
  return index;
}

// Calculate the index of point in the 1d tensor array,
// with channel k, vector index i.
int CommonLmkPostPredictor::CalIndex(int k, int i) {
    return i * lmk_num_ + k;
}

void CommonLmkPostPredictor::Lmks4PostProcess(
        const float *vector_pred,
        const xstream::BBox &box,
        std::vector<xstream::Point> &lmks4,
        int axis) {
  // lmks4 post process
  // nhwc, heatmap_pred: 1x64x1x68
  for (size_t k = 0; k < lmk_num_; ++k) {  // c
    float max_value = 0;
    int max_index = 0;
    for (int i = 0; i < vector_size_; ++i) {  // vector_size_
      int index = CalIndex(k, i);
      float value = vector_pred[index];
      if (value > max_value) {
        max_value = value;
        max_index = i;
      }
    }

    float index = max_index;
    float diff = 0;
    if (index > 0 && index < vector_size_ - 1) {
      int left = CalIndex(k, index-1);
      int right = CalIndex(k, index+1);
      diff = vector_pred[right] - vector_pred[left];
    }

    if (diff > 0) {
      diff = 0.25;
    } else if (diff < 0) {
      diff = -0.25;
    } else {
      diff = 0;
    }

    index = index + (diff * 1.0 + 0.5);
    index = index * i_o_stride_;

    auto cal_score = [&max_value] (float score) -> float {
        float new_score = 0.0;
        float val = max_value / 2.0;
        if (val > 1.0) {
          val = 1.0;
        }
        if (score > 0) {
          new_score = std::min(score, val);
        } else {
          new_score = val;
        }
        return new_score;
    };

    if (axis == 0) {
      // vertical vector
      lmks4.at(k).y_ = index * box.Height() / (
              vector_size_ * i_o_stride_) + box.y1_;
      lmks4.at(k).score_ = cal_score(lmks4.at(k).score_);
    } else {
      // horizontal vector
      lmks4.at(k).x_ = index * box.Width() / (
              vector_size_ * i_o_stride_) + box.x1_;
      lmks4.at(k).score_ = cal_score(lmks4.at(k).score_);
    }
  }  // c
}

BaseDataPtr CommonLmkPostPredictor::Lmk4Post(
        const std::vector<std::vector<int8_t>> &mxnet_outs,
        const xstream::BBox &box) {
  auto vector_x = reinterpret_cast<const float *>(mxnet_outs[0].data());
  auto vector_y = reinterpret_cast<const float *>(mxnet_outs[1].data());
  auto landmarks = std::make_shared<xstream::Landmarks>();
  landmarks->values_.resize(lmk_num_);
  Lmks4PostProcess(vector_x, box, landmarks->values_, 1);
  Lmks4PostProcess(vector_y, box, landmarks->values_, 0);

  if (NormMethod::BPU_MODEL_NORM_BY_LSIDE_RATIO == norm_type_) {
    const auto& c_x = box.CenterX();
    const auto& c_y = box.CenterY();
    for (auto& val : landmarks->values_) {
      val.x_ = (val.x_ - c_x) * expand_scale_ + c_x;
      val.y_ = (val.y_ - c_y) * expand_scale_ + c_y;
    }
  }
  return std::static_pointer_cast<BaseData>(landmarks);
}

BaseDataPtr CommonLmkPostPredictor::Lmk3Post(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    const xstream::BBox &box,
    const std::vector<std::vector<uint32_t>> &nhwc,
    const std::vector<std::vector<uint32_t>> &shifts) {
  auto heatmap_pred = reinterpret_cast<const int8_t *>(mxnet_outs[0].data());
  // static const float diff_coeff = 1.0;// for float32 of model output
  HOBOT_CHECK(shifts.size() > 0);
  HOBOT_CHECK(shifts[0].size() == lmk_num_);
  int raw_height = floor(box.Height());
  int raw_width = floor(box.Width());
  auto ratio_h = feature_h_ * i_o_stride_;
  auto ratio_w = feature_w_ * i_o_stride_;
#ifdef SEARCH_PERFORMANCE
  int step = 4;
#else
  int step = 1;
#endif
  auto landmarks = std::make_shared<xstream::Landmarks>();
  landmarks->values_.resize(lmk_num_);
  for (size_t k = 0; k < lmk_num_; ++k) {  // c
    int8_t max_value = 0;
    int max_index[2] = {0, 0};
    for (auto i = 0; i < feature_h_; i += step) {  // h
      for (auto j = 0; j < feature_w_; j += step) {  // w
        int index = CalIndex(k, i, j);
        int8_t value = heatmap_pred[index];
        if (value > max_value) {
            max_value = value;
            max_index[0] = i;
            max_index[1] = j;
          }
        }  // w
      }  // h
#ifdef SEARCH_PERFORMANCE
      auto is_max = false;
      auto campare_func = [&max_index, &max_value, &is_max, heatmap_pred, this](
                              int i, int j, int k) {
        int index = CalIndex(k, i, j);
        if (max_value < heatmap_pred[index]) {
          max_value = heatmap_pred[index];
          max_index[0] = i;
          max_index[1] = j;
          is_max = false;
        }
      };
      while (false == is_max) {
        is_max = true;
        int i = max_index[0];
        int j = max_index[1];
        if (i > 0) {
          campare_func(i - 1, j, k);
        }
        if (i < feature_h_) {
          campare_func(i + 1, j, k);
        }
        if (j > 0) {
          campare_func(i, j - 1, k);
        }
        if (j < feature_w_) {
          campare_func(i, j + 1, k);
        }
      }
#endif
    float y = max_index[0];
    float x = max_index[1];

    float diff_x = 0, diff_y = 0;
    if (y > 0 && y < feature_h_ - 1) {
      int top = CalIndex(k, y-1, x);
      int down = CalIndex(k, y+1, x);
      diff_y = heatmap_pred[down] - heatmap_pred[top];
    }
    if (x > 0 && x < feature_w_ - 1) {
      int left = CalIndex(k, y, x-1);
      int right = CalIndex(k, y, x+1);
      diff_x = heatmap_pred[right] - heatmap_pred[left];
    }

    // y = y + (diff_y * diff_coeff + 0.5); // for float32 of model output
    // x = x + (diff_x * diff_coeff + 0.5); // for float32 of model output
    y = y + (diff_y > 0 ? 0.25 : -0.25) + 0.5;
    x = x + (diff_x > 0 ? 0.25 : -0.25) + 0.5;
    y = y * i_o_stride_;
    x = x * i_o_stride_;
    y = y * raw_height / ratio_h + box.y1_;
    x = x * raw_width / ratio_w + box.x1_;

    auto &poi = landmarks->values_[k];
    poi.x_ = x;
    poi.y_ = y;
    float max_score = static_cast<float>(max_value);
    poi.score_ = max_score / (1 << shifts[0][k]);
    LOGD << "hand_lmk" << k << " x y score:( " << x << " " << y << " "
         << poi.score_;
  }  // c

  return std::static_pointer_cast<BaseData>(landmarks);
}
}  // namespace xstream
