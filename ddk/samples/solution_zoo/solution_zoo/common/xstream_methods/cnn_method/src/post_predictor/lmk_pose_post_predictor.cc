/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: LmkPosePostPredictor.cpp
 * @Brief: definition of the LmkPosePostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:18:10
 */

#include <algorithm>
#include <memory>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/post_predictor/lmk_pose_post_predictor.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

namespace xstream {

void LmkPosePostPredictor::Do(CNNMethodRunData *run_data) {
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
        HandleLmkPose(mxnet_output[dim_idx], *xstream_box,
                      run_data->real_nhwc, &output);

        for (int i = 0; i < output_slot_size_; i++) {
          auto base_data_vector =
              std::static_pointer_cast<BaseDataVector>(batch_output[i]);
          base_data_vector->datas_.push_back(output[i]);
        }
      }
    }
  }
}

void LmkPosePostPredictor::HandleLmkPose(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    const xstream::BBox &box,
    const std::vector<std::vector<uint32_t>> &nhwc,
    std::vector<BaseDataPtr> *output) {
  if (mxnet_outs.size()) {
    auto lmk = LmkPostPro(mxnet_outs, box, nhwc);
    output->push_back(lmk);
    if (mxnet_outs.size() > 3) {
      auto pose = PosePostPro(mxnet_outs[3]);
      output->push_back(pose);
    } else {
      auto pose = std::make_shared<xstream::Pose3D>();
      pose->state_ = DataState::INVALID;
      output->push_back(std::static_pointer_cast<BaseData>(pose));
    }
  } else {
    auto landmarks = std::make_shared<xstream::Landmarks>();
    landmarks->state_ = DataState::INVALID;
    output->push_back(std::static_pointer_cast<BaseData>(landmarks));
    auto pose = std::make_shared<xstream::Pose3D>();
    pose->state_ = DataState::INVALID;
    output->push_back(std::static_pointer_cast<BaseData>(pose));
  }
}

BaseDataPtr LmkPosePostPredictor::LmkPostPro(
    const std::vector<std::vector<int8_t>> &mxnet_outs,
    const xstream::BBox &box,
    const std::vector<std::vector<uint32_t>> &nhwc) {
  static const float SCORE_THRESH = 0.0;
  static const float REGRESSION_RADIUS = 3.0;
  static const float STRIDE = 4.0;
  static const float num = 1;
  static const float height_m = 16;
  static const float width_m = 16;

  auto fl_scores = reinterpret_cast<const float *>(mxnet_outs[0].data());
  auto fl_coords = reinterpret_cast<const float *>(mxnet_outs[1].data());
  std::vector<std::vector<float>> points_score;
  std::vector<std::vector<float>> points_x;
  std::vector<std::vector<float>> points_y;
  points_score.resize(5);
  points_x.resize(5);
  points_y.resize(5);

  // nhwc, 1x16x16x5, 1x16x16x10
  for (int n = 0; n < num; ++n) {          // n
    for (int i = 0; i < height_m; ++i) {   // h
      for (int j = 0; j < width_m; ++j) {  // w
        int index_score = n * nhwc[0][1] * nhwc[0][2] * nhwc[0][3] +
                          i * nhwc[0][2] * nhwc[0][3] + j * nhwc[0][3];
        int index_coords = n * nhwc[1][1] * nhwc[1][2] * nhwc[0][3] +
                           i * nhwc[1][2] * nhwc[1][3] + j * nhwc[1][3];
        for (int k = 0; k < 5; ++k) {  // c
          auto score = fl_scores[index_score + k];
          if (score > SCORE_THRESH) {
            points_score[k].push_back(score);
            float x = (j + 0.5 -
                       fl_coords[index_coords + 2 * k] * REGRESSION_RADIUS) *
                      STRIDE;
            float y =
                (i + 0.5 -
                 fl_coords[index_coords + 2 * k + 1] * REGRESSION_RADIUS) *
                STRIDE;
            x = std::min(std::max(x, 0.0f), width_m * STRIDE);
            y = std::min(std::max(y, 0.0f), height_m * STRIDE);
            points_x[k].push_back(x);
            points_y[k].push_back(y);
          }
        }
      }
    }
  }
  auto landmarks = std::make_shared<xstream::Landmarks>();
  landmarks->values_.resize(5);
  for (int i = 0; i < 5; ++i) {
    auto &poi = landmarks->values_[i];
    poi.x_ = Mean(points_x[i]);
    poi.y_ = Mean(points_y[i]);
    poi.x_ = box.x1_ + poi.x_ / 64 * (box.x2_ - box.x1_);
    poi.y_ = box.y1_ + poi.y_ / 64 * (box.y2_ - box.y1_);
    poi.score_ = static_cast<float>(points_score[i].size());
    if (poi.score_ <= 0.000001 && mxnet_outs.size() > 2) {
      auto reg_coords = reinterpret_cast<const float *>(mxnet_outs[2].data());
      poi.x_ = box.x1_ + reg_coords[i << 1] * (box.x2_ - box.x1_);
      poi.y_ = box.y1_ + reg_coords[(i << 1) + 1] * (box.y2_ - box.y1_);
    }
  }
  return std::static_pointer_cast<BaseData>(landmarks);
}

BaseDataPtr LmkPosePostPredictor::PosePostPro(
    const std::vector<int8_t> &mxnet_outs) {
  auto pose = std::make_shared<xstream::Pose3D>();
  auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs.data());
  pose->yaw_ = mxnet_out[0] * 90.0;
  pose->pitch_ = mxnet_out[1] * 90.0;
  pose->roll_ = mxnet_out[2] * 90.0;
  return std::static_pointer_cast<BaseData>(pose);
}
}  // namespace xstream
