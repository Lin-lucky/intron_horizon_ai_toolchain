/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: FaceIdPostPredictor.cpp
 * @Brief: definition of the FaceIdPostPredictor
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:27:05
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 15:18:10
 */

#include <memory>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/post_predictor/faceid_post_predictor.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

namespace xstream {

void FaceIdPostPredictor::Do(CNNMethodRunData *run_data) {
  {
    auto &input_data = (*(run_data->input));
    auto snaps = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    auto person_num = snaps->datas_.size();
    uint total_snap = run_data->input_dim_size;
    auto &mxnet_output = run_data->mxnet_output;

    std::vector<BaseDataPtr> &batch_output = run_data->output;
    batch_output.resize(output_slot_size_);
    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }

    auto data_vector =
        std::static_pointer_cast<BaseDataVector>(batch_output[0]);
    for (uint32_t person_idx = 0, g_snap_idx = 0; person_idx < person_num;
         person_idx++) {
      auto face_features = std::make_shared<BaseDataVector>();  // one person
      auto one_person_snaps =
          dynamic_cast<BaseDataVector *>(snaps->datas_[person_idx].get());
      if (!one_person_snaps) {
        continue;
      }
      for (size_t snap_idx = 0; snap_idx < one_person_snaps->datas_.size()
                                  && g_snap_idx < total_snap; snap_idx++) {
        RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");
        RUN_FPS_PROFILER(model_name_ + "_post");
        auto face_feature = FaceFeaturePostPro(mxnet_output[g_snap_idx++]);
        face_features->datas_.push_back(face_feature);
      }
      data_vector->datas_.push_back(face_features);
    }
  }
}

BaseDataPtr FaceIdPostPredictor::FaceFeaturePostPro(
    const std::vector<std::vector<int8_t>> &mxnet_outs) {
  if (mxnet_outs.size() == 0 || mxnet_outs[0].size() == 0) {
    auto feature_invalid = std::make_shared<xstream::FloatFeature>();
    feature_invalid->state_ = DataState::INVALID;
    return std::static_pointer_cast<BaseData>(feature_invalid);
  }
  static const int kFeatureCnt = 128;
  auto mxnet_rlt = reinterpret_cast<const float *>(mxnet_outs[0].data());

  auto feature = std::make_shared<xstream::FloatFeature>();
  feature->values_.resize(kFeatureCnt);
  LOGI << "feature: [";
  for (int i = 0; i < kFeatureCnt; i++) {
    feature->values_[i] = mxnet_rlt[i];
    LOGI << mxnet_rlt[i];
  }
  LOGI << "]";
  l2_norm(feature->values_, kFeatureCnt);
  return std::static_pointer_cast<BaseData>(feature);
}

}  // namespace xstream
