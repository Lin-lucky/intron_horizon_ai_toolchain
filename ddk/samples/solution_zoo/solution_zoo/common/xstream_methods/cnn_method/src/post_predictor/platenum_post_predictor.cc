/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: VehicleType.h
 * @Brief: declaration of the PlateNumPostPredictor
 * @Author: yatao.fu
 * @Email: yatao.fu@horizon.ai
 * @Date: 2019-09-28 16:18:28
 * @Last Modified by: yatao.fu
 * @Last Modified time: 16:18:28
 */

#include <memory>
#include <string>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/post_predictor/platenum_post_predictor.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

using xstream::BBox;

namespace xstream {

int32_t PlateNumPostPredictor::Init(std::shared_ptr<CNNMethodConfig> config) {
  PostPredictor::Init(config);
  return 0;
}
void PlateNumPostPredictor::Do(CNNMethodRunData *run_data) {
  {
    auto &mxnet_output = run_data->mxnet_output;
    std::vector<BaseDataPtr> &batch_output = run_data->output;
    batch_output.resize(output_slot_size_);

    auto &input_data = (*(run_data->input));
    auto plate_types = std::static_pointer_cast<BaseDataVector>(input_data[2]);
    auto rois = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    int dim_size = rois->datas_.size();

    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();
      //  base_data_vector->name_ = output_slot_names_[i];
      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }
    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");
      RUN_FPS_PROFILER(model_name_ + "_post");
      auto data_vector =
          std::static_pointer_cast<BaseDataVector>(batch_output[0]);

      for (int dim_idx = 0, box_idx = 0; box_idx < dim_size; box_idx++) {
        // loop target

        auto plate_type = std::static_pointer_cast<xstream::Attribute_<int>>(
            plate_types->datas_[box_idx]);
        auto vehiclePtr =
            std::make_shared<xstream::DataArray_<int>>();

        auto p_roi =
            std::static_pointer_cast<BBox>(rois->datas_[box_idx]);
        if (p_roi->state_ != xstream::DataState::VALID) {
          vehiclePtr->values_.clear();
        } else if (plate_type->value_ == 1) {
          auto &target_mxnet_up = mxnet_output[dim_idx++];
          auto &target_mxnet_down = mxnet_output[dim_idx++];
          // std::string str_up, str_down;
          std::vector<int> vec_up;
          std::vector<int> vec_down;

          if (target_mxnet_up.size() == 0) {
            vec_up.clear();
          } else {
            PlateNumPro(target_mxnet_up[0], vec_up);
          }
          if (target_mxnet_down.size() == 0) {
            vec_down.clear();
          } else {
            PlateNumPro(target_mxnet_down[0], vec_down);
          }

          if ((vec_up.size() == 0) || (vec_down.size() == 0)) {
            vehiclePtr->values_.clear();
          } else {
            vehiclePtr->values_.assign(vec_up.begin(), vec_up.end());
            vehiclePtr->values_.insert(vehiclePtr->values_.end(),
                                       vec_down.begin(), vec_down.end());
          }
        } else {
          auto &target_mxnet = mxnet_output[dim_idx++];
          if (target_mxnet.size() == 0) {
            vehiclePtr->values_.clear();
          } else {
            PlateNumPro(target_mxnet[0], (vehiclePtr->values_));
          }
        }
        // 车牌号只有7位或8位
        if ((vehiclePtr->values_.size() != 7) &&
            (vehiclePtr->values_.size() != 8)) {
          vehiclePtr->values_.clear();
        }

        data_vector->datas_.push_back(vehiclePtr);
      }
    }
  }
}

void PlateNumPostPredictor::PlateNumPro(const std::vector<int8_t> &mxnet_outs,
                                        std::vector<int> &platenum) {
  const int slice_num = 16;
  const int score_num = 74;
  if (mxnet_outs.size() != slice_num * score_num * sizeof(float)) {
    return;
  }

  auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs.data());

  std::vector<int> scores;

  // get slice_num max index
  for (int index = 0; index < slice_num; index++) {
    int max_index = -1;
    float max_score = -9999.0f;
    for (int i = 0; i < score_num; ++i) {
      if (mxnet_out[index * score_num + i] > max_score) {
        max_score = mxnet_out[index * score_num + i];
        max_index = i;
      }
    }

    scores.push_back(max_index);
  }
  // skip the spilt/repeat character
  int last_index = -1;
  for (auto iter = scores.begin(); iter != scores.end();) {
    if (last_index == *iter || 0 == *iter) {
      last_index = *iter;
      iter = scores.erase(iter);
    } else {
      last_index = *iter;
      iter++;
    }
  }

  platenum.assign(scores.begin(), scores.end());

  return;
}

std::string PlateNumPostPredictor::PlateNumPro(
    const std::vector<int8_t> &mxnet_outs) {
  const int slice_num = 16;
  const int score_num = 74;
  if (mxnet_outs.size() != slice_num * score_num * sizeof(float)) {
    return "unknown";
  }

  auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs.data());

  std::vector<int> scores;

  // get slice_num max index
  for (int index = 0; index < slice_num; index++) {
    int max_index = -1;
    float max_score = -9999.0f;

    for (int i = 0; i < score_num; ++i) {
      if (mxnet_out[index * score_num + i] > max_score) {
        max_score = mxnet_out[index * score_num + i];
        max_index = i;
      }
    }
    scores.push_back(max_index);
  }

  // skip the spilt/repeat character
  int last_index = -1;
  for (auto iter = scores.begin(); iter != scores.end();) {
    if (last_index == *iter || 0 == *iter) {
      last_index = *iter;
      iter = scores.erase(iter);
    } else {
      last_index = *iter;
      iter++;
    }
  }

  std::string plate_num;
  if ((scores.size() == 7 || scores.size() == 8) && scores[0] > 34 &&
      scores[1] >= 1 && scores[1] <= 24) {
    for (std::size_t i = 0; i < scores.size(); i++) {
      plate_num = plate_num + std::to_string(scores[i]) + "_";
    }
  } else {
    plate_num = "unknown";
  }

  return plate_num;
}

}  // namespace xstream
