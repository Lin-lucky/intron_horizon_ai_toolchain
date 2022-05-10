/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: VehicleTypePostPredictor.cpp
 * @Brief: definition of the VehicleTypePostPredictor
 * @Author: yatao.fu
 * @Email: yatao.fu@horizon.ai
 * @Date: 2019-09-28 16:18:28
 * @Last Modified by: yatao.fu
 * @Last Modified time: 16:18:28
 */

#include <memory>
#include <vector>

#include "cnn_method/cnn_const.h"
#include "cnn_method/post_predictor/vehicle_type_post_predictor.h"
#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

namespace xstream {
int32_t VehicleTypePostPredictor::Init(
    std::shared_ptr<CNNMethodConfig> config) {
  PostPredictor::Init(config);
  return 0;
}
void VehicleTypePostPredictor::Do(CNNMethodRunData *run_data) {
  {
    int dim_size = run_data->input_dim_size;
    auto &mxnet_output = run_data->mxnet_output;
    std::vector<BaseDataPtr> &batch_output = run_data->output;
    batch_output.resize(output_slot_size_);
    auto &input_data = (*(run_data->input));
    auto rois = std::static_pointer_cast<BaseDataVector>(input_data[0]);
    for (int i = 0; i < output_slot_size_; i++) {
      auto base_data_vector = std::make_shared<BaseDataVector>();

#if 0
      base_data_vector->name_ = output_slot_names_[i];
#endif

      batch_output[i] = std::static_pointer_cast<BaseData>(base_data_vector);
    }
    {
      RUN_PROCESS_TIME_PROFILER(model_name_ + "_post");
      RUN_FPS_PROFILER(model_name_ + "_post");

      auto data_vector =
          std::static_pointer_cast<BaseDataVector>(batch_output[0]);

      for (int dim_idx = 0; dim_idx < dim_size; dim_idx++) {  // loop target
        auto vehiclePtr = std::make_shared<xstream::Attribute_<int>>();
        auto p_roi =
            std::static_pointer_cast<BBox>(rois->datas_[dim_idx]);
#if 0
        if (log_file_.is_open()) {
          log_file_ << "box: " << p_roi->value.x1_ << " " << p_roi->value.y1_
                    << " " << p_roi->value.x2_ << " " << p_roi->value.y2_
                    << std::endl;
        }
#endif
        auto &target_mxnet = mxnet_output[dim_idx];

        if (target_mxnet.size() == 0) {
          vehiclePtr->value_ = -1;
        } else {
          vehiclePtr->value_ = VehicleTypePro(target_mxnet[0]);
        }
#if 0
        if (dim_idx == dim_size - 1) {
          if (log_file_.is_open()) {
            log_file_ << std::endl;
          }
        }
#endif

        data_vector->datas_.push_back(vehiclePtr);
      }
    }
  }
}

int VehicleTypePostPredictor::VehicleTypePro(
    const std::vector<int8_t> &mxnet_outs) {
  if (mxnet_outs.size() != 12 * sizeof(float)) {
    return -1;
  }

  auto mxnet_out = reinterpret_cast<const float *>(mxnet_outs.data());
  float max_score = -9999.0f;
  int max_index = -1;

#if 0
  if (log_file_.is_open()) {
    log_file_ << "score: ";
  }
#endif

  for (int index = 0; index < 12; index++) {
#if 0
    if (log_file_.is_open()) {
      log_file_ << mxnet_out[index] << " ";
    }
#endif

    if (mxnet_out[index] > max_score) {
      max_score = mxnet_out[index];
      max_index = index;
    }
  }

#if 0
  if (log_file_.is_open()) {
    log_file_ << std::endl;
  }
#endif

  return max_index;
}

}  // namespace xstream
