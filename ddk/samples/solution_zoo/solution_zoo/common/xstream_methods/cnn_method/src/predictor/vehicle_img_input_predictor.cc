/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: VehicleImgInputPredictor.cpp
 * @Brief: definition of the VehicleImgInputPredictor
 * @Author: ronghui.zhang
 * @Email: ronghui.zhang@horizon.ai
 * @Date: 2020-03-02 14:27:05
 */

#include "cnn_method/predictor/vehicle_img_input_predictor.h"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "cnn_method/util/util.h"
#include "hobotlog/hobotlog.hpp"
#include "image_utils.h"
#include "opencv2/opencv.hpp"
#include "xstream/profiler.h"
#include "xstream/vision_type.h"

using xstream::BBox;

namespace xstream {
std::shared_ptr<BaseDataVector> VehicleImgInputPredictor::RoisConvert(
    std::vector<BaseDataPtr> input_data) {
  HOBOT_CHECK(input_data.size() == 3)
      << "VehicleImgInputPredictor'input size must be 3";
  auto rois = std::static_pointer_cast<BaseDataVector>(input_data[0]);
  auto plate_types = std::static_pointer_cast<BaseDataVector>(input_data[2]);

  std::shared_ptr<BaseDataVector> plate_vec;
  plate_vec = PlateBoxProcess(rois, plate_types);

  return plate_vec;
}

std::shared_ptr<BaseDataVector> VehicleImgInputPredictor::PlateBoxProcess(
    std::shared_ptr<BaseDataVector> box_input,
    std::shared_ptr<BaseDataVector> plate_types) {
  std::size_t box_num = box_input->datas_.size();

  auto plate_boxes = std::make_shared<BaseDataVector>();

  for (std::size_t box_idx = 0; box_idx < box_num; box_idx++) {
    auto p_roi =
        std::static_pointer_cast<BBox>(box_input->datas_[box_idx]);
    if (p_roi->state_ != xstream::DataState::VALID) {
      continue;
    }

    auto plate_type = std::static_pointer_cast<xstream::Attribute_<int>>(
        plate_types->datas_[box_idx]);

    if (plate_type->value_ == 1) {
      auto plate_box1 = std::make_shared<xstream::BBox>();

      int roi_h = p_roi->y2_ - p_roi->y1_;
      plate_box1->y1_ = p_roi->y1_;
      plate_box1->x1_ = p_roi->x1_;
      plate_box1->x2_ = p_roi->x2_;
      plate_box1->y2_ =
          p_roi->y1_ + roi_h * (static_cast<float>(82 / 220.) + 0.04f) + 1;

      plate_box1->state_ = xstream::DataState::VALID;
      plate_boxes->datas_.push_back(plate_box1);

      auto plate_box2 = std::make_shared<xstream::BBox>();

      plate_box2->y1_ =
          p_roi->y1_ + roi_h * (static_cast<float>(82 / 220.) - 0.008f);
      plate_box2->x1_ = p_roi->x1_;
      plate_box2->x2_ = p_roi->x2_;
      plate_box2->y2_ = p_roi->y2_;
      plate_box2->state_ = xstream::DataState::VALID;
      plate_boxes->datas_.push_back(plate_box2);

    } else {
      auto plate_box = std::make_shared<BBox>();
      plate_box = p_roi;
      plate_box->state_ = xstream::DataState::VALID;
      plate_boxes->datas_.push_back(plate_box);
    }
  }
  return plate_boxes;
}

}  // namespace xstream
