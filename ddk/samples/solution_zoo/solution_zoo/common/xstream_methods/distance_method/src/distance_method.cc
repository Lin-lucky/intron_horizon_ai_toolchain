/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief     distance_method
 * @author    hangjun.yang
 * @date      2021.05.15
 */

#include "distance_method/distance_method.h"

#include <string>
#include <vector>
#include <cmath>
#include <fstream>

#include "hobotlog/hobotlog.hpp"
#include "xstream/xstream_data.h"
#include "xstream/vision_type.h"
#include "json/json.h"

namespace xstream {

int DistanceMethod::Init(const std::string &config_file_path) {
  LOGI << "Init " << config_file_path << std::endl;
  std::ifstream config_ifs(config_file_path);
  if (!config_ifs.good()) {
    LOGF << "open config file failed.";
    return -1;
  }
  Json::Value config_jv;
  config_ifs >> config_jv;
  dist_calibration_width_ = config_jv["dist_calibration_width"].asInt();
  dist_fit_factor_ = config_jv["dist_fit_factor"].asFloat();
  dist_fit_impower_ = config_jv["dist_fit_impower"].asFloat();
  dist_smooth_ = config_jv["dist_smooth"].asBool();
  return 0;
}

std::vector<BaseDataPtr> DistanceMethod::DoProcess(
    const std::vector<BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  LOGD << "input's size: " << input.size();
  // input[0] is pyramid
  // input[1] is face_box
  // output[0] is distance

  std::vector<BaseDataPtr> output;
  output.resize(1);
  if (input.size() != 2) {
    LOGF << "distance method input size is error";
  }
  auto pyramid_image =\
    std::static_pointer_cast<PyramidImageFrame>(input[0]);
  auto rois = std::static_pointer_cast<BaseDataVector>(input[1]);
  auto output_distance_attr = std::make_shared<BaseDataVector>();
  output[0] = output_distance_attr;
  if (dist_calibration_width_ == 0) {
    return output;
  }

  uint32_t box_num = rois->datas_.size();
  if (box_num == 0) {
    return output;
  }
  // get sensor resolution
  uint32_t sensor_resolution_width = pyramid_image->Width(0);
  for (size_t idx = 0; idx < box_num; ++idx) {
    auto roi = std::static_pointer_cast<BBox>(rois->datas_[idx]);
    auto dist_attr = std::make_shared<xstream::Classification>();
    dist_attr->state_ = roi->state_;
    output_distance_attr->datas_.push_back(dist_attr);
    if (roi->state_ != DataState::VALID) {
      continue;
    }

    int roi_width = roi->x2_ - roi->x1_;
    if (sensor_resolution_width != dist_calibration_width_) {
      // dist param is set in 4K resolution.
      // So when the sensor input size is not 4K, need to resize
      // the roi size.
      roi_width =\
        roi_width * dist_calibration_width_ / sensor_resolution_width;
    }
    int dist = dist_fit_factor_ * (pow(roi_width, dist_fit_impower_));
    if (dist_smooth_) {
      dist = round(static_cast<float>(dist) / 10.0) * 10;
    }
    dist_attr->value_ = dist;
    dist_attr->score_ = roi->score_;
    dist_attr->specific_type_ = std::to_string(dist);
  }
  return output;
}

}  // namespace xstream
