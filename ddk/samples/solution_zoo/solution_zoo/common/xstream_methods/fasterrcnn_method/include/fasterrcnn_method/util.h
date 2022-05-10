/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     Util
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#ifndef INCLUDE_FASTERRCNNMETHOD_UTIL_H_
#define INCLUDE_FASTERRCNNMETHOD_UTIL_H_

#include <string.h>

#include <memory>
#include <numeric>
#include <string>

#include "fasterrcnn_method/result.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_data.h"

using xstream::FloatFeature;

namespace faster_rcnn_method {

// Align by 16
#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

inline void MemZero(void* p, size_t n)
{
  memset(p, 0, n);
}

inline void l2_norm(FloatFeature &feature) {
  float sum = 0.0;
  sum = std::inner_product(feature.values_.begin(), feature.values_.end(), feature.values_.begin(), sum);
  float eps = 1e-10;
  sum = sqrt(sum) + eps;
  for (auto &value : feature.values_) {
    value = value / sum;
  }
  return;
}

inline float SigMoid(const float &input) {
  return 1 / (1 + std::exp(-1 * input));
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  return (static_cast<float>(value)) / (static_cast<float>(1 << shift));
}

// coordinate transform.
// fasterrcnn model's input size maybe not eqaul to origin image size,
// needs coordinate transform for detection result.

void CoordinateTransform(FasterRCNNOutMsg &det_result,
                         int src_image_width, int src_image_height,
                         int model_input_width, int model_input_hight) {
  for (auto &boxes : det_result.boxes) {
    for (auto &box : boxes.second) {
      box.x1_ = box.x1_ * src_image_width / model_input_width;
      box.y1_ = box.y1_ * src_image_height / model_input_hight;
      box.x2_ = box.x2_ * src_image_width / model_input_width;
      box.y2_ = box.y2_ * src_image_height / model_input_hight;
    }
  }

  for (auto &landmarks : det_result.landmarks) {
    for (auto &landmark : landmarks.second) {
      for (auto &point : landmark.values_) {
        point.x_ = point.x_ * src_image_width / model_input_width;
        point.y_ = point.y_ * src_image_height / model_input_hight;
      }
    }
  }
}

std::string GetParentPath(const std::string &path) {
  auto pos = path.rfind('/');
  if (std::string::npos != pos) {
    auto parent = path.substr(0, pos);
    return parent + "/";
  } else {
    return std::string("./");
  }
}

}  // namespace faster_rcnn_method

#endif  // INCLUDE_FASTERRCNNMETHOD_UTIL_H_
