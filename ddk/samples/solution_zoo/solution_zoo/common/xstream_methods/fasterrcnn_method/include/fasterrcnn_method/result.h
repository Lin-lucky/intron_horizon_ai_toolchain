//
// Created by yaoyao.sun on 2019-04-23.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef INCLUDE_FASTERRCNNMETHOD_RESULT_H_
#define INCLUDE_FASTERRCNNMETHOD_RESULT_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "xstream/xstream_data.h"
#include "xstream/vision_type.h"

namespace faster_rcnn_method {

using xstream::Attribute_;
using xstream::BBox;
using xstream::FloatFeature;
using xstream::Landmarks;
using xstream::Pose3D;
using xstream::Segmentation;

struct FasterRCNNOutMsg {
  std::map<std::string, std::vector<BBox>> boxes;
  std::map<std::string, std::vector<Landmarks>> landmarks;
  std::map<std::string, std::vector<FloatFeature>> features;
  std::map<std::string, std::vector<Segmentation>> segmentations;
  std::map<std::string, std::vector<Pose3D>> poses;
  std::map<std::string, std::vector<Attribute_<int>>> attributes;
};

}  // namespace faster_rcnn_method

#endif  // FASTERRCNNMETHOD_RESULT_H_
