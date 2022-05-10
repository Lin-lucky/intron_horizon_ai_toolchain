/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: result_util.h
 * @Brief: declaration of result_util
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Thu Dec 17 2020 11:05:30
 */

#ifndef INCLUDE_MULTITASKPOSTPROCESSMETHOD_RESULT_UTIL_H_
#define INCLUDE_MULTITASKPOSTPROCESSMETHOD_RESULT_UTIL_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <string>
#include "xstream/vision_type.h"
#include "dnn_async_data.h"

namespace xstream {

enum class BranchOutType {
  BBOX,
  KPS,
  MASK,
  REID,
  LMKS2_LABEL,
  LMKS2_OFFSET,
  LMKS1,
  POSE_3D,
  PLATE_COLOR,
  PLATE_ROW,
  KPS_LABEL,
  KPS_OFFSET,
  LMKS,
  INVALID
};

struct BranchInfo {
  BranchOutType type;
  std::string name;
  std::string box_name;
  std::unordered_map<int, std::string> labels;
};

struct ModelOutputInfo {
  uint32_t shift;
  std::vector<int> aligned_dims;
};

struct OutMsg {
  std::map<std::string, std::vector<BBox>> boxes;
  std::map<std::string, std::vector<Landmarks>> landmarks;
  std::map<std::string, std::vector<FloatFeature>> features;
  std::map<std::string, std::vector<Segmentation>> segmentations;
  std::map<std::string, std::vector<Pose3D>> poses;
  std::map<std::string, std::vector<Attribute_<int>>> attributes;
};

void CoordinateTransfrom(DnnAsyncData &dnn_result, OutMsg &det_results,
                         int model_input_width, int model_input_height);

int GetOutputInfo(BPU_MODEL_S bpu_model, std::vector<ModelOutputInfo> &info);

// inline float GetFloatByInt(int32_t value, uint32_t shift) {
//   return (static_cast<float>(value)) / (static_cast<float>(1 << shift));
// }

inline float SigMoid(const float &input) {
  return 1 / (1 + std::exp(-1 * input));
}

}  // namespace xstream
#endif  // INCLUDE_MULTITASKPOSTPROCESSMETHOD_RESULT_UTIL_H_
