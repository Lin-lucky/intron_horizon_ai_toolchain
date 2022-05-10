/*
 * Copyright (c) 2019 Horizon Robotics. All rights reserved
 * @Author  jianglei.huang
 * @version 0.0.1
 * @date  2019.10.16
 */

#include "match_utility/utils.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>


namespace hobot {
namespace vehicle_match_strategy {

float BBoxCenterX(const xstream::BBox &bbox) {
  return (bbox.x1_ + bbox.x2_) / 2.0;
}

float BBoxCenterY(const xstream::BBox &bbox) {
  return (bbox.y1_ + bbox.y2_) / 2.0;
}

float IntersectionArea(const xstream::BBox &bbox1, const xstream::BBox &bbox2) {
  float x = std::max(bbox1.x1_, bbox2.x1_);
  float y = std::max(bbox1.y1_, bbox2.y1_);
  float w = std::min(bbox1.x2_, bbox2.x2_) - x;
  float h = std::min(bbox1.y2_, bbox2.y2_) - y;
  return std::max(0.f, w) * std::max(0.f, h);
}

float BBoxArea(const xstream::BBox &bbox) {
  return std::max(0.f, bbox.x2_ - bbox.x1_) *
         std::max(0.f, bbox.y2_ - bbox.y1_);
}

float BBoxOverlapRatio(const xstream::BBox &bbox1, const xstream::BBox &bbox2) {
  float bbox1_area = BBoxArea(bbox1);
  float bbox2_area = BBoxArea(bbox2);
  float insect_area = IntersectionArea(bbox1, bbox2);
  return insect_area / (bbox1_area + bbox2_area - insect_area);
}

float BBoxInRatio(const xstream::BBox &bbox1, const xstream::BBox &bbox2) {
  float insect_area = IntersectionArea(bbox1, bbox2);
  float bbox1_area = BBoxArea(bbox1);
  return insect_area / bbox1_area;
}

float BBoxDistance(const xstream::BBox &bbox1, const xstream::BBox &bbox2) {
  float bbox1_cx = BBoxCenterX(bbox1);
  float bbox1_cy = BBoxCenterY(bbox1);
  float bbox2_cx = BBoxCenterX(bbox2);
  float bbox2_cy = BBoxCenterY(bbox2);
  float dis_x = bbox1_cx - bbox2_cx;
  float dis_y = bbox1_cy - bbox2_cy;
  return sqrt(dis_x * dis_x + dis_y * dis_y);
}

std::vector<int> ArgMax(const TrebleVec &mat) {
  int max_value = -1;
  std::vector<int> max_index(3, 0);
  for (size_t i = 0; i < mat.size(); ++i) {
    for (size_t j = 0; j < mat[i].size(); ++j) {
      for (size_t k = 0; k < mat[i][j].size(); ++k) {
        if (mat[i][j][k] > max_value) {
          max_value = mat[i][j][k];
          max_index[0] = i;
          max_index[1] = j;
          max_index[2] = k;
        }
      }
    }
  }

  return max_index;
}

std::vector<int> ArgMax(const DoubleVec &mat) {
  int max_value = -1;
  std::vector<int> max_index(2, 0);
  for (size_t i = 0; i < mat.size(); ++i) {
    for (size_t j = 0; j < mat[i].size(); ++j) {
      if (mat[i][j] > max_value) {
        max_value = mat[i][j];
        max_index[0] = i;
        max_index[1] = j;
      }
    }
  }

  return max_index;
}

std::vector<int> UnravelIndex(const int &index, const std::vector<int> &shape) {
  assert(shape.size() != 0);
  int index_ori = index;
  std::vector<int> index_list(shape.size(), 0);
  for (int i = shape.size() - 1; i >= 0; --i) {
    index_list[i] = index_ori % shape[i];
    index_ori /= shape[i];
  }
  return index_list;
}

float SigmoidFunction(float score) { return 1.f / (1.f + exp(-score)); }

float LinearFunction(float score, float min_value, float max_value) {
  score = (score - min_value) / (max_value - min_value);
  score = score < 0 ? 0 : score;
  score = score > 1 ? 1 : score;
  return score;
}

float MapFunction(float score, NormFunType fun_type, float min_value,
                  float max_value) {
  float res;
  switch (fun_type) {
    case NormFunType::Sigmoid:
      res = SigmoidFunction(score);
      break;
    case NormFunType::Linear:
    default:
      res = LinearFunction(score, min_value, max_value);
      break;
  }
  return res;
}

float DistanceToLine(const PointInt &point,
                     std::pair<PointInt, PointInt> &line) {
  auto dis_x = line.second.x_ - line.first.x_;
  auto dis_y = line.second.y_ - line.first.y_;
  float C = dis_x * line.first.y_ - dis_y * line.first.x_;
  return (dis_y * point.x_ - dis_x * point.y_ + C) /
         sqrt(dis_x * dis_x + dis_y * dis_y);
}

SizeCatgory GetVehicleSizeCatgory(const VehicleModelType &model_type) {
  SizeCatgory res_type;
  switch (model_type) {
    case VehicleModelType::Bus:
    case VehicleModelType::BigTruck:
      res_type = SizeCatgory::Large;
      break;
    case VehicleModelType::SUV:
    case VehicleModelType::MiniVan:
    case VehicleModelType::Lorry:
    case VehicleModelType::MPV:
    case VehicleModelType::Special_vehicle:
    case VehicleModelType::Minibus:
      res_type = SizeCatgory::Median;
      break;
    case VehicleModelType::Sedan_Car:
    case VehicleModelType::Scooter:
      res_type = SizeCatgory::Small;
      break;
    default:
      res_type = SizeCatgory::Unknown_Size;
      break;
  }
  return res_type;
}

}  // namespace vehicle_match_strategy
}  // namespace hobot
