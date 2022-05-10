/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved
 * @Author  jianglei.huang
 * @version 0.0.1
 * @date  2019.10.16
 */

#ifndef VEHICLE_MATCH_UTILITY_DATA_TYPE_HPP_
#define VEHICLE_MATCH_UTILITY_DATA_TYPE_HPP_

#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include <map>
#include <string>
#include "xstream/vision_type.h"
#include "xstream/xstream_data.h"

namespace hobot {
namespace vehicle_match_strategy {

enum class StatusType {
  kOk = 0,
  kInvalidParam = 1
};

struct Status {
  StatusType value = StatusType::kOk;
  /**
   * @brief Status Information
   */
  std::string info = "";
};

enum class NormFunType {
  Linear,
  Sigmoid
};

enum class ObjectType {
  Unknown,
  VEHICLE,
  PLATE
};

enum class VehicleModelType {
  Unknown_type = -1,
  BigTruck,         // 大货车 7.6m×2m
  Bus,              // 公交车-大货车 10m×2.5m
  Lorry,            // 小货车 6m×1.9m
  MPV,              // 商务车 4.69×1.82
  MiniVan,          // 面包车 4.13m×1.62m
  Minibus,          // 中巴车 4.85 2
  SUV,              // 越野车 4.6m×1.8m
  Scooter,          // 代步车 2.45 1.35
  Sedan_Car,        // 轿车 3.8m×1.6m
  Special_vehicle,  // 专用作业车  4.85x2
  Three_Wheeled_Truck,
  other,
};

enum class Color {
  Unknown_color = -1,
  White,        // 白
  Silver_gray,  // 银灰
  Black,        // 黑
  Red,          // 红
  Brown,        // 棕
  Blue,         // 蓝
  Yellow,       // 黄
  Purple,       // 紫
  Green,        // 绿
  Pink,         // 粉
  Ching,        // 青
  Golden,       // 金
};

typedef xstream::Point_<int> PointInt;
typedef std::shared_ptr<xstream::BBox> BBoxPtr;

struct Plate {
  BBoxPtr bbox = nullptr;
  int is_double = -1;
  std::string plate_num;
  xstream::Attribute_<int> plate_color;
  std::shared_ptr<xstream::Landmarks> plate_landmarks_ptr = nullptr;
  std::shared_ptr<xstream::BBox> plateBox = nullptr;
  std::shared_ptr<xstream::Attribute_<int>> plateType = nullptr;
};

typedef std::shared_ptr<Plate> PlatePtr;
typedef std::shared_ptr<xstream::ImageFrame> ImageFramePtr;

struct Vehicle {
  BBoxPtr bbox = nullptr;
  PlatePtr plate = nullptr;
  uint64_t time_stamp = 0;
  int model_type = static_cast<int>(VehicleModelType::Unknown_type);
  int color = static_cast<int>(Color::Unknown_color);
  // ID of lane
  int lane_id = 0;
  float occ_ratio = 0.f;
  float snap_score = 0.f;
  bool is_valid = false;
  float speed = 0;
  xstream::Point location;
  std::shared_ptr<xstream::Landmarks> vehicle_landmarks_ptr = nullptr;
  // The subimage of the vehicle.
  ImageFramePtr sub_img = nullptr;
  std::shared_ptr<xstream::BBox> vehicleBox = nullptr;
};

struct ImageMetaType {
  uint64_t time_stamp = 0;
  int img_width = 1920;
  int img_height = 1080;
};

enum class SizeCatgory {
  Unknown_Size = 0,
  Small = 1,
  Median = 2,
  Large = 3
};

typedef std::vector<Plate> PlateList;
typedef std::shared_ptr<PlateList> PlateListPtr;
typedef std::vector<Vehicle> VehicleList;
typedef std::shared_ptr<VehicleList> VehicleListPtr;

typedef std::vector<std::vector<std::vector<int>>> TrebleVec;
typedef std::vector<std::vector<int>> DoubleVec;

typedef std::tuple<int, int, float> Triple;

typedef std::map<int, std::vector<int>> IntToVecMap;

typedef std::vector<uint32_t> TrackIdList;
typedef std::shared_ptr<TrackIdList> TrackIdListPtr;

}   // namespace vehicle_match_strategy
}   // namespace hobot

#endif  // VEHICLE_MATCH_UTILITY_DATA_TYPE_HPP_
