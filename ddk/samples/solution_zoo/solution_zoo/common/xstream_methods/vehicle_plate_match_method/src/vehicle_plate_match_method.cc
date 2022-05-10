/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     BBoxFilter Method
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.23
 */

#include "vehicle_plate_match_method/vehicle_plate_match_method.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <random>
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"
#include "xstream/profiler.h"
#include "xstream/xstream_data.h"

namespace xstream {

int VehiclePlateMatchMethod::Init(const std::string &config_file_path) {
  LOGI << "VehiclePlateMatchMethod::Init " << config_file_path;
  matcher = hobot::vehicle_match_strategy::PairsMatchAPI::NewPairsMatchAPI(
      config_file_path);
  if (nullptr == matcher) return -1;
  std::ifstream config_stream(config_file_path);
  if (!config_stream.good()) {
    LOGF << "open config file failed.";
    return -1;
  }
  Json::Value cfg_jv;
  config_stream >> cfg_jv;
  using_fake_vehicle_lmks_ =
      cfg_jv["using_fake_vehicle_landmarks"].isBool()
          ? cfg_jv["using_fake_vehicle_landmarks"].asBool()
          : true;
  using_fake_plate_lmks_ = cfg_jv["using_fake_plate_landmarks"].isBool()
                               ? cfg_jv["using_fake_plate_landmarks"].asBool()
                               : true;
  using_fake_plate_color_ = cfg_jv["using_fake_plate_color"].isBool()
                                ? cfg_jv["using_fake_plate_color"].asBool()
                                : true;
  using_fake_plate_type_ = cfg_jv["using_fake_plate_type"].isBool()
                               ? cfg_jv["using_fake_plate_type"].asBool()
                               : true;
  return 0;
}

int VehiclePlateMatchMethod::UpdateParameter(InputParamPtr ptr) { return 0; }

InputParamPtr VehiclePlateMatchMethod::GetParameter() const {
  return nullptr;  // param;
}

std::vector<BaseDataPtr> VehiclePlateMatchMethod::DoProcess(
    const std::vector<BaseDataPtr> &input, const InputParamPtr &param) {
  LOGI << "VehiclePlateMatchMethod::DoProcess";
  std::vector<BaseDataPtr> output;

  {
    auto &in_batch_i = input;
    auto &out_batch_i = output;
    uint32_t input_size = in_batch_i.size();

    // inputs order: vehicle_bbox, plate_bbox, vehicle_lmks, plate_lmks,
    // plate_type, plate_color
    int input_idx = 0;
    auto in_vehicle_rects =
        std::dynamic_pointer_cast<BaseDataVector>(input[input_idx++]);
    auto in_plate_rects =
        std::dynamic_pointer_cast<BaseDataVector>(in_batch_i[input_idx++]);
    auto in_vehicle_lmk = std::make_shared<BaseDataVector>();
    auto in_plate_lmk = std::make_shared<BaseDataVector>();
    auto in_plate_type = std::make_shared<BaseDataVector>();
    auto in_plate_color = std::make_shared<BaseDataVector>();
    if (!using_fake_vehicle_lmks_) {
      HOBOT_CHECK(input_idx < in_batch_i.size())
          << input_idx << " out of range, input size: " << input_size;
      in_vehicle_lmk =
          std::dynamic_pointer_cast<BaseDataVector>(in_batch_i[input_idx++]);
    }
    if (!using_fake_plate_lmks_) {
      HOBOT_CHECK(input_idx < in_batch_i.size())
          << input_idx << " out of range, input size: " << input_size;
      in_plate_lmk =
          std::dynamic_pointer_cast<BaseDataVector>(in_batch_i[input_idx++]);
    }
    if (!using_fake_plate_type_) {
      HOBOT_CHECK(input_idx < in_batch_i.size())
          << input_idx << " out of range, input size: " << input_size;
      in_plate_type =
          std::dynamic_pointer_cast<BaseDataVector>(in_batch_i[input_idx++]);
    }
    if (!using_fake_plate_color_) {
      HOBOT_CHECK(input_idx < in_batch_i.size())
          << input_idx << " out of range, input size: " << input_size;
      in_plate_color =
          std::dynamic_pointer_cast<BaseDataVector>(in_batch_i[input_idx]);
    }

    assert("BaseDataVector" == in_vehicle_rects->type_);
    assert("BaseDataVector" == in_plate_rects->type_);
    LOGI << "match method input vehicle size : "
         << in_vehicle_rects->datas_.size();

    auto out_vehicle_rects = std::make_shared<BaseDataVector>();
    auto out_vehicle_lmk = std::make_shared<BaseDataVector>();
    auto out_plate_rects = std::make_shared<BaseDataVector>();
    auto out_plate_lmk = std::make_shared<BaseDataVector>();
    auto out_plate_type = std::make_shared<BaseDataVector>();
    auto out_plate_color = std::make_shared<BaseDataVector>();
    out_batch_i.push_back(
        std::dynamic_pointer_cast<BaseData>(out_vehicle_rects));
    out_batch_i.push_back(std::dynamic_pointer_cast<BaseData>(out_vehicle_lmk));
    out_batch_i.push_back(std::dynamic_pointer_cast<BaseData>(out_plate_rects));
    out_batch_i.push_back(std::dynamic_pointer_cast<BaseData>(out_plate_lmk));
    out_batch_i.push_back(std::dynamic_pointer_cast<BaseData>(out_plate_type));
    out_batch_i.push_back(std::dynamic_pointer_cast<BaseData>(out_plate_color));

    hobot::vehicle_match_strategy::VehicleListPtr vehicle_list =
        std::make_shared<hobot::vehicle_match_strategy::VehicleList>();
    hobot::vehicle_match_strategy::PlateListPtr plate_list =
        std::make_shared<hobot::vehicle_match_strategy::PlateList>();

    std::size_t vehicle_data_size = in_vehicle_rects->datas_.size();
    for (std::size_t vehicle_idx = 0; vehicle_idx < vehicle_data_size;
         ++vehicle_idx) {
      hobot::vehicle_match_strategy::Vehicle veh_tmp;
      auto bbox = std::dynamic_pointer_cast<xstream::BBox>(
          in_vehicle_rects->datas_[vehicle_idx]);
      if (!bbox) {
        LOGW << "bbox is null";
        continue;
      }
      hobot::vehicle_match_strategy::BBoxPtr box =
          std::make_shared<xstream::BBox>(bbox->x1_, bbox->y1_, bbox->x2_,
                                          bbox->y2_);
      if (in_vehicle_lmk && !in_vehicle_lmk->datas_.empty() &&
          vehicle_idx < in_vehicle_lmk->datas_.size()) {
        auto vehicle_lmk = std::dynamic_pointer_cast<xstream::Landmarks>(
            in_vehicle_lmk->datas_[vehicle_idx]);
        veh_tmp.vehicle_landmarks_ptr = vehicle_lmk;
      } else {
        LOGD << "no matching vehicle lmks";
      }

      veh_tmp.bbox = box;
      veh_tmp.bbox->id_ = bbox->id_;
      veh_tmp.vehicleBox = bbox;
      vehicle_list->emplace_back(std::move(veh_tmp));
    }

    for (std::size_t plate_idx = 0; plate_idx < in_plate_rects->datas_.size();
         ++plate_idx) {
      hobot::vehicle_match_strategy::Plate plate_tmp;
      auto bbox = std::dynamic_pointer_cast<xstream::BBox>(
          in_plate_rects->datas_[plate_idx]);
      if (in_plate_type && !in_plate_type->datas_.empty() &&
          plate_idx < in_plate_type->datas_.size()) {
        auto plate_type = std::dynamic_pointer_cast<xstream::Attribute_<int>>(
            in_plate_type->datas_[plate_idx]);
        plate_tmp.is_double = plate_type->value_;
        plate_tmp.plateType = plate_type;
        LOGD << "got input plate row: " << plate_type->value_;
      } else {
        LOGW << "no matching plate type";
      }

      if (in_plate_color && !in_plate_color->datas_.empty() &&
          plate_idx < in_plate_color->datas_.size()) {
        auto plate_color = std::dynamic_pointer_cast<xstream::Attribute_<int>>(
            in_plate_color->datas_[plate_idx]);
        LOGD << "got input plate color: " << plate_color->value_;
        plate_tmp.plate_color = *plate_color;
      } else {
        LOGW << "no matching plate color";
      }

      if (in_plate_lmk && !in_plate_lmk->datas_.empty() &&
          plate_idx < in_plate_lmk->datas_.size()) {
        auto plate_lmk = std::dynamic_pointer_cast<xstream::Landmarks>(
            in_plate_lmk->datas_[plate_idx]);
        plate_tmp.plate_landmarks_ptr = plate_lmk;
      } else {
        LOGD << "no match plate lmks";
      }

      if (!bbox) {
        LOGW << "bbox or plate_type is null";
        continue;
      }

      hobot::vehicle_match_strategy::BBoxPtr box =
          std::make_shared<xstream::BBox>(bbox->x1_, bbox->y1_, bbox->x2_,
                                          bbox->y2_);
      plate_tmp.bbox = box;
      plate_tmp.bbox->id_ = bbox->id_;
      plate_tmp.plateBox = bbox;
      plate_list->emplace_back(std::move(plate_tmp));
    }
    LOGI << "matcher input vehicle list size : " << vehicle_list->size();
    LOGI << "matcher input plate list size : " << plate_list->size();
    matcher->Process(vehicle_list, plate_list);
    for (const auto &itr : *vehicle_list) {
      out_vehicle_rects->datas_.push_back(itr.vehicleBox);
      out_vehicle_lmk->datas_.push_back(itr.vehicle_landmarks_ptr);
      if (itr.plate != nullptr) {
        itr.plate->plateBox->id_ = itr.vehicleBox->id_;
        out_plate_rects->datas_.push_back(itr.plate->plateBox);
        out_plate_lmk->datas_.push_back(itr.plate->plate_landmarks_ptr);
        out_plate_type->datas_.push_back(itr.plate->plateType);
        out_plate_color->datas_.push_back(
            std::make_shared<xstream::Attribute_<int>>(itr.plate->plate_color));
      }
    }
  }
  return output;
}

void VehiclePlateMatchMethod::Finalize() { LOGI << "VehicleMatch::Finalize"; }

std::string VehiclePlateMatchMethod::GetVersion() const { return "v0.0.1"; }

}  // namespace xstream
