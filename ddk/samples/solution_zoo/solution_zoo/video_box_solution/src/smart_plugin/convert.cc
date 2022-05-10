/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 04:05:52
 * @Version: v0.0.1
 * @Brief: implemenation of converter.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-02 06:26:01
 */
#include "smart_plugin/convert.h"

// #include <turbojpeg.h>

// #include <turbojpeg.h>

#include <string>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "smart_plugin/utils_box.h"
#include "video_box_common.h"
#include "vision/util.h"
#include "vision/vision_type.hpp"
#include "xproto/msg_type/vio_message.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"

namespace solution {
namespace video_box {

using solution::video_box::ImageFrameConversion;

// 默认值由smartplugin.cpp中设置

int Convertor::image_compress_quality = 50;
xstream::InputDataPtr Convertor::ConvertInput(const VioMessage *input) {
  xstream::InputDataPtr inputdata(new xstream::InputData());
  HOBOT_CHECK(input != nullptr && input->num_ > 0 && input->is_valid_uri_);

  // \todo need a better way to identify mono or semi cameras
  for (uint32_t image_index = 0; image_index < 1; ++image_index) {
    xstream::BaseDataPtr xstream_input_data;
    if (input->num_ > image_index) {
      std::shared_ptr<xstream::PyramidImageFrame> pym_img =
          input->image_[image_index];
      LOGI << "vio message, frame_id = " << pym_img->frame_id_;
#if 0
      for (uint32_t i = 0; i < DOWN_SCALE_MAX; ++i) {
        LOGD << "vio message, pym_level_" << i
             << ", width=" << pym_img->img_.down_scale[i].width
             << ", height=" << pym_img->img_.down_scale[i].height
             << ", stride=" << pym_img->img_.down_scale[i].step;
      }
#endif
      auto xstream_img = std::make_shared<xstream::ImageFrame>();
      xstream_img = pym_img;
      xstream_img->type_ = "PyramidImageFrame";
      LOGI << "Input Frame ID = " << xstream_img->frame_id_
           << ", Timestamp = " << xstream_img->time_stamp_;
      xstream_input_data = xstream::BaseDataPtr(xstream_img);

    } else {
      xstream_input_data = std::make_shared<xstream::BaseData>();
      xstream_input_data->state_ = xstream::DataState::INVALID;
    }

    if (image_index == uint32_t{0}) {
      if (input->num_ == 1) {
        xstream_input_data->name_ = "image";
      } else {
        xstream_input_data->name_ = "rgb_image";
      }
    } else {
      xstream_input_data->name_ = "nir_image";
    }
    LOGI << "input name:" << xstream_input_data->name_;
    inputdata->datas_.emplace_back(xstream_input_data);
  }

  return inputdata;
}

std::vector<solution::video_box::NoMotorVehicleInfo>
Convertor::ParseXstreamOutputToNoMotorInfo(
    const xstream::OutputDataPtr xstream_outputs) {
  std::vector<solution::video_box::NoMotorVehicleInfo> nomotor_infos;

  for (const auto &xstream_output : xstream_outputs->datas_) {
    if (xstream_output->name_ == "vehicle_un_bbox_list") {
      auto nomotor_boxes =
          std::dynamic_pointer_cast<xstream::BaseDataVector>(xstream_output);
      int nomotor_boxe_size = nomotor_boxes->datas_.size();

      for (int box_idx = 0; box_idx < nomotor_boxe_size; ++box_idx) {
        auto nomotor_box = std::dynamic_pointer_cast<
            solution::video_box::XStreamData<hobot::vision::BBox>>(
            nomotor_boxes->datas_[box_idx]);
        solution::video_box::NoMotorVehicleInfo no_motor_info;
        uint64_t track_id = nomotor_box->value.id;
        no_motor_info.track_id = track_id;
        no_motor_info.box.x1 = nomotor_box->value.x1;
        no_motor_info.box.y1 = nomotor_box->value.y1;
        no_motor_info.box.x2 = nomotor_box->value.x2;
        no_motor_info.box.y2 = nomotor_box->value.y2;
        no_motor_info.box.score = nomotor_box->value.score;

        nomotor_infos.push_back(no_motor_info);
      }
    }
  }

  return nomotor_infos;
}

std::vector<solution::video_box::PersonInfo>
Convertor::ParseXstreamOutputToPersonInfo(
    const xstream::OutputDataPtr xstream_outputs) {
  std::vector<solution::video_box::PersonInfo> person_infos;

  for (const auto &xstream_output : xstream_outputs->datas_) {
    if (xstream_output->name_ == "person_bbox_list") {
      auto person_boxes =
          std::dynamic_pointer_cast<xstream::BaseDataVector>(xstream_output);

      int person_boxes_size = person_boxes->datas_.size();

      for (int box_idx = 0; box_idx < person_boxes_size; ++box_idx) {
        auto person_box = std::dynamic_pointer_cast<
            solution::video_box::XStreamData<hobot::vision::BBox>>(
            person_boxes->datas_[box_idx]);

        solution::video_box::PersonInfo person_info;
        uint64_t track_id = person_box->value.id;
        person_info.track_id = track_id;
        person_info.box.x1 = person_box->value.x1;
        person_info.box.y1 = person_box->value.y1;
        person_info.box.x2 = person_box->value.x2;
        person_info.box.y2 = person_box->value.y2;
        person_info.box.score = person_box->value.score;

        person_infos.push_back(person_info);
      }
    }
  }

  return person_infos;
}

std::vector<solution::video_box::VehicleInfo>
Convertor::ParseXstreamOutputToVehicleInfo(
    const xstream::OutputDataPtr xstream_outputs) {
  std::vector<solution::video_box::VehicleInfo> vehicle_infos;
  for (const auto &xstream_output : xstream_outputs->datas_) {
    // 需要通过车辆框去找其他的信息
    if ((xstream_output->name_ == "vehicle_box_match") ||
        (xstream_output->name_ == "vehicle_bbox_list")) {
      auto xstream_vehicle_boxes =
          std::dynamic_pointer_cast<xstream::BaseDataVector>(xstream_output);
      std::size_t vehicle_box_size = xstream_vehicle_boxes->datas_.size();
      for (std::size_t vehicle_box_idx = 0; vehicle_box_idx < vehicle_box_size;
           ++vehicle_box_idx) {
        auto xstream_vehicle_box = std::dynamic_pointer_cast<
            solution::video_box::XStreamData<hobot::vision::BBox>>(
            xstream_vehicle_boxes->datas_[vehicle_box_idx]);
        int32_t track_id = xstream_vehicle_box->value.id;
        solution::video_box::VehicleInfo vehicle_info;
        vehicle_info.track_id = track_id;
        vehicle_info.box.x1 = xstream_vehicle_box->value.x1;
        vehicle_info.box.y1 = xstream_vehicle_box->value.y1;
        vehicle_info.box.x2 = xstream_vehicle_box->value.x2;
        vehicle_info.box.y2 = xstream_vehicle_box->value.y2;
        vehicle_info.box.score = xstream_vehicle_box->value.score;

        // 查找车的其他结构化信息
        for (const auto &xstream_sub_output : xstream_outputs->datas_) {
          if (xstream_sub_output->name_ == "plate_box_match" ||
              xstream_sub_output->name_ == "plate_box_filter") {
            // 车牌框
            auto xstream_plate_boxes =
                std::dynamic_pointer_cast<xstream::BaseDataVector>(
                    xstream_sub_output);
            for (std::size_t plate_box_idx = 0;
                 plate_box_idx < xstream_plate_boxes->datas_.size();
                 ++plate_box_idx) {  // 车牌的下标
              auto xstream_plate_box = std::dynamic_pointer_cast<
                  solution::video_box::XStreamData<hobot::vision::BBox>>(
                  xstream_plate_boxes->datas_[plate_box_idx]);
              if (xstream_plate_box->value.id == track_id) {
                LOGI << "find plate box " << track_id;
                solution::video_box::Plate &plate_info =
                    vehicle_info.plate_info;
                plate_info.box.x1 = xstream_plate_box->value.x1;
                plate_info.box.y1 = xstream_plate_box->value.y1;
                plate_info.box.x2 = xstream_plate_box->value.x2;
                plate_info.box.y2 = xstream_plate_box->value.y2;
                plate_info.box.score = xstream_plate_box->value.score;

                // 查找车牌的其他结构化信息
                for (const auto &xstream_sub_sub_output :
                     xstream_outputs->datas_) {
                  if (xstream_sub_sub_output->name_ == "plate_type_match") {
                    // 是否是双行牌照
                    auto xstream_plate_types =
                        std::dynamic_pointer_cast<xstream::BaseDataVector>(
                            xstream_sub_sub_output);
                    if (xstream_plate_types->datas_.size() !=
                        xstream_plate_boxes->datas_.size()) {
                      LOGE << "plate type and plate box size is not same"
                           << xstream_plate_types->datas_.size() << " "
                           << xstream_plate_boxes->datas_.size();
                      plate_info.is_double_plate = -1;
                    } else {
                      auto xstream_plate_type = std::dynamic_pointer_cast<
                          solution::video_box::XStreamData<
                              hobot::vision::Attribute<int>>>(
                          xstream_plate_types->datas_[plate_box_idx]);
                      plate_info.is_double_plate =
                          xstream_plate_type->state_ ==
                                  xstream::DataState::INVALID
                              ? -1
                              : xstream_plate_type->value.value;
                    }
                  } else if (xstream_sub_sub_output->name_ ==
                             "plate_num_vote") {
                    // 车牌号
                    auto xstream_plate_nums =
                        std::dynamic_pointer_cast<xstream::BaseDataVector>(
                            xstream_sub_sub_output);
                    if (xstream_plate_nums->datas_.size() !=
                        xstream_plate_boxes->datas_.size()) {
                      LOGE << "plate num and plate box size is not same"
                           << xstream_plate_nums->datas_.size() << " "
                           << xstream_plate_boxes->datas_.size();
                    } else {
                      auto xstream_plate_num = std::dynamic_pointer_cast<
                          solution::video_box::XStreamData<std::vector<int>>>(
                          xstream_plate_nums->datas_[plate_box_idx]);
                      if (!xstream_plate_num->value.empty() &&
                          xstream_plate_num->state_ !=
                              xstream::DataState::INVALID) {
                        plate_info.plate_num =
                            ConvertPlateNumber(xstream_plate_num->value);
                        plate_info.plate_idx = xstream_plate_num->value.front();
                      } else {
                        LOGI << "plate num is empty " << track_id;
                      }
                    }
                  } else if (xstream_sub_sub_output->name_ ==
                             "plate_color_vote") {
                    // 车牌颜色
                    auto xstream_plate_colors =
                        std::dynamic_pointer_cast<xstream::BaseDataVector>(
                            xstream_sub_sub_output);
                    if (xstream_plate_colors->datas_.size() !=
                        xstream_plate_boxes->datas_.size()) {
                      LOGE << "plate color and plate box size is not same "
                           << xstream_plate_colors->datas_.size() << " "
                           << xstream_plate_boxes->datas_.size();
                    } else {
                      auto xstream_plate_color = std::dynamic_pointer_cast<
                          solution::video_box::XStreamData<
                              hobot::vision::Attribute<int>>>(
                          xstream_plate_colors->datas_[plate_box_idx]);
                      if (xstream_plate_color != nullptr) {
                        plate_info.color = xstream_plate_color->value.value;
                      }
                    }
                  } else if (xstream_sub_sub_output->name_ ==
                             "plate_lmk_match") {
                    // 车牌关键点
                    auto xstream_plate_lmks =
                        std::dynamic_pointer_cast<xstream::BaseDataVector>(
                            xstream_sub_sub_output);
                    auto lmk_size = xstream_plate_lmks->datas_.size();
                    if (plate_box_idx < lmk_size) {
                      auto xstream_vehicle_lmk = std::dynamic_pointer_cast<
                          solution::video_box::XStreamData<
                              hobot::vision::Landmarks>>(
                          xstream_plate_lmks->datas_[plate_box_idx]);
                      plate_info.points.clear();
                      if (xstream_vehicle_lmk == nullptr) {
                        LOGI << "lmk num is empty " << track_id;
                      }
                      for (auto point : xstream_vehicle_lmk->value.values) {
                        solution::video_box::Point point_smart;
                        point_smart.x = point.x;
                        point_smart.y = point.y;
                        point_smart.score = point.score;
                        plate_info.points.emplace_back(point_smart);
                      }
                    } else {
                      plate_info.points = {};
                    }
                  }
                }
                break;
              }
            }
          } else if ((xstream_sub_output->name_ == "vehicle_lmk_match") ||
                     (xstream_sub_output->name_ == "vehicle_lmk")) {
            // 车辆关键点
            auto xstream_vehicle_lmks =
                std::dynamic_pointer_cast<xstream::BaseDataVector>(
                    xstream_sub_output);
            auto lmk_size = xstream_vehicle_lmks->datas_.size();
            if (vehicle_box_idx < lmk_size) {
              auto xstream_vehicle_lmk = std::dynamic_pointer_cast<
                  solution::video_box::XStreamData<hobot::vision::Landmarks>>(
                  xstream_vehicle_lmks->datas_[vehicle_box_idx]);
              vehicle_info.points.clear();
              for (auto point : xstream_vehicle_lmk->value.values) {
                solution::video_box::Point point_smart;
                point_smart.x = point.x;
                point_smart.y = point.y;
                point_smart.score = point.score;
                vehicle_info.points.emplace_back(point_smart);
              }
            } else {
              vehicle_info.points = {};
            }
          } else if (xstream_sub_output->name_ == "vehicle_type_vote") {
            // 车辆类型
            auto xstream_vehicle_types =
                std::dynamic_pointer_cast<xstream::BaseDataVector>(
                    xstream_sub_output);
            if (xstream_vehicle_types->datas_.size() != vehicle_box_size) {
              LOGE << "vehicle type and vehicle box size is not same "
                   << xstream_vehicle_types->datas_.size() << " "
                   << vehicle_box_size;
              vehicle_info.type = -1;
            } else {
              auto xstream_vehicle_type = std::dynamic_pointer_cast<
                  solution::video_box::XStreamData<int>>(
                  xstream_vehicle_types->datas_[vehicle_box_idx]);
              vehicle_info.type = xstream_vehicle_type->value;
            }
          } else if (xstream_sub_output->name_ == "vehicle_color_vote") {
            // 车辆颜色
            auto xstream_vehicle_colors =
                std::dynamic_pointer_cast<xstream::BaseDataVector>(
                    xstream_sub_output);
            if (xstream_vehicle_colors->datas_.size() != vehicle_box_size) {
              LOGE << "vehicle color and vehicle box size is not same "
                   << xstream_vehicle_colors->datas_.size() << " "
                   << vehicle_box_size;
              vehicle_info.color = -1;
            } else {
              auto xstream_vehicle_color = std::dynamic_pointer_cast<
                  solution::video_box::XStreamData<int>>(
                  xstream_vehicle_colors->datas_[vehicle_box_idx]);
              vehicle_info.color = xstream_vehicle_color->value;
            }
          }
        }
        vehicle_infos.push_back(vehicle_info);
      }
    }
  }
  return vehicle_infos;
}  // namespace iot

std::vector<uint64_t> Convertor::ParseXstreamOutputToLostTrack(
    const xstream::OutputDataPtr xstream_outputs) {
  std::vector<uint64_t> lost_track_ids;
  for (const auto &xstream_output : xstream_outputs->datas_) {
    if (xstream_output->name_ == "vehicle_disappeared_track_id_list") {
      // lost的track
      auto xstream_lost_tracks =
          std::dynamic_pointer_cast<xstream::BaseDataVector>(xstream_output);
      for (auto xstream_lost_track : xstream_lost_tracks->datas_) {
        auto track_id = std::dynamic_pointer_cast<
            solution::video_box::XStreamData<uint32_t>>(xstream_lost_track);
        lost_track_ids.push_back(track_id->value);
      }
    }
  }
  return lost_track_ids;
}

std::string Convertor::ConvertPlateNumber(
    const std::vector<int> &plate_num_array_model) {
  static std::vector<std::string> plate_num_map{
      " ",  "A",  "B",  "C",  "D",  "E",  "F",  "G",  "H",  "J",  "K",
      "L",  "M",  "N",  "P",  "Q",  "R",  "S",  "T",  "U",  "V",  "W",
      "X",  "Y",  "Z",  "0",  "1",  "2",  "3",  "4",  "5",  "6",  "7",
      "8",  "9",  "京", "津", "沪", "渝", "黑", "吉", "辽", "冀", "晋",
      "鲁", "豫", "陕", "甘", "青", "苏", "浙", "皖", "鄂", "湘", "闽",
      "赣", "川", "贵", "云", "粤", "琼", "蒙", "宁", "新", "桂", "藏",
      "学", "警", "领", "军", "使", "港", "挂", "澳"};
  std::string plate_num;
  for (auto plate_num_model : plate_num_array_model) {
    if (static_cast<std::size_t>(plate_num_model) >= plate_num_map.size()) {
      return "";
    }
    plate_num.append(plate_num_map[plate_num_model]);
  }
  return plate_num;
}

bool IsValidBaseData(xstream::BaseDataPtr data) {
  if (data.get() && data->state_ == xstream::DataState::VALID) {
    return true;
  }
  return false;
}

bool IsValidElement(const xstream::BaseDataVector *list, size_t index) {
  return list && list->datas_.size() > index &&
         IsValidBaseData(list->datas_[index]);
}

void ParseFaceSmartData(
    HorizonVisionSmartData *targets, const xstream::BaseDataVector *face_boxs,
    const xstream::BaseDataVector *head_box, const xstream::BaseDataVector *lmk,
    const xstream::BaseDataVector *nir_lmk, const xstream::BaseDataVector *pose,
    const xstream::BaseDataVector *age, const xstream::BaseDataVector *gender,
    const xstream::BaseDataVector *anti_spf_list,
    const xstream::BaseDataVector *rgb_anti_spf,
    const xstream::BaseDataVector *rgb_norm_rois,
    const xstream::BaseDataVector *nir_anti_spf,
    const xstream::BaseDataVector *nir_norm_rois,
    const xstream::BaseDataVector *nir_boxs,
    const solution::video_box::FaceQuality *face_quality,
    const xstream::BaseDataVector *breathing_mask_detect,
    const bool &with_debug_info) {
  if (!face_boxs) {
    return;
  }
  int current_valid_idx = 0;
  for (size_t i = 0; i < face_boxs->datas_.size(); ++i) {
    HorizonVisionFaceSmartData *face_data = nullptr;
    HorizonVisionAllocFaceSmartData(&face_data);
    solution::video_box::CreateVisionBox(face_boxs->datas_[i],
                                         &face_data->face_rect);
    if (IsValidElement(head_box, i)) {
      solution::video_box::CreateVisionBox(head_box->datas_[i],
                                           &face_data->head_rect);
    }
    if (IsValidElement(pose, i)) {
      solution::video_box::CreateVisionPose(pose->datas_[i],
                                            &face_data->pose3d);
    }
    face_data->landmarks = nullptr;
    if (IsValidElement(lmk, i)) {
      HorizonVisionAllocLandmarks(&face_data->landmarks);
      solution::video_box::CreateVisionLmk(lmk->datas_[i],
                                           face_data->landmarks);
    }
    if (IsValidElement(age, i)) {
      solution::video_box::CreateVisionAge(age->datas_[i], &face_data->age);
    }
    if (IsValidElement(gender, i)) {
      solution::video_box::CreateVisionAttribute(gender->datas_[i],
                                                 &face_data->gender);
    }
    if (IsValidElement(face_quality->blur, i)) {
      solution::video_box::CreateVisionAttribute(face_quality->blur->datas_[i],
                                                 &face_data->quality.blur);
    }
    if (IsValidElement(face_quality->brightness, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->brightness->datas_[i], &face_data->quality.brightness);
    }
    if (IsValidElement(face_quality->eye_abnormalities, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->eye_abnormalities->datas_[i],
          &face_data->quality.eye_abnormalities);
    }
    if (IsValidElement(face_quality->mouth_abnormal, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->mouth_abnormal->datas_[i],
          &face_data->quality.mouth_abnormal);
    }
    if (IsValidElement(face_quality->left_eye, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->left_eye->datas_[i], &face_data->quality.left_eye);
    }
    if (IsValidElement(face_quality->right_eye, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->right_eye->datas_[i], &face_data->quality.right_eye);
    }
    if (IsValidElement(face_quality->left_brow, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->left_brow->datas_[i], &face_data->quality.left_brow);
    }
    if (IsValidElement(face_quality->right_brow, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->right_brow->datas_[i], &face_data->quality.right_brow);
    }
    if (IsValidElement(face_quality->forehead, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->forehead->datas_[i], &face_data->quality.forehead);
    }
    if (IsValidElement(face_quality->left_cheek, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->left_cheek->datas_[i], &face_data->quality.left_cheek);
    }
    if (IsValidElement(face_quality->right_cheek, i)) {
      solution::video_box::CreateVisionAttribute(
          face_quality->right_cheek->datas_[i],
          &face_data->quality.right_cheek);
    }
    if (IsValidElement(face_quality->nose, i)) {
      solution::video_box::CreateVisionAttribute(face_quality->nose->datas_[i],
                                                 &face_data->quality.nose);
    }
    if (IsValidElement(face_quality->mouth, i)) {
      solution::video_box::CreateVisionAttribute(face_quality->mouth->datas_[i],
                                                 &face_data->quality.mouth);
    }
    if (IsValidElement(face_quality->jaw, i)) {
      solution::video_box::CreateVisionAttribute(face_quality->jaw->datas_[i],
                                                 &face_data->quality.jaw);
    }
    if (IsValidElement(breathing_mask_detect, i)) {
      solution::video_box::CreateVisionAttribute(
          breathing_mask_detect->datas_[i], &face_data->mask);
    }
    if (IsValidElement(anti_spf_list, i)) {
      solution::video_box::CreateVisionAttribute(anti_spf_list->datas_[i],
                                                 &face_data->anti_spoofing);
      LOGI << "liveness for current frame is "
           << face_data->anti_spoofing.value;
    } else {
      face_data->anti_spoofing.value = -1;
      face_data->anti_spoofing.score = 1;
    }
    face_data->track_id = face_data->face_rect.id;
    targets[current_valid_idx].face = face_data;
    if (!IsValidBaseData(face_boxs->datas_[i])) {
      // filtered
      targets[current_valid_idx].type = SMART_TYPE_FILTERED;
    } else {
      targets[current_valid_idx].type = SMART_TYPE_NORMAL;
    }
    if (with_debug_info) {
      auto extra_info = reinterpret_cast<HorizonVisionFaceExtraInfo *>(
          calloc(1, sizeof(HorizonVisionFaceExtraInfo)));
      //       rgb anti spf
      if (IsValidElement(rgb_anti_spf, i)) {
        solution::video_box::CreateVisionAttribute(rgb_anti_spf->datas_[i],
                                                   &extra_info->rgb_anti_spf_);
      } else {
        extra_info->rgb_anti_spf_.value = -1;
        extra_info->rgb_anti_spf_.score = 1;
      }
      // rgb norm box
      if (IsValidElement(rgb_norm_rois, i)) {
        solution::video_box::CreateVisionBox(rgb_norm_rois->datas_[i],
                                             &extra_info->rgb_norm_box_);
      }
      // rgb box
      if (IsValidElement(face_boxs, i)) {
        solution::video_box::CreateVisionBox(face_boxs->datas_[i],
                                             &extra_info->rgb_box_);
      }
      // get nir idx
      size_t nir_idx = 0;
      bool match = false;
      if (nir_boxs) {
        for (size_t nir_i = 0; nir_i < nir_boxs->datas_.size(); ++nir_i) {
          auto nir_box = dynamic_cast<solution::video_box::XRocBBox *>(
              nir_boxs->datas_[nir_i].get());
          if (nir_box && nir_box->value.id == (int32_t)face_data->track_id) {
            nir_idx = nir_i;
            match = true;
            break;
          }
        }
      }
#if 0
      if (IsValidElement(nir_lmk, nir_idx)) {
        HorizonVisionAllocLandmarks(&extra_info->nir_landmarks_);
        solution::video_box::CreateVisionLmk(nir_lmk->datas_[nir_idx],
                                               extra_info->nir_landmarks_);
      }
#endif

      // nir anti spf
      if (match && IsValidElement(nir_anti_spf, nir_idx)) {
        solution::video_box::CreateVisionAttribute(
            nir_anti_spf->datas_[nir_idx], &extra_info->nir_anti_spf_);
      } else {
        extra_info->nir_anti_spf_.value = -1;
        extra_info->nir_anti_spf_.score = 1;
      }
      // nir norm box
      if (match && IsValidElement(nir_norm_rois, nir_idx)) {
        solution::video_box::CreateVisionBox(nir_norm_rois->datas_[nir_idx],
                                             &extra_info->nir_norm_box_);
      }
      // nir box
      if (match && IsValidElement(nir_boxs, nir_idx)) {
        solution::video_box::CreateVisionBox(nir_boxs->datas_[nir_idx],
                                             &extra_info->nir_box_);
      }
      targets[current_valid_idx].face_extra = extra_info;
    }
    targets[current_valid_idx++].track_id = face_data->track_id;
  }
}

void ParseBodySmartData(HorizonVisionSmartData *targets,
                        const xstream::BaseDataVector *body_boxs,
                        const xstream::BaseDataVector *kps,
                        const xstream::BaseDataVector *mask) {
  if (!body_boxs) {
    LOGD << "no body boxes";
    return;
  }

  int current_valid_idx = 0;
  for (size_t i = 0; i < body_boxs->datas_.size(); ++i) {
    if (!IsValidBaseData(body_boxs->datas_[i])) {
      LOGI << "Has not body rects";
      continue;
    }
    HorizonVisionBodySmartData *body_data = nullptr;
    HorizonVisionAllocBodySmartData(&body_data);
    solution::video_box::CreateVisionBox(body_boxs->datas_[i],
                                         &body_data->body_rect);
    body_data->skeleton = nullptr;
    if (IsValidElement(kps, i)) {
      HorizonVisionAllocLandmarks(&body_data->skeleton);
      solution::video_box::CreateVisionLmk(kps->datas_[i], body_data->skeleton);
    }
    body_data->segmentation = nullptr;
    if (IsValidElement(mask, i)) {
      HorizonVisionAllocSegmentation(&body_data->segmentation);
      solution::video_box::CreateVisionSegmentation(mask->datas_[i],
                                                    body_data->segmentation);
    }
    body_data->track_id = body_data->body_rect.id;
    targets[current_valid_idx].body = body_data;
    targets[current_valid_idx++].track_id = body_data->track_id;
  }
}

void ParseDisappredSmartData(HorizonVisionSmartData *targets,
                             const xstream::BaseDataVector *ids) {
  if (!ids) {
    LOGD << "no disappeared ids";
    return;
  }

  for (size_t i = 0; i < ids->datas_.size(); ++i) {
    auto id = dynamic_cast<solution::video_box::XRocID *>(ids->datas_[i].get());
    HOBOT_CHECK(id);
    targets[i].track_id = id->value;
    LOGD << id->value << " disappeared!";
    targets[i].type = SMART_TYPE_DISAPPEARED;
  }
}

/**
 * @brief Convert xstream::OutputDataPtr to HorizonVisionSmartFrame
 * @param xroc_output
 * @return output HorizonVisionSmartFrame*
 */
HorizonVisionSmartFrame *Convertor::ConvertOutputToSmartFrame(
    const xstream::OutputDataPtr &xroc_output) {
  // at least image info
  HOBOT_CHECK(!xroc_output->datas_.empty()) << "Empty XRoc Output";
  HorizonVisionSmartFrame *smart_frame = nullptr;
  HorizonVisionAllocSmartFrame(&smart_frame);

  xstream::BaseDataVector *face_boxs = nullptr;
  xstream::BaseDataVector *head_box = nullptr;
  xstream::BaseDataVector *lmk = nullptr;
  xstream::BaseDataVector *nir_lmk = nullptr;
  xstream::BaseDataVector *pose = nullptr;
  xstream::BaseDataVector *age = nullptr;
  xstream::BaseDataVector *gender = nullptr;
  xstream::BaseDataVector *anti_spf_list = nullptr;
  xstream::BaseDataVector *body_boxs = nullptr;
  xstream::BaseDataVector *kps = nullptr;
  xstream::BaseDataVector *mask = nullptr;
  // xstream::BaseDataVector *snapshot = nullptr;
  // xstream::BaseDataVector *face_feature = nullptr;
  // xstream::BaseDataVector *breathing_mask = nullptr;
  // xstream::BaseDataVector *feature_uncertainty_list = nullptr;
  xstream::BaseDataVector *breathing_mask_detect = nullptr;
  //  xstream::BaseDataVector *face_quality_blur = nullptr;
  xstream::BaseDataVector *face_disappeared_track_id_list = nullptr;
  /// debug info
  xstream::BaseDataVector *rgb_anti_spf = nullptr;
  xstream::BaseDataVector *nir_anti_spf = nullptr;
  xstream::BaseDataVector *nir_boxs = nullptr;
  xstream::BaseDataVector *rgb_norm_rois = nullptr;
  xstream::BaseDataVector *nir_norm_rois = nullptr;
  solution::video_box::FaceQuality face_quality;
  for (const auto &output : xroc_output->datas_) {
    // todo filter xroc output with capability from encrypt chip
    LOGD << output->name_ << ", type is " << output->type_;
    // global anti_spf output
    if (output->name_ == "anti_spf") {
      anti_spf_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "age") {
      age = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_anti_spf") {
      rgb_anti_spf = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_anti_spf") {
      nir_anti_spf = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_norm_rois") {
      rgb_norm_rois = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_norm_rois") {
      nir_norm_rois = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_merge_bbox_list") {
      nir_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_anti_spf_calculated") {
      // auto tmp = dynamic_cast<xstream::BaseDataVector *>(output.get());
      // for (const auto &data : tmp->datas_) {
      // if (IsValidBaseData(data)) {
      //  auto xroc_attr = dynamic_cast<XRocAttribute *>(data.get());
      //  LOGI << "rgb_anti_spf_calculated output is "
      //       << xroc_attr->value.value;
      // }
      // }
    } else if (output->name_ == "gender") {
      gender = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "lmk" || output->name_ == "lmk_after_filter") {
      lmk = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_lmk") {
      nir_lmk = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "pose" ||
               output->name_ == "pose_after_filter") {
      pose = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_bbox_list_final"
               // face names in advances
               || output->name_ == "rgb_merge_bbox_list" ||
               output->name_ == "face_bbox_list" ||
               output->name_ == "rgb_merge_bbox_list_gathering" ||
               output->name_ == "face_bbox_list_gathering") {
      face_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_head_box") {
      head_box = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "body_box" ||
               output->name_ == "body_final_box") {
      body_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "kps" || output->name_ == "rgb_kps" ||
               output->name_ == "rgb_kps_gathering") {
      kps = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "mask") {
      mask = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "anti_spf_snapshot_list" ||
               output->name_ == "snap_list") {
      // snapshot = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "feature_list") {
      // face_feature = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "breathing_mask") {
      // breathing_mask = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "breathing_mask_detect") {
      breathing_mask_detect =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "feature_uncertainty_list") {
      // feature_uncertainty_list =
      //    dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "blur") {
      //      face_quality_blur =
      //          dynamic_cast<xstream::BaseDataVector *>(output.get());
      face_quality.blur = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "brightness") {
      face_quality.brightness =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "eye_abnormalities") {
      face_quality.eye_abnormalities =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "mouth_abnormal") {
      face_quality.mouth_abnormal =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "left_eye") {
      face_quality.left_eye =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "right_eye") {
      face_quality.right_eye =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "left_brow") {
      face_quality.left_brow =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "right_brow") {
      face_quality.right_brow =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "forehead") {
      face_quality.forehead =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "left_cheek") {
      face_quality.left_cheek =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "right_cheek") {
      face_quality.right_cheek =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nose") {
      face_quality.nose = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "mouth") {
      face_quality.mouth =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "jaw") {
      face_quality.jaw = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "disappeared_track_id") {
      face_disappeared_track_id_list =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else {
      LOGD << "No support data " << output->name_;
    }
  }
  // for mono face flow, rgb_anti_spf is equal to total anti_spf
  if (rgb_anti_spf == nullptr) {
    rgb_anti_spf = anti_spf_list;
  }

  auto face_target = face_boxs ? face_boxs->datas_.size() : 0;
  auto body_target = body_boxs ? body_boxs->datas_.size() : 0;
  auto disappeared_target = face_disappeared_track_id_list
                                ? face_disappeared_track_id_list->datas_.size()
                                : 0;
  auto total_targets = face_target + body_target + disappeared_target;
  LOGI << "Total target num = " << face_target << " + " << body_target << "+"
       << disappeared_target;
  HorizonVisionSmartData *targets = nullptr;
  if (total_targets > 0) {
    HorizonVisionAllocSmartData(&targets, total_targets);
    ParseFaceSmartData(targets, face_boxs, head_box, lmk, nir_lmk, pose, age,
                       gender, anti_spf_list, rgb_anti_spf, rgb_norm_rois,
                       nir_anti_spf, nir_norm_rois, nir_boxs, &face_quality,
                       breathing_mask_detect, false);
    ParseBodySmartData(&targets[face_target], body_boxs, kps, mask);
    ParseDisappredSmartData(&targets[face_target + body_target],
                            face_disappeared_track_id_list);
  }  // end of smart data

  HOBOT_CHECK(xroc_output->context_);
  smart_frame->image_num = 0;
  smart_frame->image_frame = nullptr;
#if 0
  smart_frame->time_stamp = input->image[0]->time_stamp;
  smart_frame->frame_id = input->image[0]->frame_id;
  smart_frame->time_stamp = input->image_[0]->time_stamp;
  smart_frame->frame_id = input->image_[0]->frame_id;
#endif
  smart_frame->smart_data_list = targets;
  smart_frame->smart_data_list_num = total_targets;
  // input will be used in ConstructVotData and move delete to global_config
  return smart_frame;
}

void Convertor::ConvertOutputToSmartFrame(
    const xstream::OutputDataPtr &xroc_output,
    HorizonVisionSmartFrame *smart_frame) {
  if (!smart_frame) return;
  // at least image info
  HOBOT_CHECK(!xroc_output->datas_.empty()) << "Empty XRoc Output";
  xstream::BaseDataVector *face_boxs = nullptr;
  xstream::BaseDataVector *head_box = nullptr;
  xstream::BaseDataVector *lmk = nullptr;
  xstream::BaseDataVector *nir_lmk = nullptr;
  xstream::BaseDataVector *pose = nullptr;
  xstream::BaseDataVector *age = nullptr;
  xstream::BaseDataVector *gender = nullptr;
  xstream::BaseDataVector *anti_spf_list = nullptr;
  xstream::BaseDataVector *body_boxs = nullptr;
  xstream::BaseDataVector *kps = nullptr;
  xstream::BaseDataVector *mask = nullptr;
  // xstream::BaseDataVector *snapshot = nullptr;
  // xstream::BaseDataVector *face_feature = nullptr;
  // xstream::BaseDataVector *breathing_mask = nullptr;
  // xstream::BaseDataVector *feature_uncertainty_list = nullptr;
  xstream::BaseDataVector *breathing_mask_detect = nullptr;
  //  xstream::BaseDataVector *face_quality_blur = nullptr;
  xstream::BaseDataVector *face_disappeared_track_id_list = nullptr;
  /// debug info
  xstream::BaseDataVector *rgb_anti_spf = nullptr;
  xstream::BaseDataVector *nir_anti_spf = nullptr;
  xstream::BaseDataVector *nir_boxs = nullptr;
  xstream::BaseDataVector *rgb_norm_rois = nullptr;
  xstream::BaseDataVector *nir_norm_rois = nullptr;
  solution::video_box::FaceQuality face_quality;
  for (const auto &output : xroc_output->datas_) {
    // todo filter xroc output with capability from encrypt chip
    LOGD << output->name_ << ", type is " << output->type_;
    // global anti_spf output
    if (output->name_ == "anti_spf") {
      anti_spf_list = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "age") {
      age = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_anti_spf") {
      rgb_anti_spf = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_anti_spf") {
      nir_anti_spf = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_norm_rois") {
      rgb_norm_rois = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_norm_rois") {
      nir_norm_rois = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_merge_bbox_list") {
      nir_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "rgb_anti_spf_calculated") {
      // auto tmp = dynamic_cast<xstream::BaseDataVector *>(output.get());
      // for (const auto &data : tmp->datas_) {
      // if (IsValidBaseData(data)) {
      //  auto xroc_attr = dynamic_cast<XRocAttribute *>(data.get());
      //  LOGI << "rgb_anti_spf_calculated output is "
      //       << xroc_attr->value.value;
      // }
      // }
    } else if (output->name_ == "gender") {
      gender = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "lmk" || output->name_ == "lmk_after_filter") {
      lmk = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nir_lmk") {
      nir_lmk = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "pose" ||
               output->name_ == "pose_after_filter") {
      pose = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "face_final_box"
               // face names in advances
               || output->name_ == "rgb_merge_bbox_list" ||
               output->name_ == "face_bbox_list" ||
               output->name_ == "rgb_merge_bbox_list_gathering" ||
               output->name_ == "face_bbox_list_gathering") {
      face_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "head_final_box") {
      head_box = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "body_box" ||
               output->name_ == "body_final_box") {
      body_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "kps" || output->name_ == "rgb_kps") {
      kps = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "mask") {
      mask = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "anti_spf_snapshot_list" ||
               output->name_ == "snap_list") {
      // snapshot = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "feature_list") {
      // face_feature = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "breathing_mask") {
      //  breathing_mask = dynamic_cast<xstream::BaseDataVector
      //  *>(output.get());
    } else if (output->name_ == "breathing_mask_detect") {
      breathing_mask_detect =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "feature_uncertainty_list") {
      // feature_uncertainty_list =
      //     dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "blur") {
      //      face_quality_blur =
      //          dynamic_cast<xstream::BaseDataVector *>(output.get());
      face_quality.blur = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "brightness") {
      face_quality.brightness =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "eye_abnormalities") {
      face_quality.eye_abnormalities =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "mouth_abnormal") {
      face_quality.mouth_abnormal =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "left_eye") {
      face_quality.left_eye =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "right_eye") {
      face_quality.right_eye =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "left_brow") {
      face_quality.left_brow =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "right_brow") {
      face_quality.right_brow =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "forehead") {
      face_quality.forehead =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "left_cheek") {
      face_quality.left_cheek =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "right_cheek") {
      face_quality.right_cheek =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "nose") {
      face_quality.nose = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "mouth") {
      face_quality.mouth =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "jaw") {
      face_quality.jaw = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "face_disappeared_track_id_list") {
      face_disappeared_track_id_list =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else {
      LOGD << "No support data " << output->name_;
    }
  }
  // for mono face flow, rgb_anti_spf is equal to total anti_spf
  if (rgb_anti_spf == nullptr) {
    rgb_anti_spf = anti_spf_list;
  }

  auto face_target = face_boxs ? face_boxs->datas_.size() : 0;
  auto body_target = body_boxs ? body_boxs->datas_.size() : 0;
  auto disappeared_target = face_disappeared_track_id_list
                                ? face_disappeared_track_id_list->datas_.size()
                                : 0;
  auto total_targets = face_target + body_target + disappeared_target;
  LOGD << "Total target num = " << face_target << " + " << body_target << "+"
       << disappeared_target;
  HorizonVisionSmartData *targets = nullptr;
  if (total_targets > 0) {
    HorizonVisionAllocSmartData(&targets, total_targets);
    ParseFaceSmartData(targets, face_boxs, head_box, lmk, nir_lmk, pose, age,
                       gender, anti_spf_list, rgb_anti_spf, rgb_norm_rois,
                       nir_anti_spf, nir_norm_rois, nir_boxs, &face_quality,
                       breathing_mask_detect, false);
    ParseBodySmartData(&targets[face_target], body_boxs, kps, mask);
    ParseDisappredSmartData(&targets[face_target + body_target],
                            face_disappeared_track_id_list);
  }  // end of smart data

  // HOBOT_CHECK(xroc_output->context_);
  smart_frame->image_num = 0;
  smart_frame->image_frame = nullptr;
#if 0
  smart_frame->time_stamp = input->image[0]->time_stamp;
  smart_frame->frame_id = input->image[0]->frame_id;
  smart_frame->time_stamp = input->image_[0]->time_stamp;
  smart_frame->frame_id = input->image_[0]->frame_id;
#endif
  smart_frame->smart_data_list = targets;
  smart_frame->smart_data_list_num = total_targets;
  return;
}

FloatArray CreateVisionFeature(xstream::BaseDataPtr base) {
  auto xstream_feature = dynamic_cast<XStreamFeature *>(base.get());
  HOBOT_CHECK(xstream_feature);

  FloatArray feature;
  feature.type_ = "feature";
  feature.score_ = xstream_feature->value.score;

#ifdef FEATURE_ENCRYPT
  HobotAESEncrypt(
      reinterpret_cast<char *>(xstream_feature->value.values.data()),
      reinterpret_cast<char *>(feature.values.data()), AES_FEATURE,
      sizeof(float) * xstream_feature->value.values.size());
#else
  feature.values_ = xstream_feature->value.values;
#endif

  return feature;
}

void ConvertCaptureInfo(SnapshotInfoXStreamBaseDataPtr one_capture,
                        xstream::BaseDataPtr one_feature,
                        FeatureFrameMessage &feature_contents) {
  FeatureInfo feature_info;
#if 0
  int64_t current_frame_id = -1;
  int64_t current_timestamp = -1;
  int32_t current_track_id = -1;

  auto &user_datas = one_capture->userdata;

  // pose and lmk and face_box
  HOBOT_CHECK(user_datas.size() >= 3)
      << " Capture data has only " << user_datas.size() << " userdata";

  LOGE << "Capture userdata size:" << user_datas.size() << std::endl;

  /**
   * slot0: pose_after_filter
   * slot2: face_bbox_list_after_filter
   * slot1: lmk_after_filter
   * slot3: anti_spf
   **/
  if (user_datas[0] && user_datas[0]->state_ == xstream::DataState::VALID) {
    auto passthrough = dynamic_cast<XStreamUserData *>(user_datas[0].get());
    current_frame_id = passthrough->value.frame_id_;
    current_timestamp = passthrough->value.timestamp_;
    current_track_id = passthrough->value.box_.id;
  } else {
    LOGE << "UserData[0] is not valid";
  }

  feature_info.frame_id_ = current_frame_id;
  feature_info.time_stamp_ = current_timestamp;
  feature_info.track_id_ = current_track_id;
#endif
  feature_info.track_id_ = one_capture->track_id;
  if (one_feature) {
    LOGI << "Convert Feature";
    feature_info.float_arrays_.emplace_back(CreateVisionFeature(one_feature));

    feature_contents.feature_infos_.emplace_back(feature_info);
  }
}

void ConvertFaceCapture(const xstream::BaseDataVector *face_capture,
                        const xstream::BaseDataVector *face_feature,
                        FeatureFrameMessage &feature_contents) {
  if (face_capture && face_feature) {
    auto capture_num = face_capture->datas_.size();
    LOGI << "Capture target num is " << capture_num;
    auto feature_num = face_feature->datas_.size();
    LOGI << "feature target num is " << feature_num;

    HOBOT_CHECK(feature_num == capture_num)
        << "face feature target size = " << face_feature->datas_.size()
        << " while face capture target size = " << capture_num;

    for (size_t i = 0; i < capture_num; ++i) {
      auto single_capture_face = face_capture->datas_[i];
      HOBOT_CHECK("BaseDataVector" == single_capture_face->type_)
          << " snap list data type is not BaseDataVector";
      auto single_face_images =
          dynamic_cast<xstream::BaseDataVector *>(single_capture_face.get());
      auto single_face_capture_num = single_face_images->datas_.size();
      HOBOT_CHECK(single_face_capture_num > 0) << " single face has 0 capture";
      xstream::BaseDataVector *single_feature_face = nullptr;
      if (face_feature && face_feature->datas_.size() > i) {
        single_feature_face = dynamic_cast<xstream::BaseDataVector *>(
            face_feature->datas_[i].get());
        HOBOT_CHECK("BaseDataVector" == single_feature_face->type_)
            << "feature data type is not BaseDataVector";
        auto single_face_feature_num = single_feature_face->datas_.size();
        HOBOT_CHECK(single_face_feature_num > 0)
            << " single face has 0 feature";
      }
      for (size_t j = 0; j < single_face_capture_num; ++j) {
        auto single_capture = std::dynamic_pointer_cast<XStreamSnapshotInfo>(
            single_face_images->datas_[j]);
#if 0
        LOGD << "Terminate, capture ID: " << single_capture->value->track_id
             << " TimeStamp: "
             << std::dynamic_pointer_cast<XRocUserData>(
                 single_capture->value->userdata[0])
                 ->value.timestamp_
             << " pointer use count: " << single_capture.use_count();
#endif
        xstream::BaseDataPtr single_feature = nullptr;
        if (single_feature_face) {
          single_feature = single_feature_face->datas_[j];
        }
        ConvertCaptureInfo(single_capture->value, single_feature,
                           feature_contents);
      }
    }
  } else {
    LOGI << "no capture or feature result";
  }
  LOGD << "Convert Feature done";
}

void Convertor::ConvertOutput(const xstream::OutputDataPtr &xstream_output,
                              FeatureFrameMessage &feature_frame_message) {
  // at least image info
  HOBOT_CHECK(!xstream_output || !xstream_output->datas_.empty())
      << "Empty XRoc Output";

  xstream::BaseDataVector *face_capture = nullptr;
  xstream::BaseDataVector *face_feature = nullptr;

  for (const auto &output : xstream_output->datas_) {
    if (output->name_ == "snap_list") {
      face_capture = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "The Frame has snap list";
    } else if (output->name_ == "face_feature") {
      face_feature = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "The Frame has feature list";
    }
  }

  ConvertFaceCapture(face_capture, face_feature, feature_frame_message);
}

//    std::string Convertor::JsonToUnStyledString(const Json::Value &jv) {
//      Json::StreamWriterBuilder writer_builder;
//      writer_builder["indentation"] = "";
//      return Json::writeString(writer_builder, jv);
//    }

}  // namespace video_box
}  // namespace solution
