/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-01 20:38:52
 * @Version: v0.0.1
 * @Brief: custom_smart_message impl
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 05:04:11
 */

#include "smart_plugin/message/custom_smart_message.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "attribute_convert.h"
#include "hobotlog/hobotlog.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "smart_plugin/gesture_threshold.h"
#include "xproto/message/flowmsg.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/msg_type/vio_message.h"
#include "xstream/xstream_world.h"

namespace xproto {

using xstream::BBox;
using xstream::Segmentation;

using xproto::message::VioMessage;
using xproto::message::CustomSmartMessage;
using ImageFramePtr = std::shared_ptr<xstream::ImageFrame>;

using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_MESSAGE)

struct target_key {
  std::string category;
  int id;
  target_key(std::string category, int id) {
    this->category = category;
    this->id = id;
  }
};

struct cmp_key {
  bool operator()(const target_key &a, const target_key &b) {
    if (a.category == b.category) {
      return a.id < b.id;
    }
    return a.category < b.category;
  }
};

std::mutex CustomSmartMessage::static_attr_mutex_;

int dist_calibration_w = 0;
float dist_fit_factor = 0.0;
float dist_fit_impower = 0.0;
bool dist_smooth = true;

void CustomSmartMessage::Serialize_Print(Json::Value &root) {
  // not implement
  return;
}

void CustomSmartMessage::Serialize_Dump_Result() {
  // not implement
  return;
}

std::string CustomSmartMessage::Serialize() {
  // not implement
  return "";
}

std::string CustomSmartMessage::Serialize(int ori_w, int ori_h, int dst_w,
                                          int dst_h) {
  HOBOT_CHECK(ori_w > 0 && ori_h > 0 && dst_w > 0 && dst_h > 0)
      << "Serialize param error";
  float x_ratio = 1.0 * dst_w / ori_w;
  float y_ratio = 1.0 * dst_h / ori_h;
  // serialize smart message using defined smart protobuf.
  std::string proto_str;
  x3::FrameMessage proto_frame_message;
  proto_frame_message.set_timestamp_(time_stamp_);
  auto smart_msg = proto_frame_message.mutable_smart_msg_();
  smart_msg->set_timestamp_(time_stamp_);
  smart_msg->set_error_code_(0);
  // add fps to output
  auto static_msg = proto_frame_message.mutable_statistics_msg_();
  auto fps_attrs = static_msg->add_attributes_();
  fps_attrs->set_type_("fps");
  fps_attrs->set_value_(frame_fps_);
  fps_attrs->set_value_string_(std::to_string(frame_fps_));
  // user-defined output parsing declaration.
  xstream::BaseDataVector *face_boxes = nullptr;
  xstream::BaseDataVector *face_lmks = nullptr;
  xstream::BaseDataVector *hand_lmks = nullptr;
  xstream::BaseDataVector *lmks = nullptr;
  xstream::BaseDataVector *mask = nullptr;
  xstream::BaseDataVector *features = nullptr;
  auto name_prefix = [](const std::string name,
                        const char separator) -> std::string {
    auto pos = name.find(separator);
    if (pos == std::string::npos)
      return "";

    return name.substr(0, pos);
  };

  auto name_postfix = [](const std::string name,
                         const char separator) -> std::string {
    auto pos = name.rfind(separator);
    if (pos == std::string::npos)
      return "";

    return name.substr(pos + 1);
  };

  std::vector<std::shared_ptr<xstream::BBox>> face_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> head_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> body_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> hand_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> vehicle_box_list;
  std::vector<std::shared_ptr<xstream::BBox>> plate_box_list;
  std::map<target_key, x3::Target *, cmp_key> smart_target;
  for (const auto &output : smart_result->datas_) {
    auto old_output_name = output->name_;
    output->name_ = name_prefix(output->name_, '|');
    if (output->name_.empty()) {
      output->name_ = old_output_name;
    }
    LOGD << "output name: " << output->name_
         << ", real output name: " << output->name_;
    auto prefix = name_prefix(output->name_, '_');
    auto postfix = name_postfix(output->name_, '_');
    if (output->name_ == "face_bbox_list" || output->name_ == "head_box" ||
        output->name_ == "body_box" || postfix == "box") {
      face_boxes = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "box type: " << output->name_
           << ", box size: " << face_boxes->datas_.size();
      bool track_id_valid = true;
      if (face_boxes->datas_.size() > 1) {
        track_id_valid = false;
        for (size_t i = 0; i < face_boxes->datas_.size(); ++i) {
          auto face_box =
              std::static_pointer_cast<xstream::BBox>(face_boxes->datas_[i]);
          if (face_box->id_ != 0) {
            track_id_valid = true;
            break;
          }
        }
      }
      for (size_t i = 0; i < face_boxes->datas_.size(); ++i) {
        auto face_box =
            std::static_pointer_cast<xstream::BBox>(face_boxes->datas_[i]);
        if (!track_id_valid) {
          face_box->id_ = i + 1;
        }
        if (prefix == "hand") {
          face_box->specific_type_ = "hand";
        } else if (prefix == "vehicle") {
          face_box->specific_type_ = "vehicle";
        } else if (prefix == "nonmotor") {
          face_box->specific_type_ = "nonmotor";
        } else if (prefix == "plate") {
          face_box->specific_type_ = "plate";
        } else {
          face_box->specific_type_ = "person";
        }
        LOGD << output->name_ << " id: " << face_box->id_
             << " x1: " << face_box->x1_ << " y1: " << face_box->y1_
             << " x2: " << face_box->x2_ << " y2: " << face_box->y2_;
        if (prefix == "face") {
          face_box_list.push_back(face_box);
        } else if (prefix == "head") {
          head_box_list.push_back(face_box);
        } else if (prefix == "body") {
          body_box_list.push_back(face_box);
        } else if (prefix == "hand") {
          hand_box_list.push_back(face_box);
        } else if (prefix == "vehicle") {
          vehicle_box_list.push_back(face_box);
        } else if (prefix == "plate") {
          plate_box_list.push_back(face_box);
        } else if (prefix == "nonmotor" || prefix == "pedestrian") {
          // avoid error log
        } else {
          LOGE << "unsupport box name: " << output->name_;
        }
        target_key face_key(face_box->specific_type_, face_box->id_);
        if (face_box->id_ != -1) {
          if (smart_target.find(face_key) ==
              smart_target.end()) {  // 新track_id
            auto target = smart_msg->add_targets_();
            target->set_track_id_(face_box->id_);
            if (prefix == "hand") {
              target->set_type_("hand");
            } else if (prefix == "vehicle") {
              target->set_type_("vehicle");
            } else if (prefix == "nonmotor") {
              target->set_type_("nonmotor");
            } else if (prefix == "plate") {
              target->set_type_("plate");
            } else {
              target->set_type_("person");
            }
            smart_target[face_key] = target;
          }

          if (prefix == "face" && dist_calibration_w > 0) {
            // cal distance with face box width
            int face_box_width = face_box->x2_ - face_box->x1_;
            if (dist_calibration_w != ori_w) {
              // cam input is not 4K, convert width
              face_box_width = face_box_width * dist_calibration_w / ori_w;
            }
            int dist =
                dist_fit_factor * (pow(face_box_width, dist_fit_impower));
            // 四舍五入法平滑
            if (dist_smooth) {
              dist = round(static_cast<float>(dist) / 10.0) * 10;
            }
            auto attrs = smart_target[face_key]->add_attributes_();
            attrs->set_type_("dist");
            // todo
            // distance is calculated from face box,
            // use face box score as distance score
            // calculate distance score from face pose is more accurate
            attrs->set_score_(face_box->score_);
            attrs->set_value_(dist);
            attrs->set_value_string_(std::to_string(dist));
          }

          auto proto_box = smart_target[face_key]->add_boxes_();
          proto_box->set_type_(prefix);  // "face", "head", "body", "vehicle",
                                         // "nonmotor", "plate"
          auto point1 = proto_box->mutable_top_left_();
          point1->set_x_(face_box->x1_ * x_ratio);
          point1->set_y_(face_box->y1_ * y_ratio);
          point1->set_score_(face_box->score_);
          auto point2 = proto_box->mutable_bottom_right_();
          point2->set_x_(face_box->x2_ * x_ratio);
          point2->set_y_(face_box->y2_ * y_ratio);
          point2->set_score_(face_box->score_);

          // body_box在前
          if (prefix == "body" && smart_target[face_key]->boxes__size() > 1) {
            auto body_box_index = smart_target[face_key]->boxes__size() - 1;
            auto body_box =
                smart_target[face_key]->mutable_boxes_(body_box_index);
            auto first_box = smart_target[face_key]->mutable_boxes_(0);
            first_box->Swap(body_box);
          }
        }
      }
    }
    if (output->name_ == "kps" || output->name_ == "lowpassfilter_body_kps") {
      lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "kps size: " << lmks->datas_.size();
      if (lmks->datas_.size() != body_box_list.size()) {
        LOGE << "kps size: " << lmks->datas_.size()
             << ", body_box size: " << body_box_list.size();
      }
      for (size_t i = 0; i < lmks->datas_.size(); ++i) {
        auto lmk = std::static_pointer_cast<
            xstream::Landmarks>(lmks->datas_[i]);
        // 查找对应的track_id
        if (body_box_list[i]->id_ == -1) {
          continue;
        }
        target_key body_key(body_box_list[i]->specific_type_,
                            body_box_list[i]->id_);
        if (smart_target.find(body_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          auto target = smart_target[body_key];
          auto proto_points = target->add_points_();
          proto_points->set_type_("body_landmarks");
          for (size_t i = 0; i < lmk->values_.size(); ++i) {
            auto point = proto_points->add_points_();
            point->set_x_(lmk->values_[i].x_ * x_ratio);
            point->set_y_(lmk->values_[i].y_ * y_ratio);
            point->set_score_(lmk->values_[i].score_);
            LOGD << "x: " << std::round(lmk->values_[i].x_)
                 << " y: " << std::round(lmk->values_[i].y_)
                 << " score: " << lmk->values_[i].score_ << "\n";
          }
        }
      }
    }
    static bool check_env = false;
    static bool need_check_mask_time = false;
    static bool need_dump_matting = false;
    if (!check_env) {
      auto check_mask_time = getenv("check_mask_time");
      if (check_mask_time && !strcmp(check_mask_time, "ON")) {
        need_check_mask_time = true;
      }
      auto dump_matting = getenv("dump_matting");
      if (dump_matting && !strcmp(dump_matting, "ON")) {
        need_dump_matting = true;
      }
      check_env = true;
    }

    auto mask_start_time = std::chrono::system_clock::now();
    if (output->name_ == "mask") {
      mask = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "mask size: " << mask->datas_.size();
      if (mask->datas_.size() != body_box_list.size()) {
        LOGE << "mask size: " << mask->datas_.size()
             << ", body_box size: " << body_box_list.size();
      }
      for (size_t i = 0; i < mask->datas_.size(); ++i) {
        auto one_mask_start_time = std::chrono::system_clock::now();
        auto one_mask = std::static_pointer_cast<
            xstream::Segmentation>(mask->datas_[i]);
        if (one_mask->state_ != xstream::DataState::VALID) {
          continue;
        }
        // 查找对应的track_id
        if (body_box_list[i]->id_ == -1) {
          continue;
        }
        target_key body_key(body_box_list[i]->specific_type_,
                            body_box_list[i]->id_);
        if (smart_target.find(body_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
          continue;
        } else {
          auto body_box = body_box_list[i];
          int x1 = body_box->x1_;
          int y1 = body_box->y1_;
          int x2 = body_box->x2_;
          int y2 = body_box->y2_;
          auto mask = one_mask;
          int h_w = sqrt(mask->values_.size());
          cv::Mat mask_mat(h_w, h_w, CV_32F);

          for (int h = 0; h < h_w; ++h) {
            float *ptr = mask_mat.ptr<float>(h);
            for (int w = 0; w < h_w; ++w) {
              *(ptr + w) = (mask->values_)[h * h_w + w];
            }
          }
          float max_ratio = 1;
          int width = x2 - x1;
          int height = y2 - y1;

          float w_ratio = static_cast<float>(width) / h_w;
          float h_ratio = static_cast<float>(height) / h_w;
          if (w_ratio >= 4 && h_ratio >= 4) {
            max_ratio = w_ratio < h_ratio ? w_ratio : h_ratio;
            if (max_ratio >= 4) {
              max_ratio = 4;
            }
            width = width / max_ratio;
            height = height / max_ratio;
          }
          cv::resize(mask_mat, mask_mat, cv::Size(width, height));
          cv::Mat mask_gray(height, width, CV_8UC1);
          mask_gray.setTo(0);
          std::vector<std::vector<cv::Point>> contours;

          for (int h = 0; h < height; ++h) {
            uchar *p_gray = mask_gray.ptr<uchar>(h);
            const float *p_mask = mask_mat.ptr<float>(h);
            for (int w = 0; w < width; ++w) {
              if (p_mask[w] > 0) {
                // 这个点在人体内
                p_gray[w] = 1;
              } else {
              }
            }
          }
          mask_mat.release();
          cv::findContours(mask_gray, contours, cv::noArray(), cv::RETR_CCOMP,
                           cv::CHAIN_APPROX_NONE);

          mask_gray.release();
          auto target = smart_target[body_key];
          auto Points = target->add_points_();
          Points->set_type_("mask");
          for (size_t i = 0; i < contours.size(); i++) {
            auto one_line = contours[i];
            for (size_t j = 0; j < one_line.size(); j += 4) {
              auto point = Points->add_points_();
              point->set_x_((contours[i][j].x * max_ratio + x1) * x_ratio);
              point->set_y_((contours[i][j].y * max_ratio + y1) * y_ratio);
              point->set_score_(0);
            }
          }
          contours.clear();
          std::vector<std::vector<cv::Point>>(contours).swap(contours);
        }
        auto one_mask_end_time = std::chrono::system_clock::now();
        if (need_check_mask_time) {
          auto duration_time =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  one_mask_end_time - one_mask_start_time);
          LOGW << "process one mask used:  " << duration_time.count() << " ms";
        }
      }
    }
    auto mask_end_time = std::chrono::system_clock::now();
    if (need_check_mask_time) {
      auto duration_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              mask_end_time - mask_start_time);
      LOGW << "process one frame, mask total used:  " << duration_time.count()
           << " ms";
    }

    if (output->name_ == "age") {
      auto ages = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "age size: " << ages->datas_.size();
      if (ages->datas_.size() != face_box_list.size()) {
        LOGE << "ages size: " << ages->datas_.size()
             << ", face_box size: " << face_box_list.size();
      }
      for (size_t i = 0; i < ages->datas_.size(); ++i) {
        auto age =
            std::static_pointer_cast<xstream::Age>(
                ages->datas_[i]);
        // 查找对应的track_id
        if (face_box_list[i]->id_ == -1) {
          continue;
        }
        target_key face_key(face_box_list[i]->specific_type_,
                            face_box_list[i]->id_);
        if (smart_target.find(face_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          if (age->state_ != xstream::DataState::VALID) {
            LOGE << "-1 -1 -1";
            continue;
          }
          auto target = smart_target[face_key];
          auto attrs = target->add_attributes_();
          attrs->set_type_("age");
          attrs->set_value_((age->min_ + age->max_) / 2);
          attrs->set_score_(age->score_);
          attrs->set_value_string_(std::to_string((age->min_ + age->max_) / 2));
          LOGD << " " << age->min_ << " " << age->max_;
        }
      }
    }

    if (output->name_ == "gender") {
      auto genders = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "gender size: " << genders->datas_.size();
      if (genders->datas_.size() != face_box_list.size()) {
        LOGE << "genders size: " << genders->datas_.size()
             << ", face_box size: " << face_box_list.size();
      }
      for (size_t i = 0; i < genders->datas_.size(); ++i) {
        auto gender = std::static_pointer_cast<
            xstream::Gender>(genders->datas_[i]);
        // 查找对应的track_id
        if (face_box_list[i]->id_ == -1) {
          continue;
        }
        target_key face_key(face_box_list[i]->specific_type_,
                            face_box_list[i]->id_);
        if (smart_target.find(face_key) ==
              smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          if (genders->state_ != xstream::DataState::VALID) {
            LOGE << "-1";
            continue;
          }
          auto target = smart_target[face_key];
          auto attrs = target->add_attributes_();
          attrs->set_type_("gender");
          attrs->set_value_(gender->value_);
          attrs->set_score_(gender->score_);
          auto gender_des = AttributeConvert::Instance().GetAttrDes(
            "gender", gender->value_);
          attrs->set_value_string_(gender_des);
          LOGD << " " << gender->value_;
        }
      }
    }

    if (output->name_ == "vehicle_color_vote") {
      auto colors = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "vehicle colors size: " << colors->datas_.size();
      if (colors->datas_.size() != vehicle_box_list.size()) {
        LOGE << "vehicle colors size: " << colors->datas_.size()
             << ", vehicle_box size: " << vehicle_box_list.size();
      }
      for (size_t i = 0; i < colors->datas_.size(); ++i) {
        auto color = std::static_pointer_cast<xstream::Attribute_<int>>(
            colors->datas_[i]);
        // 查找对应的track_id
        if (vehicle_box_list[i]->id_ == -1) {
          continue;
        }
        target_key vehicle_key(vehicle_box_list[i]->specific_type_,
                               vehicle_box_list[i]->id_);
        if (smart_target.find(vehicle_key) == smart_target.end()) {
          LOGE << "Not found the track_id target: " << vehicle_key.id;
        } else {
          if (color->state_ != xstream::DataState::VALID) {
            LOGE << "-1";
            continue;
          }
          auto target = smart_target[vehicle_key];
          auto attrs = target->add_attributes_();
          attrs->set_type_("vehicle_color");
          attrs->set_value_(color->value_);
          attrs->set_score_(color->score_);
          auto vehicle_color_des = AttributeConvert::Instance().GetAttrDes(
              "vehicle_color", color->value_);
          attrs->set_value_string_(vehicle_color_des);
          LOGD << "vehicle_color: " << color->value_ << ", "
               << vehicle_color_des;
        }
      }
    }

    if (output->name_ == "vehicle_type_vote") {
      auto types = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "vehicle types size: " << types->datas_.size();
      if (types->datas_.size() != vehicle_box_list.size()) {
        LOGE << "vehicle types size: " << types->datas_.size()
             << ", vehicle_box size: " << vehicle_box_list.size();
      }
      for (size_t i = 0; i < types->datas_.size(); ++i) {
        auto type = std::static_pointer_cast<xstream::Attribute_<int>>(
            types->datas_[i]);
        // 查找对应的track_id
        if (vehicle_box_list[i]->id_ == -1) {
          continue;
        }
        target_key vehicle_key(vehicle_box_list[i]->specific_type_,
                               vehicle_box_list[i]->id_);
        if (smart_target.find(vehicle_key) == smart_target.end()) {
          LOGE << "Not found the track_id target: " << vehicle_key.id;
        } else {
          if (type->state_ != xstream::DataState::VALID) {
            LOGE << "-1";
            continue;
          }
          auto target = smart_target[vehicle_key];
          auto attrs = target->add_attributes_();
          attrs->set_type_("vehicle_type");
          attrs->set_value_(type->value_);
          attrs->set_score_(type->score_);
          auto vehicle_type_des = AttributeConvert::Instance().GetAttrDes(
              "vehicle_type", type->value_);
          attrs->set_value_string_(vehicle_type_des);
          LOGD << "vehicle_type: " << type->value_ << ", "
               << vehicle_type_des;
        }
      }
    }

    if (output->name_ == "plate_num_vote") {
      auto plate_nums = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "plate_nums size: " << plate_nums->datas_.size();
      if (plate_nums->datas_.size() != plate_box_list.size()) {
        LOGE << "plate_nums size: " << plate_nums->datas_.size()
             << ", plate_box size: " << plate_box_list.size();
      }
      for (size_t i = 0; i < plate_nums->datas_.size(); ++i) {
        auto plate_num =
            std::static_pointer_cast<xstream::PlateNum>(plate_nums->datas_[i]);
        // 查找对应的track_id
        if (plate_box_list[i]->id_ == -1) {
          continue;
        }
        target_key plate_key(plate_box_list[i]->specific_type_,
                             plate_box_list[i]->id_);
        if (smart_target.find(plate_key) == smart_target.end()) {
          LOGE << "Not found the track_id target: " << plate_key.id;
        } else {
          if (plate_num->state_ != xstream::DataState::VALID) {
            LOGE << "-1";
            continue;
          }
          auto target = smart_target[plate_key];
          auto attrs = target->add_attributes_();
          attrs->set_type_("plate_num");
          attrs->set_score_(plate_num->score_);
          attrs->set_value_string_(plate_num->specific_type_);
          LOGD << "plate_num: " << plate_num->specific_type_;
        }
      }
    }

    if (output->name_ == "plate_color_match") {
      auto plate_colors = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "plate_colors size: " << plate_colors->datas_.size();
      if (plate_colors->datas_.size() != plate_box_list.size()) {
        LOGE << "plate_colors size: " << plate_colors->datas_.size()
             << ", plate_box size: " << plate_box_list.size();
      }
      for (size_t i = 0; i < plate_colors->datas_.size(); ++i) {
        auto plate_color = std::static_pointer_cast<xstream::Attribute_<int>>(
            plate_colors->datas_[i]);
        // 查找对应的track_id
        LOGD << "plate box list size: " << plate_box_list.size();
        if (plate_box_list[i]->id_ == -1) {
          continue;
        }
        target_key plate_key(plate_box_list[i]->specific_type_,
                             plate_box_list[i]->id_);
        if (smart_target.find(plate_key) == smart_target.end()) {
          LOGE << "Not found the track_id target: " << plate_key.id;
        } else {
          if (plate_color->state_ != xstream::DataState::VALID) {
            LOGE << "-1";
            continue;
          }
          auto target = smart_target[plate_key];
          auto attrs = target->add_attributes_();
          attrs->set_type_("plate_color");
          attrs->set_value_(plate_color->value_);
          attrs->set_score_(plate_color->score_);
          auto plate_color_des = AttributeConvert::Instance().GetAttrDes(
              "plate_color", plate_color->value_);
          attrs->set_value_string_(plate_color_des);
          LOGD << "plate_color: " << plate_color->value_ << ", "
               << plate_color_des;
        }
      }
    }

    if (output->name_ == "is_double_plate") {
      auto plate_rows = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "plate_rows size: " << plate_rows->datas_.size();
      if (plate_rows->datas_.size() != plate_box_list.size()) {
        LOGE << "plate_rows size: " << plate_rows->datas_.size()
             << ", plate_box size: " << plate_box_list.size();
      }
      for (size_t i = 0; i < plate_rows->datas_.size(); ++i) {
        auto plate_row = std::static_pointer_cast<xstream::Attribute_<int>>(
            plate_rows->datas_[i]);
        if (plate_row == nullptr) {
          LOGE << "failed to cast plate row";
        }
        // 查找对应的track_id
        if (plate_box_list[i]->id_ == -1) {
          continue;
        }
        target_key plate_key(plate_box_list[i]->specific_type_,
                             plate_box_list[i]->id_);
        if (smart_target.find(plate_key) == smart_target.end()) {
          LOGE << "Not found the track_id target: " << plate_key.id;
        } else {
          if (plate_row->state_ != xstream::DataState::VALID) {
            LOGE << "-1";
            continue;
          }
          auto target = smart_target[plate_key];
          auto attrs = target->add_attributes_();
          attrs->set_type_("is_double_plate");
          attrs->set_value_(plate_row->value_);
          attrs->set_score_(plate_row->score_);
          auto plate_row_des = AttributeConvert::Instance().GetAttrDes(
              "is_double_plate", plate_row->value_);
          attrs->set_value_string_(plate_row_des);
          LOGD << "plate_row: " << plate_row->value_ << ", " << plate_row_des;
        }
      }
    }
    if (output->name_ == "face_feature") {
      features = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << output->name_ << " size: " << features->datas_.size();

      if (features->datas_.size() != face_box_list.size()) {
        LOGI << "face feature size: " << features->datas_.size()
             << ", face_box size: " << face_box_list.size();
      }
      for (size_t i = 0; i < features->datas_.size(); ++i) {
        if (face_box_list[i]->id_ == -1) {
          continue;
        }
        target_key face_key(face_box_list[i]->specific_type_,
                            face_box_list[i]->id_);
        // 查找对应的track_id
        if (smart_target.find(face_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          auto one_person_feature =
              std::static_pointer_cast<xstream::BaseDataVector>(
                  features->datas_[i]);
          auto target = smart_target[face_key];
          for (size_t idx = 0; idx < one_person_feature->datas_.size(); idx++) {
            auto one_feature = std::static_pointer_cast<xstream::FloatFeature>(
                one_person_feature->datas_[idx]);
            if (one_feature->state_ != xstream::DataState::VALID) {
              LOGE << "-1";
              continue;
            }
            auto feature = target->add_float_arrays_();
            feature->set_type_(output->name_);
            for (auto item : one_feature->values_) {
              feature->add_value_(item);
            }
          }
        }
      }
    }
    if (output->name_ == "hand_lmk" ||
        output->name_ == "lowpassfilter_hand_lmk") {
      hand_lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "hand_lmk size: " << hand_lmks->datas_.size();
      if (hand_lmks->datas_.size() != hand_box_list.size()) {
        LOGE << "hand_lmk size: " << hand_lmks->datas_.size()
             << ", hand_box size: " << hand_box_list.size();
      }
      for (size_t i = 0; i < hand_lmks->datas_.size(); ++i) {
        auto lmk =
            std::static_pointer_cast<xstream::Landmarks>(hand_lmks->datas_[i]);
        // 查找对应的track_id
        if (hand_box_list[i]->id_ == -1) {
          continue;
        }
        target_key hand_key(hand_box_list[i]->specific_type_,
                            hand_box_list[i]->id_);
        if (smart_target.find(hand_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          auto target = smart_target[hand_key];
          auto proto_points = target->add_points_();
          proto_points->set_type_("hand_landmarks");
          for (size_t i = 0; i < lmk->values_.size(); ++i) {
            auto point = proto_points->add_points_();
            point->set_x_(lmk->values_[i].x_ * x_ratio);
            point->set_y_(lmk->values_[i].y_ * y_ratio);
            point->set_score_(lmk->values_[i].score_);
            LOGD << "x: " << std::round(lmk->values_[i].x_)
                 << " y: " << std::round(lmk->values_[i].y_)
                 << " score: " << lmk->values_[i].score_ << "\n";
          }
        }
      }
    }
    if (output->name_ == "lowpassfilter_lmk_106pts") {
      face_lmks = dynamic_cast<xstream::BaseDataVector *>(output.get());
      if (face_lmks->datas_.size() != face_box_list.size()) {
        LOGE << "lmk_106pts size: " << face_lmks->datas_.size()
             << ", hand_box size: " << face_box_list.size();
      }
      for (size_t i = 0; i < face_lmks->datas_.size(); ++i) {
        auto lmk =
            std::static_pointer_cast<xstream::Landmarks>(face_lmks->datas_[i]);
        // 查找对应的track_id
        if (face_box_list[i]->id_ == -1) {
          continue;
        }
        target_key lmk106pts_key(face_box_list[i]->specific_type_,
                                 face_box_list[i]->id_);
        if (smart_target.find(lmk106pts_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          auto target = smart_target[lmk106pts_key];
          auto proto_points = target->add_points_();
          proto_points->set_type_("lmk_106pts");
          for (size_t i = 0; i < lmk->values_.size(); ++i) {
            auto point = proto_points->add_points_();
            point->set_x_(lmk->values_[i].x_ * x_ratio);
            point->set_y_(lmk->values_[i].y_ * y_ratio);
            point->set_score_(lmk->values_[i].score_);
            LOGD << "x: " << std::round(lmk->values_[i].x_)
                 << " y: " << std::round(lmk->values_[i].y_)
                 << " score: " << lmk->values_[i].score_;
          }
        }
      }
    }
    if (output->name_ == "gesture_vote") {
      std::lock_guard<std::mutex> lk(static_attr_mutex_);
      auto gesture_votes =
          dynamic_cast<xstream::BaseDataVector *>(output.get());
      LOGD << "gesture_vote size: " << gesture_votes->datas_.size();
      if (gesture_votes->datas_.size() != hand_box_list.size()) {
        LOGE << "gesture_vote size: " << gesture_votes->datas_.size()
             << ", body_box size: " << hand_box_list.size();
      }
      for (size_t i = 0; i < gesture_votes->datas_.size(); ++i) {
        auto gesture_vote = std::static_pointer_cast<xstream::Attribute_<int>>(
            gesture_votes->datas_[i]);
        // 查找对应的track_id
        if (hand_box_list[i]->id_ == -1) {
          LOGI << "hand id invalid";
          continue;
        }
        target_key hand_key(hand_box_list[i]->specific_type_,
                            hand_box_list[i]->id_);
        if (smart_target.find(hand_key) == smart_target.end()) {
          LOGE << "Not found the track_id target";
        } else {
          if (gesture_vote->state_ != xstream::DataState::VALID) {
            LOGI << "gesture vote state not valid";
            continue;
          }
          int gesture_ret = -1;
          auto gesture_val = gesture_vote->value_;
          auto gesture_orig = gesture_vote->value_;
          int32_t hand_id = hand_box_list[i]->id_;
          LOGI << "gesture_type:" << gesture_val
               << "  frame_id:" << frame_id_
               << "  hand id:" << hand_id;

          // send original gesture, which is used for debug
          {
            auto target = smart_target[hand_key];
            auto attrs = target->add_attributes_();
            attrs->set_type_("gesture");
            attrs->set_value_(gesture_orig);
            attrs->set_value_string_(AttributeConvert::Instance().GetAttrDes(
                    "gesture", gesture_orig));
          }
        }
      }
    }
    output->name_ = old_output_name;
  }
  if (ap_mode_) {
    x3::MessagePack pack;
    pack.set_flow_(x3::MessagePack_Flow::MessagePack_Flow_CP2AP);
    pack.set_type_(x3::MessagePack_Type::MessagePack_Type_kXPlugin);
    if (channel_id_ == IMAGE_CHANNEL_FROM_AP) {  //  image is from ap
      pack.mutable_addition_()->mutable_frame_()->set_sequence_id_(frame_id_);
    }
    pack.set_content_(proto_frame_message.SerializeAsString());
    pack.SerializeToString(&proto_str);
  } else {
    proto_frame_message.SerializeToString(&proto_str);
  }
  LOGD << "smart result serial success";

  return std::move(proto_str);
}  // NOLINT

}  // namespace xproto
