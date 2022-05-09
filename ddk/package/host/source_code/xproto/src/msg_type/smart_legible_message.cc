//
// Created by xudong.du@hobot.cc on 14/15/2021.
// Copyright (c) 2018 horizon robotics. All rights reserved.
//
#include "xproto/msg_type/smart_legible_message.h"

#include <memory>

#include "hobotlog/hobotlog.hpp"
#include "xproto/msg_type/protobuf/x3.pb.h"

namespace xproto {
namespace message {
using RawDataImageFramePtr = std::shared_ptr<xstream::RawDataImageFrame>;

static const char g_map_Segmentation[] = "map_seg";
static const char g_body_Segmentation[] = "body_seg";
static const char g_face_pose[] = "face_pose";
static const char g_face_feature[] = "face_feature";
std::string SmartLegibleMessage::Serialize() {
  std::string proto_str;
  x3::FrameMessage proto_frame_message;
  proto_frame_message.set_timestamp_(time_stamp_);
  // img
  if (img_serialize_type_ == kSmartImageTypeNv12 && paramid_img_) {
    auto image = proto_frame_message.mutable_img_();
    auto image_str = image->mutable_buf_();
    int len = paramid_img_->DataSize(0) + paramid_img_->DataUVSize(0);
    image_str->resize(len);
    memcpy(const_cast<char *>(image_str->data()),
           (const char *)paramid_img_->Data(0), paramid_img_->DataSize(0));
    memcpy(const_cast<char *>(image_str->data() + paramid_img_->DataSize(0)),
           (const char *)paramid_img_->DataUV(0), paramid_img_->DataUVSize(0));
    image->set_type_(img_serialize_type_);
    image->set_width_(paramid_img_->Width(0));
    image->set_height_(paramid_img_->Height(0));
  }
  if (img_serialize_type_ == kSmartImageTypeJpg && background_img_ &&
      background_img_->DataSize() > 0) {
    int len = background_img_->DataSize();
    auto image = proto_frame_message.mutable_img_();
    auto image_str = image->mutable_buf_();
    image_str->resize(len);
    memcpy(const_cast<char *>(image_str->data()),
           (const char *)background_img_->Data(), len);
    image->set_type_(img_serialize_type_);
    image->set_width_(background_img_->Width());
    image->set_height_(background_img_->Height());
  }
  // smart_msg
  auto smart_msg = proto_frame_message.mutable_smart_msg_();
  smart_msg->set_timestamp_(time_stamp_);
  smart_msg->set_error_code_(smart_data_.error_code_);
  smart_msg->set_sequence_id_(sequence_id_);
  smart_msg->set_channel_id_(channel_id_);
  smart_msg->set_frame_id_(frame_id_);

  LOGD << "time_stamp: " << time_stamp_;
  LOGD << "targets_ num = " << smart_data_.targets_.size();
  for (auto org_target : smart_data_.targets_) {
    LOGD << "track_id: " << org_target->track_id_
         << ", type = " << org_target->type_;
    auto dst_target = smart_msg->add_targets_();
    dst_target->set_type_(org_target->type_);
    dst_target->set_track_id_(org_target->track_id_);
    /* box */
    for (auto org_box : org_target->boxs_) {
      if (org_box && org_box->state_ != xstream::DataState::VALID) {
        LOGE << "state is invalid!";
        continue;
      }
      LOGD << *org_box;
      auto dst_box = dst_target->add_boxes_();
      dst_box->set_type_(org_box->type_);
      dst_box->set_score_(org_box->score_);
      dst_box->set_name_(org_box->name_);
      dst_box->set_specific_type_(org_box->specific_type_);
      auto point1 = dst_box->mutable_top_left_();
      point1->set_x_(org_box->x1_);
      point1->set_y_(org_box->y1_);
      auto point2 = dst_box->mutable_bottom_right_();
      point2->set_x_(org_box->x2_);
      point2->set_y_(org_box->y2_);
    }
    /* lmks */
    for (auto org_lmk : org_target->lmks_) {
      if (org_lmk && org_lmk->state_ != xstream::DataState::VALID) {
        LOGE << "state is invalid!";
        continue;
      }
      LOGD << *org_lmk;
      auto dst_lmks = dst_target->add_points_();
      dst_lmks->set_type_(org_lmk->type_);
      dst_lmks->set_name_(org_lmk->name_);
      dst_lmks->set_specific_type_(org_lmk->specific_type_);
      for (auto point : org_lmk->values_) {
        auto dst_point = dst_lmks->add_points_();
        dst_point->set_score_(point.score_);
        dst_point->set_x_(point.x_);
        dst_point->set_y_(point.y_);
      }
    }
    /* attributes */
    for (auto org_attr : org_target->attributes_) {
      if (org_attr && org_attr->state_ != xstream::DataState::VALID) {
        LOGE << "state is invalid!";
        continue;
      }
      LOGD << *org_attr;
      auto dst_attr = dst_target->add_attributes_();
      dst_attr->set_type_(org_attr->type_);
      dst_attr->set_value_(static_cast<float>(org_attr->value_));
      dst_attr->set_value_string_(
          org_attr->specific_type_); /* 属性的此字段需要序列化 */
      dst_attr->set_score_(org_attr->score_);
      dst_attr->set_name_(org_attr->name_);
    }
    if (org_target->face_feature_ &&
        org_target->face_feature_->state_ == xstream::DataState::VALID) {
      auto org_feature = org_target->face_feature_;
      LOGD << *org_feature;
      auto dst_feature = dst_target->add_float_arrays_();
      dst_feature->set_type_(g_face_feature);
      dst_feature->set_name_(org_feature->name_);
      dst_feature->set_score_(org_feature->score_);
      dst_feature->set_specific_type_(org_feature->specific_type_);
      for (auto f : org_feature->values_) {
        dst_feature->add_value_(f);
      }
    }
    if (org_target->face_pose_ &&
        org_target->face_pose_->state_ == xstream::DataState::VALID) {
      auto org_pose = org_target->face_pose_;
      LOGD << *org_pose;
      auto dst_feature = dst_target->add_float_arrays_();
      dst_feature->set_type_(g_face_pose);
      dst_feature->set_score_(org_pose->score_);
      dst_feature->set_name_(org_pose->name_);
      /* 按照指定P/Y/R的顺序序列化*/
      dst_feature->add_value_(org_pose->pitch_);
      dst_feature->add_value_(org_pose->yaw_);
      dst_feature->add_value_(org_pose->roll_);
    }
    for (auto org_seg : org_target->body_seg_) {
      if (org_seg && org_seg->state_ != xstream::DataState::VALID) {
        LOGE << "state is invalid!";
        continue;
      }
      if (org_seg->width_ * org_seg->height_ != org_seg->values_.size()) {
        LOGE << "seg value has errors!";
        continue;
      }
      auto dst_seg = dst_target->add_float_matrixs_();
      dst_seg->set_type_(g_body_Segmentation);  // 用于反序列化时区分
      dst_seg->set_score_(org_seg->score_);
      dst_seg->set_name_(org_seg->name_);
      dst_seg->set_specific_type_(org_seg->specific_type_);
      for (int h = 0; h < org_seg->height_; h++) {
        auto float_array = dst_seg->add_arrays_();
        auto base_index = h * org_seg->width_;
        for (int w = 0; w < org_seg->width_; w++) {
          float_array->add_value_(org_seg->values_[base_index + w]);
        }
      }
    }
    if (org_target->map_seg_ &&
        org_target->map_seg_->state_ != xstream::DataState::VALID) {
      auto org_seg = org_target->map_seg_;
      if (org_seg->width_ * org_seg->height_ == org_seg->values_.size()) {
        auto dst_seg = dst_target->add_float_matrixs_();
        dst_seg->set_type_(g_map_Segmentation);
        dst_seg->set_score_(org_seg->score_);
        dst_seg->set_name_(org_seg->name_);
        dst_seg->set_specific_type_(org_seg->specific_type_);
        for (int h = 0; h < org_seg->height_; h++) {
          auto float_array = dst_seg->add_arrays_();
          auto base_index = h * org_seg->width_;
          for (int w = 0; w < org_seg->width_; w++) {
            float_array->add_value_(org_seg->values_[base_index + w]);
          }
        }
      } else {
        LOGE << "Seg value has errors!";
      }
    }
  }
  proto_frame_message.SerializeToString(&proto_str);
  return std::move(proto_str);
}

bool SmartLegibleMessage::DeSerialize(const std::string &data) {
  if (data.empty()) {
    return false;
  }
  x3::FrameMessage proto_frame_message;
  auto ret = proto_frame_message.ParseFromString(data);
  time_stamp_ = proto_frame_message.timestamp_();
  // img
  auto img = proto_frame_message.img_();
  if (img.type_() == kSmartImageTypeNv12 || img.type_() == kSmartImageTypeJpg) {
    RawDataImageFramePtr raw_image(new xstream::RawDataImageFrame(),
                                   [&](xstream::RawDataImageFrame *p) {
                                     if (p && p->data_) {
                                       LOGD << "delete rawdata image frame";
                                       std::free(p->data_);
                                     }
                                   });
    background_img_ = raw_image;
    if (img.type_() == kSmartImageTypeNv12) {
      raw_image->pixel_format_ = xstream::kHorizonVisionPixelFormatRawNV12;
      img_serialize_type_ = kSmartImageTypeNv12;
    } else if (img.type_() == kSmartImageTypeJpg) {
      raw_image->pixel_format_ =
          xstream::kHorizonVisionPixelFormatImageContainer;
      img_serialize_type_ = kSmartImageTypeJpg;
    }
    raw_image->width_ = img.width_();
    raw_image->height_ = img.height_();
    raw_image->image_size_ = img.buf_().length();
    raw_image->data_ = static_cast<uint8_t *>(calloc(1, img.buf_().length()));
    memcpy(raw_image->data_, img.buf_().data(), img.buf_().length());
  }
  // smart msg
  auto smart_msg = proto_frame_message.smart_msg_();
  channel_id_ = smart_msg.channel_id_();
  frame_id_ = smart_msg.frame_id_();
  sequence_id_ = smart_msg.sequence_id_();
  smart_data_.error_code_ = smart_msg.error_code_();
  auto targets = smart_msg.targets_();
  for (auto target : targets) {
    auto dst_target = std::make_shared<Target>();
    dst_target->track_id_ = target.track_id_();
    dst_target->type_ = target.type_();
    /* boxs */
    auto boxs = target.boxes_();
    for (auto box : boxs) {
      auto dst_box = std::make_shared<BBox>();
      dst_box->type_ = box.type_();
      dst_box->specific_type_ = box.specific_type_();  // 其type字段是固定的
      dst_box->score_ = box.score_();
      dst_box->name_ = box.name_();
      dst_box->x1_ = box.top_left_().x_();
      dst_box->y1_ = box.top_left_().y_();
      dst_box->x2_ = box.bottom_right_().x_();
      dst_box->y2_ = box.bottom_right_().y_();

      dst_target->boxs_.push_back(dst_box);
    }
    /* lmks */
    auto lmks = target.points_();
    for (auto lmk : lmks) {
      auto dst_lmk = std::make_shared<Landmarks>();
      dst_lmk->specific_type_ = lmk.type_();
      dst_lmk->specific_type_ = lmk.specific_type_();
      dst_lmk->name_ = lmk.name_();  // lmk中的每个point中设置了score
      for (auto point : lmk.points_()) {
        xstream::Point p(point.x_(), point.y_(), point.score_());
        dst_lmk->values_.push_back(p);
      }

      dst_target->lmks_.push_back(dst_lmk);
    }
    /* attributes */
    auto attributes = target.attributes_();
    for (auto org_attr : attributes) {
      auto dst_attr = std::make_shared<Attribute_<int32_t>>();
      dst_attr->type_ = org_attr.type_();
      dst_attr->name_ = org_attr.name_();
      dst_attr->value_ = org_attr.value_();
      dst_attr->specific_type_ = org_attr.value_string_();
      dst_attr->score_ = org_attr.score_();

      dst_target->attributes_.push_back(dst_attr);
    }
    /* feature and pose */
    auto float_array = target.float_arrays_();
    for (auto org_float_array : float_array) {
      if (org_float_array.type_() == g_face_pose) {
        auto dst_pose = std::make_shared<Pose3D>();
        dst_pose->name_ = org_float_array.name_();
        dst_pose->score_ = org_float_array.score_();
        auto org_value = org_float_array.value_();
        if (org_value.size() == 3) {
          dst_pose->pitch_ = org_value.Get(0);
          dst_pose->yaw_ = org_value.Get(1);
          dst_pose->roll_ = org_value.Get(2);
        }
        dst_target->face_pose_ = dst_pose;
      }
      if (org_float_array.type_() == g_face_feature) {
        auto dst_feature = std::make_shared<FloatFeature>();
        dst_feature->name_ = org_float_array.name_();
        dst_feature->score_ = org_float_array.score_();
        dst_feature->specific_type_ = org_float_array.specific_type_();
        auto org_value = org_float_array.value_();
        for (auto v : org_value) {
          dst_feature->values_.push_back(v);
        }
        dst_target->face_feature_ = dst_feature;
      }
    }

    /* segmentation */
    auto org_bogy_seg = target.float_matrixs_();
    for (auto org_seg : org_bogy_seg) {
      auto dst_seg = std::make_shared<Segmentation>();
      if (org_seg.type_() == g_body_Segmentation) {
        dst_target->body_seg_.push_back(dst_seg);
      } else if (org_seg.type_() == g_map_Segmentation) {
        dst_target->map_seg_ = dst_seg;
      } else {
        continue;
      }
      dst_seg->score_ = org_seg.score_();
      dst_seg->name_ = org_seg.name_();
      dst_seg->specific_type_ = org_seg.specific_type_();
      dst_seg->height_ = org_seg.arrays_().size();
      dst_seg->width_ = org_seg.arrays_(0).value_().size();
      for (int h = 0; h < dst_seg->height_; h++) {
        auto org_float_array = org_seg.arrays_(h);
        for (int w = 0; w < dst_seg->width_; w++) {
          dst_seg->values_.push_back(org_float_array.value_(w));
        }
      }
    }
    smart_data_.targets_.push_back(dst_target);
  }
  return ret;
}

SmartLegibleMessage::SmartLegibleMessage(const SmartLegibleMessage &other) {
  this->channel_id_ = other.channel_id_;
  this->frame_id_ = other.frame_id_;
  this->time_stamp_ = other.time_stamp_;
  this->sequence_id_ = other.sequence_id_;
  this->image_name_ = other.image_name_;
  this->img_serialize_type_ = other.img_serialize_type_;
  this->paramid_img_ = other.paramid_img_;
  this->background_img_ = other.background_img_;

  for (auto target : other.smart_data_.targets_) {
    auto dst_target = std::make_shared<Target>();
    dst_target->track_id_ = target->track_id_;
    dst_target->type_ = target->type_;
    /* boxs */
    for (auto box : target->boxs_) {
      auto dst_box = std::make_shared<BBox>();
      *dst_box = *box;
      dst_target->boxs_.push_back(dst_box);
    }
    /* lmks */
    for (auto lmk : target->lmks_) {
      auto dst_lmk = std::make_shared<Landmarks>();
      *dst_lmk = *lmk;

      dst_target->lmks_.push_back(dst_lmk);
    }
    /* attributes */
    for (auto org_attr : target->attributes_) {
      auto dst_attr = std::make_shared<Attribute_<int32_t>>();
      *dst_attr = *org_attr;
      dst_target->attributes_.push_back(dst_attr);
    }
    /* feature and pose */
    if (target->face_pose_) {
      auto dst_pose = std::make_shared<Pose3D>();
      *dst_pose = *target->face_pose_;
      dst_target->face_pose_ = dst_pose;
    }
    if (target->face_feature_) {
      auto dst_feature = std::make_shared<FloatFeature>();
      *dst_feature = *target->face_feature_;
      dst_target->face_feature_ = dst_feature;
    }

    /* segmentation */
    for (auto org_seg : target->body_seg_) {
      auto dst_seg = std::make_shared<Segmentation>();
      *dst_seg = *org_seg;
      dst_target->body_seg_.push_back(dst_seg);
    }
    if (target->map_seg_) {
      auto dst_seg = std::make_shared<Segmentation>();
      *dst_seg = *target->map_seg_;
      dst_target->map_seg_ = dst_seg;
    }
    smart_data_.targets_.push_back(dst_target);
  }
}

SmartLegibleMessage &SmartLegibleMessage::operator=(
    const SmartLegibleMessage &other) {
  if (this == &other)  // return self
  {
    return *this;
  }
  this->channel_id_ = other.channel_id_;
  this->frame_id_ = other.frame_id_;
  this->time_stamp_ = other.time_stamp_;
  this->sequence_id_ = other.sequence_id_;
  this->image_name_ = other.image_name_;
  this->img_serialize_type_ = other.img_serialize_type_;
  this->paramid_img_ = other.paramid_img_;
  this->background_img_ = other.background_img_;

  for (auto target : other.smart_data_.targets_) {
    auto dst_target = std::make_shared<Target>();
    dst_target->track_id_ = target->track_id_;
    dst_target->type_ = target->type_;
    /* boxs */
    for (auto box : target->boxs_) {
      auto dst_box = std::make_shared<BBox>();
      *dst_box = *box;
      dst_target->boxs_.push_back(dst_box);
    }
    /* lmks */
    for (auto lmk : target->lmks_) {
      auto dst_lmk = std::make_shared<Landmarks>();
      *dst_lmk = *lmk;

      dst_target->lmks_.push_back(dst_lmk);
    }
    /* attributes */
    for (auto org_attr : target->attributes_) {
      auto dst_attr = std::make_shared<Attribute_<int32_t>>();
      *dst_attr = *org_attr;
      dst_target->attributes_.push_back(dst_attr);
    }
    /* feature and pose */
    if (target->face_pose_) {
      auto dst_pose = std::make_shared<Pose3D>();
      *dst_pose = *target->face_pose_;
      dst_target->face_pose_ = dst_pose;
    }
    if (target->face_feature_) {
      auto dst_feature = std::make_shared<FloatFeature>();
      *dst_feature = *target->face_feature_;
      dst_target->face_feature_ = dst_feature;
    }

    /* segmentation */
    for (auto org_seg : target->body_seg_) {
      auto dst_seg = std::make_shared<Segmentation>();
      *dst_seg = *org_seg;
      dst_target->body_seg_.push_back(dst_seg);
    }
    if (target->map_seg_) {
      auto dst_seg = std::make_shared<Segmentation>();
      *dst_seg = *target->map_seg_;
      dst_target->map_seg_ = dst_seg;
    }
    smart_data_.targets_.push_back(dst_target);
  }
  return *this;
}
}  // namespace message
}  // namespace xproto
