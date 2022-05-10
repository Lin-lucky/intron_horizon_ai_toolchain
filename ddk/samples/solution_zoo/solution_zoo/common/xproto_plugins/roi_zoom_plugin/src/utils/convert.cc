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
#include "utils/convert.h"

#include <turbojpeg.h>

#include <string>
#include <vector>
#include <cstring>

#include "hobotlog/hobotlog.hpp"
#include "utils/roi_vision_type.h"
#include "xstream/vision_type.h"
#include "roi_zoom_plugin/roi_zoom_common.h"
namespace xproto {

using Points = xstream::Points_<float>;
using Attribute = xstream::Attribute_<int32_t>;
using TrackID = xstream::Attribute_<uint32_t>;
using ImageFramePtr = std::shared_ptr<xstream::ImageFrame>;
using XRocBaseDataVectorPtr = std::shared_ptr<xstream::BaseDataVector>;

#if 0
template <typename expect_Type, typename orig_Type>
void BoxConversion(
    expect_Type *t1, const orig_Type &t2, int32_t w_offset = 0,
    int32_t h_offset = 0, float w_ratio = 1, float h_ratio = 1) {
  t1->x1 = (t2.x1 - w_offset) * w_ratio;
  t1->y1 = (t2.y1 - h_offset) * h_ratio;
  t1->x2 = (t2.x2 - w_offset) * w_ratio;
  t1->y2 = (t2.y2 - h_offset) * h_ratio;
  t1->score = t2.score;
  t1->id = t2.id;
}
#endif

void BoxConversion(HorizonVisionBBox *t1, const xstream::BBox &t2,
                   int32_t w_offset = 0, int32_t h_offset = 0,
                   float w_ratio = 1, float h_ratio = 1) {
  t1->x1 = (t2.x1_ - w_offset) * w_ratio;
  t1->y1 = (t2.y1_ - h_offset) * h_ratio;
  t1->x2 = (t2.x2_ - w_offset) * w_ratio;
  t1->y2 = (t2.y2_ - h_offset) * h_ratio;
  t1->score = t2.score_;
  t1->id = t2.id_;
}

void BoxConversion(xstream::BBox *t1, const HorizonVisionBBox &t2,
                   int32_t w_offset = 0, int32_t h_offset = 0,
                   float w_ratio = 1, float h_ratio = 1) {
  t1->x1_ = (t2.x1 - w_offset) * w_ratio;
  t1->y1_ = (t2.y1 - h_offset) * h_ratio;
  t1->x2_ = (t2.x2 - w_offset) * w_ratio;
  t1->y2_ = (t2.y2 - h_offset) * h_ratio;
  t1->score_ = t2.score;
  t1->id_ = t2.id;
}

void CreateVisionBox(xstream::BaseDataPtr base, HorizonVisionBBox *vision_box);
xstream::BaseDataPtr CreateXRocBBox(const HorizonVisionBBox &vision_box);

xstream::BaseDataPtr CreateXRocFaceBBox(const HorizonVisionSmartData *);
xstream::BaseDataPtr CreateXRocHeadBBox(const HorizonVisionSmartData *);

xstream::BaseDataPtr CreateXRocBodyBBox(const HorizonVisionSmartData *);

template <typename expect_Type, typename orig_Type>
void PoseConversion(expect_Type *t1, const orig_Type &t2) {
  t1->yaw = t2.yaw;
  t1->pitch = t2.pitch;
  t1->roll = t2.roll;
  t1->score = t2.score;
}

xstream::BaseDataPtr CreateXRocPose(const HorizonVisionSmartData *);

void CreateVisionPose(xstream::BaseDataPtr base,
                      HorizonVisionPose3D *vision_pose);

xstream::BaseDataPtr CreateXRocLmk(const HorizonVisionSmartData *);

void ConvertLmks(const xstream::Landmarks &xroc_lmks,
                 HorizonVisionLandmarks *vision_lmk, float w_ratio = 1,
                 float h_ratio = 1);

void ConvertLmks(HorizonVisionLandmarks *vision_lmk,
                 const HorizonVisionLandmarks &lmk, int32_t w_offset = 0,
                 int32_t h_offset = 0, float w_ratio = 1, float h_ratio = 1);

void CreateVisionLmk(xstream::BaseDataPtr base,
                     HorizonVisionLandmarks *vision_lmk);

template <typename expect_Type, typename orig_Type>
void AgeConversion(expect_Type *t1, const orig_Type &t2) {
  t1->value = t2.value;
  t1->min = t2.min;
  t1->max = t2.max;
  t1->score = t2.score;
}

xstream::BaseDataPtr CreateXRocAge(const HorizonVisionSmartData *);

void CreateVisionAge(xstream::BaseDataPtr base, HorizonVisionAge *);

xstream::BaseDataPtr CreateXRocAttribute(const HorizonVisionAttribute &,
                                         const std::string &);

xstream::BaseDataPtr CreateXRocGender(const HorizonVisionSmartData *);

void CreateVisionAttribute(xstream::BaseDataPtr base,
                           HorizonVisionAttribute *vision_attr);
Points Box2Points(const xstream::BBox &box);

xstream::BBox Points2Box(const Points &points);
xstream::BaseDataPtr CreateXRocKPS(const HorizonVisionSmartData *);
xstream::BaseDataPtr CreateXRocReid(const HorizonVisionSmartData *);
xstream::BaseDataPtr CreateXRocMask(const HorizonVisionSmartData *);
void CreateVisionSegmentation(xstream::BaseDataPtr base,
                              HorizonVisionSegmentation *vision_mask);

void CreateVisionBox(xstream::BaseDataPtr base, HorizonVisionBBox *vision_box) {
  auto xroc_box = dynamic_cast<xstream::BBox *>(base.get());
  // BoxConversion<HorizonVisionBBox, xstream::BBox>(vision_box, *xroc_box);
  BoxConversion(vision_box, *xroc_box);
}

xstream::BaseDataPtr CreateXRocBBox(const HorizonVisionBBox &vision_box) {
  xstream::BBox tmp_box;
  tmp_box.type_ = "BBox";
  // BoxConversion<xstream::BBox, HorizonVisionBBox>(&tmp_box, vision_box);
  BoxConversion(&tmp_box, vision_box);
  std::shared_ptr<xstream::BBox> bbox(
      new xstream::BBox(tmp_box.x1_, tmp_box.y1_, tmp_box.x2_,
                        tmp_box.y2_, tmp_box.score_, tmp_box.id_));

  return bbox;
}

xstream::BaseDataPtr CreateXRocFaceBBox(
    const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face) {
    return CreateXRocBBox(smart_frame->face->face_rect);
  } else {
    return xstream::BaseDataPtr();
  }
}
xstream::BaseDataPtr CreateXRocHeadBBox(
    const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face) {
    return CreateXRocBBox(smart_frame->face->head_rect);
  } else {
    return xstream::BaseDataPtr();
  }
}

xstream::BaseDataPtr CreateXRocBodyBBox(
    const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->body) {
    return CreateXRocBBox(smart_frame->body->body_rect);
  } else {
    return xstream::BaseDataPtr();
  }
}

xstream::BaseDataPtr CreateXRocPose(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face) {
    auto vision_pose3d = smart_frame->face->pose3d;
    std::shared_ptr<xstream::Pose3D> pose(new xstream::Pose3D);
    pose->pitch_ = vision_pose3d.pitch;
    pose->yaw_ = vision_pose3d.yaw;
    pose->roll_ = vision_pose3d.roll;
    pose->score_ = vision_pose3d.score;
    return pose;
  } else {
    return xstream::BaseDataPtr();
  }
}

void CreateVisionPose(xstream::BaseDataPtr base,
                      HorizonVisionPose3D *vision_pose) {
  auto xroc_pose = dynamic_cast<xstream::Pose3D *>(base.get());
  vision_pose->pitch = xroc_pose->pitch_;
  vision_pose->yaw = xroc_pose->yaw_;
  vision_pose->roll = xroc_pose->roll_;
  vision_pose->score = xroc_pose->score_;
}

xstream::BaseDataPtr CreateXRocLmk(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face && smart_frame->face->landmarks) {
    auto vision_lmk = smart_frame->face->landmarks;
    std::shared_ptr<xstream::Landmarks> lmk(new xstream::Landmarks());
    lmk->type_ = "Landmarks";
    lmk->score_ = vision_lmk->score;
    for (size_t i = 0; i < vision_lmk->num; ++i) {
      xstream::Point p(vision_lmk->points[i].x, vision_lmk->points[i].y,
                             vision_lmk->points[i].score);
      lmk->values_.emplace_back(p);
    }
    return xstream::BaseDataPtr(lmk);
  } else {
    return xstream::BaseDataPtr();
  }
}

void ConvertLmks(const xstream::Landmarks &xroc_lmks,
                 HorizonVisionLandmarks *vision_lmk, float w_ratio,
                 float h_ratio) {
  HOBOT_CHECK(vision_lmk);
  vision_lmk->score = xroc_lmks.score_;
  vision_lmk->num = xroc_lmks.values_.size();
  if (vision_lmk->num > 0) {
    vision_lmk->points = static_cast<HorizonVisionPoint *>(
        std::calloc(vision_lmk->num, sizeof(HorizonVisionPoint)));
    for (size_t i = 0; i < vision_lmk->num; ++i) {
      vision_lmk->points[i].x = xroc_lmks.values_[i].x_ * w_ratio;
      vision_lmk->points[i].y = xroc_lmks.values_[i].y_ * h_ratio;
      vision_lmk->points[i].score = xroc_lmks.values_[i].score_;
    }
  }
}

void ConvertLmks(HorizonVisionLandmarks *vision_lmk,
                 const HorizonVisionLandmarks &lmk, int32_t w_offset,
                 int32_t h_offset, float w_ratio, float h_ratio) {
  HOBOT_CHECK(vision_lmk);
  vision_lmk->num = lmk.num;
  for (size_t i = 0; i < vision_lmk->num; ++i) {
    vision_lmk->points[i].x = (lmk.points[i].x - w_offset) * w_ratio;
    vision_lmk->points[i].y = (lmk.points[i].y - h_offset) * h_ratio;
  }
}

void CreateVisionLmk(xstream::BaseDataPtr base,
                     HorizonVisionLandmarks *vision_lmk) {
  // auto xroc_lmk = dynamic_cast<XRocLandmarks *>(base.get());
  auto xroc_lmk = dynamic_cast<xstream::Landmarks *>(base.get());
  HOBOT_CHECK(xroc_lmk);
  ConvertLmks(*xroc_lmk, vision_lmk);
}

xstream::BaseDataPtr CreateXRocAge(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face) {
    auto vision_age = smart_frame->face->age;
    std::shared_ptr<xstream::Age> age(new xstream::Age());
    age->type_ = "Age";
    age->value_ = vision_age.value;
    age->min_ = vision_age.min;
    age->max_ = vision_age.max;
    age->score_ = vision_age.score;
    return age;
  } else {
    return xstream::BaseDataPtr();
  }
}

void CreateVisionAge(xstream::BaseDataPtr base, HorizonVisionAge *vision_age) {
  auto xroc_age = dynamic_cast<xstream::Age *>(base.get());
  vision_age->value = xroc_age->value_;
  vision_age->min = xroc_age->min_;
  vision_age->max = xroc_age->max_;
  vision_age->score = xroc_age->score_;
}

xstream::BaseDataPtr CreateXRocAttribute(
    const HorizonVisionAttribute &vision_attr, const std::string &type_name) {
  std::shared_ptr<Attribute> attr(new Attribute());
  attr->type_ = type_name;
  attr->value_ = vision_attr.value;
  attr->score_ = vision_attr.score;
  return attr;
}

xstream::BaseDataPtr CreateXRocGender(
    const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face) {
    return CreateXRocAttribute(smart_frame->face->gender, "Gender");
  } else {
    return xstream::BaseDataPtr();
  }
}

void CreateVisionAttribute(xstream::BaseDataPtr base,
                           HorizonVisionAttribute *vision_attr) {
  auto xroc_attr = dynamic_cast<Attribute *>(base.get());
  HOBOT_CHECK(xroc_attr) << "input type is " << base->type_;
  vision_attr->score = xroc_attr->score_;
  vision_attr->value = xroc_attr->value_;
}

Points Box2Points(const xstream::BBox &box) {
  xstream::Point top_left = {box.x1_, box.y1_, box.score_};
  xstream::Point bottom_right = {box.x2_, box.y2_, box.score_};
  Points points;
  points.values_ = {top_left, bottom_right};
  return points;
}

xstream::BBox Points2Box(const Points &points) {
  xstream::BBox box;
  box.x1_ = points.values_[0].x_;
  box.y1_ = points.values_[0].y_;
  box.x2_ = points.values_[1].x_;
  box.y2_ = points.values_[1].y_;
  box.score_ = points.values_[0].score_;
  return box;
}

xstream::BaseDataPtr CreateXRocKPS(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->body) {
    /// \todo
    return xstream::BaseDataPtr();
  } else {
    return xstream::BaseDataPtr();
  }
}

xstream::BaseDataPtr CreateXRocReid(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->body) {
    ///\ todo
    return xstream::BaseDataPtr();
  } else {
    return xstream::BaseDataPtr();
  }
}

xstream::BaseDataPtr CreateXRocMask(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->body) {
    /// \todo
    return xstream::BaseDataPtr();
  } else {
    return xstream::BaseDataPtr();
  }
}

void CreateVisionSegmentation(xstream::BaseDataPtr base,
                              HorizonVisionSegmentation *vision_mask) {
  auto xroc_segmentation = dynamic_cast<xstream::Segmentation *>(base.get());
  HOBOT_CHECK(xroc_segmentation);
  vision_mask->width = xroc_segmentation->width_;
  vision_mask->height = xroc_segmentation->height_;
  vision_mask->num = xroc_segmentation->values_.size();
  vision_mask->values =
      static_cast<float *>(std::calloc(vision_mask->num, sizeof(float)));
  memcpy(vision_mask->values, xroc_segmentation->values_.data(),
         vision_mask->num * sizeof(float));
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
    const FaceQuality *face_quality,
    const xstream::BaseDataVector *breathing_mask_detect,
    const bool &with_debug_info) {
  if (!face_boxs) {
    return;
  }
  int current_valid_idx = 0;
  for (size_t i = 0; i < face_boxs->datas_.size(); ++i) {
    HorizonVisionFaceSmartData *face_data = nullptr;
    HorizonVisionAllocFaceSmartData(&face_data);
    CreateVisionBox(face_boxs->datas_[i],
                                              &face_data->face_rect);
    if (IsValidElement(head_box, i)) {
      CreateVisionBox(head_box->datas_[i],
                                                &face_data->head_rect);
    }
    if (IsValidElement(pose, i)) {
      CreateVisionPose(pose->datas_[i],
                                                 &face_data->pose3d);
    }
    face_data->landmarks = nullptr;
    if (IsValidElement(lmk, i)) {
      HorizonVisionAllocLandmarks(&face_data->landmarks);
      CreateVisionLmk(lmk->datas_[i],
                                                face_data->landmarks);
    }
    if (IsValidElement(age, i)) {
      CreateVisionAge(age->datas_[i],
                                                &face_data->age);
    }
    if (IsValidElement(gender, i)) {
      CreateVisionAttribute(gender->datas_[i],
                                                      &face_data->gender);
    }
    if (IsValidElement(face_quality->blur, i)) {
      CreateVisionAttribute(
          face_quality->blur->datas_[i], &face_data->quality.blur);
    }
    if (IsValidElement(face_quality->brightness, i)) {
      CreateVisionAttribute(
          face_quality->brightness->datas_[i], &face_data->quality.brightness);
    }
    if (IsValidElement(face_quality->eye_abnormalities, i)) {
      CreateVisionAttribute(
          face_quality->eye_abnormalities->datas_[i],
          &face_data->quality.eye_abnormalities);
    }
    if (IsValidElement(face_quality->mouth_abnormal, i)) {
      CreateVisionAttribute(
          face_quality->mouth_abnormal->datas_[i],
          &face_data->quality.mouth_abnormal);
    }
    if (IsValidElement(face_quality->left_eye, i)) {
      CreateVisionAttribute(
          face_quality->left_eye->datas_[i], &face_data->quality.left_eye);
    }
    if (IsValidElement(face_quality->right_eye, i)) {
      CreateVisionAttribute(
          face_quality->right_eye->datas_[i], &face_data->quality.right_eye);
    }
    if (IsValidElement(face_quality->left_brow, i)) {
      CreateVisionAttribute(
          face_quality->left_brow->datas_[i], &face_data->quality.left_brow);
    }
    if (IsValidElement(face_quality->right_brow, i)) {
      CreateVisionAttribute(
          face_quality->right_brow->datas_[i], &face_data->quality.right_brow);
    }
    if (IsValidElement(face_quality->forehead, i)) {
      CreateVisionAttribute(
          face_quality->forehead->datas_[i], &face_data->quality.forehead);
    }
    if (IsValidElement(face_quality->left_cheek, i)) {
      CreateVisionAttribute(
          face_quality->left_cheek->datas_[i], &face_data->quality.left_cheek);
    }
    if (IsValidElement(face_quality->right_cheek, i)) {
      CreateVisionAttribute(
          face_quality->right_cheek->datas_[i],
          &face_data->quality.right_cheek);
    }
    if (IsValidElement(face_quality->nose, i)) {
      CreateVisionAttribute(
          face_quality->nose->datas_[i], &face_data->quality.nose);
    }
    if (IsValidElement(face_quality->mouth, i)) {
      CreateVisionAttribute(
          face_quality->mouth->datas_[i], &face_data->quality.mouth);
    }
    if (IsValidElement(face_quality->jaw, i)) {
      CreateVisionAttribute(
          face_quality->jaw->datas_[i], &face_data->quality.jaw);
    }
    if (IsValidElement(breathing_mask_detect, i)) {
      CreateVisionAttribute(
          breathing_mask_detect->datas_[i], &face_data->mask);
    }
    if (IsValidElement(anti_spf_list, i)) {
      CreateVisionAttribute(
          anti_spf_list->datas_[i], &face_data->anti_spoofing);
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
        CreateVisionAttribute(
            rgb_anti_spf->datas_[i], &extra_info->rgb_anti_spf_);
      } else {
        extra_info->rgb_anti_spf_.value = -1;
        extra_info->rgb_anti_spf_.score = 1;
      }
      // rgb norm box
      if (IsValidElement(rgb_norm_rois, i)) {
        CreateVisionBox(rgb_norm_rois->datas_[i],
                                                  &extra_info->rgb_norm_box_);
      }
      // rgb box
      if (IsValidElement(face_boxs, i)) {
        CreateVisionBox(face_boxs->datas_[i],
                                                  &extra_info->rgb_box_);
      }
      // get nir idx
      size_t nir_idx = 0;
      bool match = false;
      if (nir_boxs) {
        for (size_t nir_i = 0; nir_i < nir_boxs->datas_.size(); ++nir_i) {
          auto nir_box = dynamic_cast<xstream::BBox *>(
              nir_boxs->datas_[nir_i].get());
          if (nir_box && nir_box->id_ == (int32_t)face_data->track_id) {
            nir_idx = nir_i;
            match = true;
            break;
          }
        }
      }
#if 0
      if (IsValidElement(nir_lmk, nir_idx)) {
        HorizonVisionAllocLandmarks(&extra_info->nir_landmarks_);
        CreateVisionLmk(nir_lmk->datas_[nir_idx],
                                               extra_info->nir_landmarks_);
      }
#endif

      // nir anti spf
      if (match && IsValidElement(nir_anti_spf, nir_idx)) {
        CreateVisionAttribute(
            nir_anti_spf->datas_[nir_idx], &extra_info->nir_anti_spf_);
      } else {
        extra_info->nir_anti_spf_.value = -1;
        extra_info->nir_anti_spf_.score = 1;
      }
      // nir norm box
      if (match && IsValidElement(nir_norm_rois, nir_idx)) {
        CreateVisionBox(
            nir_norm_rois->datas_[nir_idx], &extra_info->nir_norm_box_);
      }
      // nir box
      if (match && IsValidElement(nir_boxs, nir_idx)) {
        CreateVisionBox(nir_boxs->datas_[nir_idx],
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
    CreateVisionBox(body_boxs->datas_[i],
                                              &body_data->body_rect);
    body_data->skeleton = nullptr;
    if (IsValidElement(kps, i)) {
      HorizonVisionAllocLandmarks(&body_data->skeleton);
      CreateVisionLmk(kps->datas_[i],
                                                body_data->skeleton);
    }
    body_data->segmentation = nullptr;
    if (IsValidElement(mask, i)) {
      HorizonVisionAllocSegmentation(&body_data->segmentation);
      CreateVisionSegmentation(
          mask->datas_[i], body_data->segmentation);
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
    auto id = dynamic_cast<TrackID *>(ids->datas_[i].get());
    HOBOT_CHECK(id);
    targets[i].track_id = id->value_;
    LOGD << id->value_ << " disappeared!";
    targets[i].type = SMART_TYPE_DISAPPEARED;
  }
}

void ParseHandSmartData(HorizonVisionSmartData *targets,
                        const xstream::BaseDataVector *hand_boxs,
                        const xstream::BaseDataVector *gesture_votes) {
  if (!hand_boxs) {
    LOGD << "no hand boxs";
    return;
  }

  int current_valid_idx = 0;
  for (size_t i = 0; i < hand_boxs->datas_.size(); ++i) {
    if (!IsValidBaseData(hand_boxs->datas_[i])) {
      LOGI << "Has not hand rects";
      continue;
    }
    HorizonVisionHandSmartData *hand_data = nullptr;
    HorizonVisionAllocHandSmartData(&hand_data);
    CreateVisionBox(hand_boxs->datas_[i], &hand_data->hand_rect);

    if (IsValidElement(gesture_votes, i)) {
      auto gesture_vote =
          std::static_pointer_cast<xstream::Attribute_<int>>(
              gesture_votes->datas_[i]);
      hand_data->hand_gesture = gesture_vote->value_;
    }

    hand_data->track_id = hand_data->hand_rect.id;
    targets[current_valid_idx].hand = hand_data;
    targets[current_valid_idx++].track_id = hand_data->track_id;
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
  // xstream::BaseDataVector *face_disappeared_track_id_list = nullptr;
  /// debug info
  xstream::BaseDataVector *rgb_anti_spf = nullptr;
  xstream::BaseDataVector *nir_anti_spf = nullptr;
  xstream::BaseDataVector *nir_boxs = nullptr;
  xstream::BaseDataVector *rgb_norm_rois = nullptr;
  xstream::BaseDataVector *nir_norm_rois = nullptr;
  FaceQuality face_quality;
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
      // face_disappeared_track_id_list =
      //     dynamic_cast<xstream::BaseDataVector *>(output.get());
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

  auto total_targets = face_target + body_target;
  LOGI << "Total target num = " << face_target << " + " << body_target;
  HorizonVisionSmartData *targets = nullptr;
  if (total_targets > 0) {
    HorizonVisionAllocSmartData(&targets, total_targets);
    ParseFaceSmartData(targets, face_boxs, head_box, lmk, nir_lmk, pose, age,
                       gender, anti_spf_list, rgb_anti_spf, rgb_norm_rois,
                       nir_anti_spf, nir_norm_rois, nir_boxs, &face_quality,
                       breathing_mask_detect, false);
    ParseBodySmartData(&targets[face_target], body_boxs, kps, mask);
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
  xstream::BaseDataVector *hand_boxs = nullptr;
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
  // xstream::BaseDataVector *face_disappeared_track_id_list = nullptr;
  /// debug info
  xstream::BaseDataVector *rgb_anti_spf = nullptr;
  xstream::BaseDataVector *nir_anti_spf = nullptr;
  xstream::BaseDataVector *nir_boxs = nullptr;
  xstream::BaseDataVector *rgb_norm_rois = nullptr;
  xstream::BaseDataVector *nir_norm_rois = nullptr;
  xstream::BaseDataVector *gesture_votes = nullptr;

  FaceQuality face_quality;
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
               output->name_ == "face_bbox_list_gathering" ||
               output->name_ == "face_lowpassfilter_box") {
      face_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "head_final_box") {
      head_box = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "hand_box" ||
               output->name_ == "hand_final_box" ||
               output->name_ == "hand_lowpassfilter_box") {
      hand_boxs = dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "body_box" ||
               output->name_ == "body_final_box" ||
               output->name_ == "body_lowpassfilter_box") {
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
      // face_disappeared_track_id_list =
      //     dynamic_cast<xstream::BaseDataVector *>(output.get());
    } else if (output->name_ == "gesture_vote") {
      gesture_votes = dynamic_cast<xstream::BaseDataVector *>(output.get());
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
  auto hand_target = hand_boxs ? hand_boxs->datas_.size() : 0;
  auto total_targets = face_target + body_target + hand_target;
  LOGD << "Total target num = " << face_target << " + " << body_target << " + "
       << hand_target;
  HorizonVisionSmartData *targets = nullptr;
  if (total_targets > 0) {
    HorizonVisionAllocSmartData(&targets, total_targets);
    ParseFaceSmartData(targets, face_boxs, head_box, lmk, nir_lmk, pose, age,
                       gender, anti_spf_list, rgb_anti_spf, rgb_norm_rois,
                       nir_anti_spf, nir_norm_rois, nir_boxs, &face_quality,
                       breathing_mask_detect, false);
    ParseBodySmartData(&targets[face_target], body_boxs, kps, mask);
    ParseHandSmartData(&targets[face_target + body_target],
                       hand_boxs, gesture_votes);
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

}  // namespace xproto
