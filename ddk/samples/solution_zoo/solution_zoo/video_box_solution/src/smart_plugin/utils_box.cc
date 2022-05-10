/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file util.h
 * @brief
 * @author ruoting.ding
 * @email ruoting.ding@horizon.ai
 * @date 2019/4/29
 */

#include "smart_plugin/utils_box.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "xstream/vision_type.h"

namespace solution {
namespace video_box {

void CreateVisionBox(xstream::BaseDataPtr base, HorizonVisionBBox *vision_box) {
//  XRocBBox *xroc_box = dynamic_cast<XRocBBox *>(base.get());
  xstream::BBox* xroc_box = dynamic_cast<xstream::BBox *>(base.get());
  BoxConversion<HorizonVisionBBox, xstream::BBox>(vision_box,
                                                        *xroc_box);
                                                   //     xroc_box->value);
}

xstream::BaseDataPtr CreateXRocBBox(const HorizonVisionBBox &vision_box) {
//  std::shared_ptr<XRocBBox> bbox(new XRocBBox());
  std::shared_ptr<xstream::BBox> bbox(new xstream::BBox());
  bbox->type_ = "BBox";
  #if 0
  BoxConversion<xstream::BBox*, HorizonVisionBBox>(bbox,
                                                        vision_box);
  #endif
  bbox->x1_ = vision_box.x1;
  bbox->y1_ = vision_box.y1;
  bbox->x2_ = vision_box.x2;
  bbox->y2_ = vision_box.y2;
  return xstream::BaseDataPtr(bbox);
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
  //  std::shared_ptr<XRocPose3D> pose(new XRocPose3D());
    std::shared_ptr<xstream::Pose3D> pose(new xstream::Pose3D());
    pose->type_ = "Pose3D";
  //  PoseConversion<hobot::vision::Pose3D, HorizonVisionPose3D>(&pose->value,
  //                                                             vision_pose3d);
    pose->yaw_ = vision_pose3d.yaw;
    pose->pitch_ = vision_pose3d.pitch;
    pose->roll_ = vision_pose3d.roll;
    pose->score_ = vision_pose3d.score;
    return xstream::BaseDataPtr(pose);
  } else {
    return xstream::BaseDataPtr();
  }
}

void CreateVisionPose(xstream::BaseDataPtr base,
                      HorizonVisionPose3D *vision_pose) {
  #if 0
  auto xroc_pose = dynamic_cast<XRocPose3D *>(base.get());
  PoseConversion<HorizonVisionPose3D, hobot::vision::Pose3D>(vision_pose,
                                                             xroc_pose->value);
  #endif
  auto pose = dynamic_cast<xstream::Pose3D *>(base.get());
  vision_pose->yaw = pose->yaw_;
  vision_pose->pitch = pose->pitch_;
  vision_pose->roll = pose->roll_;
  vision_pose->score = pose->score_;
}

xstream::BaseDataPtr CreateXRocLmk(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face && smart_frame->face->landmarks) {
    auto vision_lmk = smart_frame->face->landmarks;
  //  std::shared_ptr<XRocLandmarks> lmk(new XRocLandmarks());
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
//  auto xroc_lmk = dynamic_cast<XRocLandmarks *>(base.get());
  auto lmk = dynamic_cast<xstream::Landmarks *>(base.get());
  HOBOT_CHECK(lmk);
  ConvertLmks(*lmk, vision_lmk);
}

xstream::BaseDataPtr CreateXRocAge(const HorizonVisionSmartData *smart_frame) {
  if (smart_frame && smart_frame->face) {
    auto vision_age = smart_frame->face->age;
    auto age = std::make_shared<xstream::Age>();
    age->type_ = "Age";
    age->value_ = vision_age.value;
    age->min_ = vision_age.min;
    age->max_ = vision_age.max;
    age->score_ = vision_age.score;
    return xstream::BaseDataPtr(age);
  } else {
    return xstream::BaseDataPtr();
  }
}

void CreateVisionAge(xstream::BaseDataPtr base, HorizonVisionAge *vision_age) {
  auto xroc_age = dynamic_cast<xstream::Age *>(base.get());
  AgeConversion<HorizonVisionAge, xstream::Age>(*vision_age,
                                                      *xroc_age);
}

xstream::BaseDataPtr CreateXRocAttribute(
    const HorizonVisionAttribute &vision_attr, const std::string &type_name) {
  std::shared_ptr<XRocAttribute> attr(new XRocAttribute());
  attr->type_ = type_name;
  attr->value.value = vision_attr.value;
  attr->value.score = vision_attr.score;
  return xstream::BaseDataPtr(attr);
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
  auto xroc_attr = dynamic_cast<xstream::Attribute_<int32_t> *>(base.get());
  HOBOT_CHECK(xroc_attr) << "input type is " << base->type_;
  vision_attr->score = xroc_attr->score_;
  vision_attr->value = xroc_attr->value_;
}

xstream::FloatPoints Box2Points(const xstream::BBox &box) {
  xstream::Point top_left = {box.x1_, box.y1_, box.score_};
  xstream::Point bottom_right = {box.x2_, box.y2_, box.score_};
  xstream::FloatPoints points;
  points.values_ = {top_left, bottom_right};
  return points;
}

xstream::BBox Points2Box(const xstream::FloatPoints &points) {
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
  /*
memcpy(vision_mask->values, xroc_segmentation->values_.data(),
     vision_mask->num * sizeof(float));
     */
  for (int i = 0; i < vision_mask->num; i++) {
    vision_mask->values[i] = xroc_segmentation->values_[i];
  }
}

}  // namespace video_box
}  // namespace solution
