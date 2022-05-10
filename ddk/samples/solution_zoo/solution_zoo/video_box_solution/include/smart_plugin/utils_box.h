/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file util.h
 * @brief
 * @author ruoting.ding
 * @email ruoting.ding@horizon.ai
 * @date 2019/4/29
 */

#ifndef INCLUDE_SMARTPLUGINBOX_UTILSBOX_H_
#define INCLUDE_SMARTPLUGINBOX_UTILSBOX_H_
#include <cstdint>
#include <memory>
#include <string>

#include "smart_plugin/convert.h"
#include "vision/vision_type.h"
#include "vision/vision_type.hpp"
#include "vision/vision_type_util.h"
#include "xstream/xstream_world.h"
namespace solution {
namespace video_box {

using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XRocImageFramePtr = solution::video_box::XStreamData<ImageFramePtr>;
using XRocBBox = solution::video_box::XStreamData<hobot::vision::BBox>;
using XRocPose3D = solution::video_box::XStreamData<hobot::vision::Pose3D>;
using XRocLandmarks =
    solution::video_box::XStreamData<hobot::vision::Landmarks>;
using XRocAge = solution::video_box::XStreamData<hobot::vision::Age>;
using XRocAttribute =
    solution::video_box::XStreamData<hobot::vision::Attribute<int32_t>>;
using XRocBaseDataVectorPtr = std::shared_ptr<xstream::BaseDataVector>;
using XRocSegmentation =
    solution::video_box::XStreamData<hobot::vision::Segmentation>;
using XRocFeature = solution::video_box::XStreamData<hobot::vision::Feature>;
using XRocAntiSpoofing =
    solution::video_box::XStreamData<hobot::vision::AntiSpoofing>;
using XRocID = solution::video_box::XStreamData<HorizonVisionTrackID>;

template <typename expect_Type, typename orig_Type>
void BoxConversion(expect_Type *t1, const orig_Type &t2, int32_t w_offset = 0,
                   int32_t h_offset = 0, float w_ratio = 1, float h_ratio = 1) {
  t1->x1 = (t2.x1_ - w_offset) * w_ratio;
  t1->y1 = (t2.y1_ - h_offset) * h_ratio;
  t1->x2 = (t2.x2_ - w_offset) * w_ratio;
  t1->y2 = (t2.y2_ - h_offset) * h_ratio;
  t1->score = t2.score_;
  t1->id = t2.id_;
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
void AgeConversion(expect_Type &t1, const orig_Type &t2) {
  t1.value = t2.value_;
  t1.min = t2.min_;
  t1.max = t2.max_;
  t1.score = t2.score_;
}

xstream::BaseDataPtr CreateXRocAge(const HorizonVisionSmartData *);

void CreateVisionAge(xstream::BaseDataPtr base, HorizonVisionAge *);

xstream::BaseDataPtr CreateXRocAttribute(const HorizonVisionAttribute &,
                                         const std::string &);

xstream::BaseDataPtr CreateXRocGender(const HorizonVisionSmartData *);

void CreateVisionAttribute(xstream::BaseDataPtr base,
                           HorizonVisionAttribute *vision_attr);
xstream::FloatPoints Box2Points(const xstream::BBox &box);

xstream::BBox Points2Box(const xstream::FloatPoints &points);
xstream::BaseDataPtr CreateXRocKPS(const HorizonVisionSmartData *);
xstream::BaseDataPtr CreateXRocReid(const HorizonVisionSmartData *);
xstream::BaseDataPtr CreateXRocMask(const HorizonVisionSmartData *);
void CreateVisionSegmentation(xstream::BaseDataPtr base,
                              HorizonVisionSegmentation *vision_mask);
}  // namespace video_box
}  // namespace solution

#endif  //  INCLUDE_SMARTPLUGINBOX_UTILSBOX_H_
