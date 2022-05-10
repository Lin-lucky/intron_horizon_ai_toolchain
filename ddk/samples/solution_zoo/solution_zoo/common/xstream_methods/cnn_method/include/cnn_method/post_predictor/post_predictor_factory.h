/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: PostPredictorFactory.h
 * @Brief: declaration of the PostPredictorFactory
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-07-17 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-07-17 16:01:54
 */

#ifndef INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTORFACTORY_H_
#define INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTORFACTORY_H_

#include "cnn_method/post_predictor/act_post_predictor.h"
#include "cnn_method/post_predictor/age_gender_post_predictor.h"
#include "cnn_method/post_predictor/antispf_post_predictor.h"
#include "cnn_method/post_predictor/backbone_post_predictor.h"
#include "cnn_method/post_predictor/binary_classify_post_predictor.h"
#include "cnn_method/post_predictor/classify_post_predictor.h"
#include "cnn_method/post_predictor/common_lmk_post_predictor.h"
#include "cnn_method/post_predictor/faceid_post_predictor.h"
#include "cnn_method/post_predictor/face_quality_post_predictor.h"
#include "cnn_method/post_predictor/lmk_pose_post_predictor.h"
#include "cnn_method/post_predictor/platenum_post_predictor.h"
#include "cnn_method/post_predictor/post_predictor.h"
#include "cnn_method/post_predictor/vehicle_color_post_predictor.h"
#include "cnn_method/post_predictor/vehicle_type_post_predictor.h"
#include "cnn_method/post_predictor/vid_post_predictor.h"

namespace xstream {

class PostPredictorFactory {
 public:
  static PostPredictor* GetPostPredictor(PostFun post_fun) {
    switch (post_fun) {
      case PostFun::FACE_ID:
        return new FaceIdPostPredictor();
      case PostFun::ANTI_SPF:
        return new AntiSpfPostPredictor();
      case PostFun::LMK_POSE:
        return new LmkPosePostPredictor();
      case PostFun::AGE_GENDER:
        return new AgeGenderPostPredictor();
      case PostFun::FACE_QUALITY:
        return new FaceQualityPostPredictor();
      case PostFun::BINARYCLASSIFY:
        return new BinaryClassifyPostPredictor();
      case PostFun::VEHICLE_TYPE:
        return new VehicleTypePostPredictor();
      case PostFun::VEHICLE_COLOR:
        return new VehicleColorPostPredictor();
      case PostFun::PLATE_NUM:
        return new PlateNumPostPredictor();
      case PostFun::CLASSIFY:
        return new ClassifyPostPredictor();
      case PostFun::ACT_DET:
        return new ActPostPredictor();
      case PostFun::BACK_BONE:
        return new BackBonePostPredictor();
      case PostFun::VID:
        return new VidPostPredictor();
      case PostFun::COMMON_LMK:
        return new CommonLmkPostPredictor();
    }
    return nullptr;
  }
};

}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_POSTPREDICTOR_POSTPREDICTORFACTORY_H_
