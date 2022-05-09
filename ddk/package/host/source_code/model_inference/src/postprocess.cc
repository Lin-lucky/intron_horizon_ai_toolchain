/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of preprocess
 * @file   preprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/postprocess/postprocess.h"

#include "model_inference/postprocess/age_gender_postprocess.h"
#include "model_inference/postprocess/face_quality_postprocess.h"
#include "model_inference/postprocess/faceid_postprocess.h"
#include "model_inference/postprocess/gesture_postprocess.h"
#include "model_inference/postprocess/horizon_multitask_postprocess.h"
#include "model_inference/postprocess/lmks3_postprocess.h"
#include "model_inference/postprocess/lmks4_postprocess.h"
#include "model_inference/postprocess/matrix_detection_postprocess.h"
#include "model_inference/postprocess/mobilenetv2_postprocess.h"
#include "model_inference/postprocess/plate_num_postprocess.h"
#include "model_inference/postprocess/vehicle_color_postprocess.h"
#include "model_inference/postprocess/vehicle_type_postprocess.h"
#include "model_inference/postprocess/yolov3_postprocess.h"

namespace inference {
std::shared_ptr<PostProcess> PostProcess::GetInstance(std::string class_name) {
  if (class_name == "yolov3_postprocess") {
    return std::make_shared<YoloV3PostProcess>();
  } else if (class_name == "mobilenetv2_postprocess") {
    return std::make_shared<MobileNetV2PostProcess>();
  } else if (class_name == "age_gender_postprocess") {
    return std::make_shared<AgeGenderPostProcess>();
  } else if (class_name == "horizon_multitask_postprocess") {
    return std::make_shared<MultitaskPostProcess>();
  } else if (class_name == "face_quality_postprocess") {
    return std::make_shared<FaceQualityPostProcess>();
  } else if (class_name == "faceid_postprocess") {
    return std::make_shared<FaceIDPostProcess>();
  } else if (class_name == "lmks3_postprocess") {
    return std::make_shared<Lmks3PostProcess>();
  } else if (class_name == "lmks4_postprocess") {
    return std::make_shared<Lmks4PostProcess>();
  } else if (class_name == "vehicle_color_postprocess") {
    return std::make_shared<VehicleColorPostProcess>();
  } else if (class_name == "vehicle_type_postprocess") {
    return std::make_shared<VehicleTypePostProcess>();
  } else if (class_name == "plate_num_postprocess") {
    return std::make_shared<PlateNumPostProcess>();
  } else if (class_name == "gesture_postprocess") {
    return std::make_shared<GesturePostProcess>();
  } else if (class_name == "matrix_detection_postprocess") {
    return std::make_shared<MatrixDetectionPostProcess>();
  } else {
    return nullptr;
  }
}

}  // namespace inference
