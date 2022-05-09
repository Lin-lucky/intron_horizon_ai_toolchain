/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of preprocess
 * @file   preprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include <string>
#include "model_inference/preprocess/preprocess.h"
#include "model_inference/preprocess/pyramid_preprocess.h"
#include "model_inference/preprocess/pyramid_roi_resizer_preprocess.h"
#include "model_inference/preprocess/pyramid_roi_bpu_preprocess.h"
#include "model_inference/preprocess/pyramid_roi_preprocess.h"
#include "model_inference/preprocess/image_preprocess.h"
#include "model_inference/preprocess/faceid_preprocess.h"
#include "model_inference/preprocess/gesture_preprocess.h"

namespace inference {
std::shared_ptr<PreProcess> PreProcess::GetInstance(
    std::string class_name, Inferencer* infer) {
  if (class_name == "pyramid_preprocess") {
    return std::make_shared<PyramidPreProcess>(infer);
  } else if (class_name == "pyramid_roi_resizer_preprocess") {
    return std::make_shared<PyramidRoiResizerPreProcess>(infer);
  } else if (class_name == "pyramid_roi_bpu_preprocess") {
    return std::make_shared<PyramidRoiBPUPreProcess>(infer);
  } else if (class_name == "pyramid_roi_preprocess") {
    return std::make_shared<PyramidRoiPreProcess>(infer);
  } else if (class_name == "image_preprocess") {
    return std::make_shared<ImageInputWithPreProcess>(infer);
  } else if (class_name == "faceid_preprocess") {
    return std::make_shared<FaceIDPreProcess>(infer);
  } else if (class_name == "gesture_preprocess") {
    return std::make_shared<GesturePreProcess>(infer);
  } else {
    return nullptr;
  }
}

}  // namespace inference

