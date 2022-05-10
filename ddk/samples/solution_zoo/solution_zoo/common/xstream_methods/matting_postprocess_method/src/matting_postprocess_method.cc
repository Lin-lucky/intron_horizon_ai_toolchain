/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: matting_postprocess_method.cc
 * @Brief: definition of the MattingPostProcessMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-11-26 13:36:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2029-11-26 18:23:08
 */

#include "matting_postprocess_method/matting_postprocess_method.h"
#include <string>
#include <vector>
#include <fstream>
#include "xstream/vision_type.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "hobotlog/hobotlog.hpp"
#include "bpu_predict_extension.h"

namespace xstream {

int MattingPostProcessMethod::Init(const std::string &cfg_path) {
  DnnPostProcessMethod::Init(cfg_path);
  threshold_ = config_.GetIntValue("threshold", threshold_);
  return 0;
}

int MattingPostProcessMethod::ParseDnnResult(
    DnnAsyncData &dnn_result,
    std::vector<BaseDataPtr> &frame_result) {
  LOGD << "MattingPostProcessMethod ParseDnnResult";
  frame_result.resize(1);  // matting result, represented as Segmentation
  auto matting_result = std::make_shared<xstream::BaseDataVector>();
  frame_result[0] = matting_result;

  auto &input_tensors = dnn_result.input_tensors;
  auto &output_tensors = dnn_result.output_tensors;
  for (size_t i = 0; i < output_tensors.size(); i++) {
    auto &input_tensor = input_tensors[i];
    auto &output_tensor = output_tensors[i];
    if (output_tensor.size() == 0) {
      auto segmentation = std::make_shared<Segmentation>();
      segmentation->state_ = DataState::INVALID;
      matting_result->datas_.push_back(segmentation);
      continue;
    }
    // parse valid output_tensor
    HOBOT_CHECK(output_tensor.size() == 1);  // one output layer
    HB_SYS_flushMemCache(&(output_tensor[0].data),
                         HB_SYS_MEM_CACHE_INVALIDATE);

    // postprocess
    float *trimap_float = reinterpret_cast<float *>(
        input_tensor[0].data.virAddr);
    float *bpu_out = reinterpret_cast<float *>(
        output_tensor[0].data.virAddr);
    int size = 256 * 256;
    static int result[256*256];
    for (int i = 0; i < size; i++) {
      if (trimap_float[i] >= 170) {
        result[i] = 1;
      } else if (trimap_float[i] < 85) {
        result[i] = 0;
      } else {
        result[i] = bpu_out[i];
      }
    }

    Segmentation matting;
    matting.values_ = std::vector<int>(result, result+256*256);
    auto segmentation = std::make_shared<Segmentation>(matting);
    matting_result->datas_.push_back(segmentation);
  }
  return 0;
}

}  // namespace xstream
