/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: matting_trimapfree_postprocess_method.cc
 * @Brief: definition of the MattingTrimapFreePostProcessMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-12-15 20:06:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-12-15 20:43:08
 */

#include "matting_trimapfree_postprocess_method/matting_trimapfree_postprocess_method.h"
#include <string>
#include <vector>
#include <fstream>
#include "xstream/profiler.h"
#include "dnn_util.h"
#include "xstream/vision_type.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "hobotlog/hobotlog.hpp"

namespace xstream {

int MattingTrimapFreePostProcessMethod::Init(const std::string &cfg_path) {
  DnnPostProcessMethod::Init(cfg_path);
  matting_low_thresh_ =
      config_.GetFloatValue("matting_low_thresh", matting_low_thresh_);
  matting_high_thresh_ =
      config_.GetFloatValue("matting_high_thresh", matting_high_thresh_);
  return 0;
}

int MattingTrimapFreePostProcessMethod::ParseDnnResult(
    DnnAsyncData &dnn_result,
    std::vector<BaseDataPtr> &frame_result) {
  LOGD << "MattingTrimapFreePostProcessMethod ParseDnnResult";
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
    // parse valid output_tensor: 3层 1x512x512x1 int8 NHWC
    HOBOT_CHECK(input_tensor.size() == 1);  // one input layer
    HOBOT_CHECK(output_tensor.size() == 3);  // three output layer
    std::vector<std::vector<float>> out_datas(3);

    // 需要移位(与模型有关, 和算法确认即可)
    // 刷新flush
    for (size_t i = 0; i < 3; i++) {
      HB_SYS_flushMemCache(&(output_tensor[i].data),
                           HB_SYS_MEM_CACHE_INVALIDATE);
    }
    int model_output_height, model_output_width;
    HB_BPU_getHW(dnn_result.dnn_model->bpu_model.outputs[0].data_type,
                 &dnn_result.dnn_model->bpu_model.outputs[0].shape,
                 &model_output_height, &model_output_width);

    for (size_t i = 0; i < 3; i++) {
      int model_out_size =
          dnn_result.dnn_model->bpu_model.outputs[i].shape.d[0] *
          dnn_result.dnn_model->bpu_model.outputs[i].shape.d[1] *
          dnn_result.dnn_model->bpu_model.outputs[i].shape.d[2] *
          dnn_result.dnn_model->bpu_model.outputs[i].shape.d[3];
      out_datas[i].resize(model_out_size);
      RUN_PROCESS_TIME_PROFILER("Convert_Float");
      ConvertOutputToFloat(
          output_tensor[i].data.virAddr, out_datas[i].data(),
          dnn_result.dnn_model->bpu_model, i);
    }

    // postprocess
    cv::Mat weighted_fg, weighted_bg, matting_pred;
    cv::Mat fg(model_output_height, model_output_width,
               CV_32FC1, out_datas[0].data());
    cv::Mat bg(model_output_height, model_output_width,
               CV_32FC1, out_datas[1].data());
    cv::Mat fusion_mask(model_output_height, model_output_width,
                        CV_32FC1, out_datas[2].data());
    cv::multiply(fg, fusion_mask, weighted_fg);
    cv::multiply(1.0 - bg, 1.0 - fusion_mask, weighted_bg);
    cv::Mat matting_res;
    matting_res = weighted_fg + weighted_bg;
    matting_res *= 255;
    for (int i = 0; i < matting_res.rows; i++) {
       float* data = matting_res.ptr<float>(i);
       for (int j = 0; j < matting_res.cols; j++) {
         if (data[j] <= matting_low_thresh_) {
          data[j] = 0;
        } else if (data[j] >= matting_high_thresh_) {
          data[j] = 255;
        }
       }
    }
    matting_res.convertTo(matting_res, CV_32SC1);

    Segmentation matting;
    matting.values_ = std::vector<int>(matting_res.reshape(1, 1));
    matting.width_ = model_output_width;
    matting.height_ = model_output_height;
    auto segmentation = std::make_shared<Segmentation>(matting);
    matting_result->datas_.push_back(segmentation);
  }
  return 0;
}

}  // namespace xstream
