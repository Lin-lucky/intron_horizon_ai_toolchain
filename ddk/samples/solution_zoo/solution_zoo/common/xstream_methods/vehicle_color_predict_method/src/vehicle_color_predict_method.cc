/*
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: vehicle_color_predict_method.cc
 * @Brief: implementation of VehicleColorPredictMethod
 * @Author: shiyu.fu
 * @Email: shiyu.fu@horizon.ai
 * @Date: Wed May 12 2021 14:56:50
 */

#include "vehicle_color_predict_method/vehicle_color_predict_method.h"
#include <string.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "dnn_async_data.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/vision_type.h"

namespace xstream {

int32_t VehicleColorPredictMethod::Init(const std::string &cfg_path) {
  LOGD << "RectInputPredictMethod Init";
  DnnPredictMethod::Init(cfg_path);

  // model_path, dnn_run_with_roi is got in Init() of parent class
  std::string norm_method = config_["norm_method"].isString()
                                ? config_["norm_method"].asString()
                                : "norm_by_nothing";
  auto norm_iter = g_norm_method_map.find(norm_method);
  HOBOT_CHECK(norm_iter != g_norm_method_map.end())
      << "unknown norm method: " << norm_method;
  norm_params_.norm_method = norm_iter->second;
  norm_params_.expand_scale = config_["expand_scale"].isDouble()
                                  ? config_["expand_scale"].asFloat()
                                  : 1.0f;
  norm_params_.aspect_ratio = config_["aspect_ratio"].isDouble()
                                  ? config_["aspect_ratio"].asFloat()
                                  : 1.0f;

  std::string filter_method = config_["filter_method"].isString()
                                  ? config_["filter_method"].asString()
                                  : "no_filter";
  auto filter_iter = g_filter_method_map.find(filter_method);
  HOBOT_CHECK(filter_iter != g_filter_method_map.end())
      << "unknown filter method: " << filter_method;
  filter_method_ = filter_iter->second;

  pyramid_layer_ =
      config_["pyramid_layer"].isInt() ? config_["pyramid_layer"].asInt() : 0;
  max_handle_num_ = config_["max_handle_num"].isInt()
                        ? config_["max_handle_num"].asUInt()
                        : -1;

  return 0;
}

int VehicleColorPredictMethod::PrepareInputData(
    const std::vector<BaseDataPtr> &input, const InputParamPtr param,
    std::vector<std::vector<BPU_TENSOR_S>> &input_tensors,
    std::vector<std::vector<BPU_TENSOR_S>> &output_tensors) {
  LOGD << "MultitaskPredictMethod PrepareInputData";
  auto vehicle_boxes = std::static_pointer_cast<BaseDataVector>(input[0]);
  auto xstream_img = std::static_pointer_cast<ImageFrame>(input[1]);

  HOBOT_CHECK(xstream_img->type_ == "PyramidImageFrame")
      << "not support " << xstream_img->type_;
  auto pyramid = std::static_pointer_cast<PyramidImageFrame>(xstream_img);
  auto pyramid_height = pyramid->Height(pyramid_layer_);
  auto pyramid_width = pyramid->Width(pyramid_layer_);

  uint32_t vehicle_num = vehicle_boxes->datas_.size();
  auto handle_num = max_handle_num_ == -1
                        ? vehicle_boxes->datas_.size()
                        : std::min(max_handle_num_, vehicle_num);

  input_tensors.resize(handle_num);
  output_tensors.resize(handle_num);

  {
    RUN_PROCESS_TIME_PROFILER("VehicleColorPreProcess");
    RUN_FPS_PROFILER("VehicleColorPreProcess");
    for (size_t roi_idx = 0; roi_idx < handle_num; ++roi_idx) {
      auto roi = std::static_pointer_cast<BBox>(vehicle_boxes->datas_[roi_idx]);
      if (roi->state_ != DataState::VALID) {
        continue;
      }

      // normalize roi
      auto normed_roi = std::make_shared<BBox>();
      NormalizeRoi(roi.get(), normed_roi.get(), norm_params_, pyramid_width - 1,
                   pyramid_height - 1, filter_method_);
      LOGD << "norm roi norm_type:"
           << static_cast<int>(norm_params_.norm_method)
           << " expand_scale:" << norm_params_.expand_scale
           << " aspect_ratio:" << norm_params_.aspect_ratio
           << "  from:" << roi->x1_ << ", " << roi->y1_ << ", " << roi->x2_
           << ", " << roi->y2_ << "  to:" << normed_roi->x1_ << ", "
           << normed_roi->y1_ << ", " << normed_roi->x2_ << ", "
           << normed_roi->y2_;

      // crop and resize
      // set roi
      BPU_ROI_S input_roi;
      input_roi.x1 = normed_roi->x1_;
      input_roi.y1 = normed_roi->y1_;
      input_roi.x2 = normed_roi->x2_;
      input_roi.y2 = normed_roi->y2_;
      // set crop and resize input tensor
      BPU_TENSOR_S nv12_tensor;
      nv12_tensor.data_type = BPU_TYPE_IMG_NV12_SEPARATE;
      int h_idx, w_idx, c_idx;
      HB_BPU_getHWCIndex(nv12_tensor.data_type, nullptr, &h_idx, &w_idx,
                         &c_idx);
      nv12_tensor.data_shape.ndim = 4;
      nv12_tensor.data_shape.d[0] = 1;
      nv12_tensor.data_shape.d[h_idx] = pyramid_height;
      nv12_tensor.data_shape.d[w_idx] = pyramid_width;
      nv12_tensor.data_shape.d[c_idx] = 3;
      nv12_tensor.aligned_shape = nv12_tensor.data_shape;
      nv12_tensor.data.virAddr = reinterpret_cast<void *>(
          pyramid->img_.down_scale[pyramid_layer_].y_vaddr);
      nv12_tensor.data.phyAddr =
          pyramid->img_.down_scale[pyramid_layer_].y_paddr;
      nv12_tensor.data.memSize = pyramid_height * pyramid_width;
      nv12_tensor.data_ext.virAddr = reinterpret_cast<void *>(
          pyramid->img_.down_scale[pyramid_layer_].c_vaddr);
      nv12_tensor.data_ext.phyAddr =
          pyramid->img_.down_scale[pyramid_layer_].c_paddr;
      nv12_tensor.data_ext.memSize = pyramid_height * pyramid_width / 2;

      int ret = AllocInputTensor(input_tensors[roi_idx]);
      if (ret != 0) {
        LOGE << "ROI index: " << roi_idx << ", Alloc InputTensor failed!";
        continue;
      }

      HOBOT_CHECK(input_tensors[roi_idx].size() == 1)
          << "expect single input, got: " << input_tensors[roi_idx].size();
      HOBOT_CHECK(input_tensors[roi_idx][0].data_type =
                      BPU_TYPE_IMG_NV12_SEPARATE)
          << "not support: " << input_tensors[roi_idx][0].data_type;

      BPU_RESIZE_CTRL_S ctrl_param = {BPU_RESIZE_TYPE_BILINEAR,
                                      BPU_TYPE_IMG_NV12_SEPARATE, -1};
      BPU_TENSOR_S &resized_nv12_tensor = input_tensors[roi_idx][0];
      ret = HB_BPU_cropAndResize(&nv12_tensor, &input_roi, &resized_nv12_tensor,
                                 &ctrl_param);
      if (ret != 0) {
        LOGE << "roi index: " << roi_idx
             << ", HB_BPU_cropAndResize failed, ret: " << ret << ": "
             << HB_BPU_getErrorName(ret);
        LOGE << "input_roi (: " << input_roi.x1 << ", " << input_roi.y1 << ", "
             << input_roi.x2 << ", " << input_roi.y2 << ")";
        LOGE << "input tensor, height: " << nv12_tensor.data_shape.d[h_idx]
             << ", width: " << nv12_tensor.data_shape.d[w_idx];
        // if failed, free in dnn_predict_method
        continue;
      }
      HB_SYS_flushMemCache(&resized_nv12_tensor.data, HB_SYS_MEM_CACHE_CLEAN);
      HB_SYS_flushMemCache(&resized_nv12_tensor.data_ext,
                           HB_SYS_MEM_CACHE_CLEAN);

      ret = AllocOutputTensor(output_tensors[roi_idx]);
      if (ret != 0) {
        LOGE << "ROI index: " << roi_idx << ", Alloc OutputTensor failed!";
        continue;
      }
    }
  }

  return 0;
}
}  // namespace xstream
