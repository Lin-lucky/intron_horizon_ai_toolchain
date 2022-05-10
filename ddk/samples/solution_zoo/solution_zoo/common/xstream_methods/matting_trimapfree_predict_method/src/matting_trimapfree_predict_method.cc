/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: matting_trimapfree_predict_method.cc
 * @Brief: definition of the MattingTrimapFreePredictMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-12-15 13:15:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-12-15 16:17:08
 */

#include "matting_trimapfree_predict_method/matting_trimapfree_predict_method.h"
#include <string>
#include <string.h>
#include <vector>
#include <fstream>
#include "xstream/profiler.h"
#include "xstream/vision_type.h"
#include "image_utils.h"
#include "hobotlog/hobotlog.hpp"
#include "dnn_async_data.h"

namespace xstream {

int MattingTrimapFreePredictMethod::Init(const std::string &cfg_path) {
  DnnPredictMethod::Init(cfg_path);
  expansion_ratio_ = config_["expansion_ratio"].isNumeric() ?
                     config_["expansion_ratio"].asFloat() : expansion_ratio_;
  return 0;
}

int MattingTrimapFreePredictMethod::PrepareInputData(
      const std::vector<BaseDataPtr> &input,
      const InputParamPtr param,
      std::vector<std::vector<BPU_TENSOR_S>> &input_tensors,
      std::vector<std::vector<BPU_TENSOR_S>> &output_tensors) {
  LOGD << "MattingTrimapFreePredictMethod PrepareInputData";
  HOBOT_CHECK(input.size() == 2);  // image, box

  auto xstream_img = std::static_pointer_cast<ImageFrame>(input[0]);  // image
  auto rois = std::static_pointer_cast<BaseDataVector>(input[1]);   // body_box

  std::string img_type = xstream_img->type_;
  // CVImageFrame TODO
  HOBOT_CHECK(img_type == "PyramidImageFrame") << "not support " << img_type;

  auto pyramid_image = std::static_pointer_cast<PyramidImageFrame>(xstream_img);
  // 默认使用pym第0层，box对应0层 TODO
  int pym_layer = 0;
  const int width = pyramid_image->Width(pym_layer);
  const int height = pyramid_image->Height(pym_layer);

  int64_t box_num = rois->datas_.size();
  // 一个roi对应一次预测
  input_tensors.resize(box_num);
  output_tensors.resize(box_num);

  for (int32_t roi_idx = 0; roi_idx < box_num; roi_idx++) {
    auto roi = std::static_pointer_cast<BBox>(rois->datas_[roi_idx]);
    if (roi->state_ != xstream::DataState::VALID) {
      continue;
    }
    float src_roi_height = roi->Height() + 1;
    float src_roi_width = roi->Width() + 1;
    float expansion_height = src_roi_height * expansion_ratio_;
    float expansion_width = src_roi_width * expansion_ratio_;

    int ret = 0;
    BPU_TENSOR_S nv12_tensor;
    nv12_tensor.data_type = BPU_TYPE_IMG_NV12_SEPARATE;
    int h_idx, w_idx, c_idx;
    // 1. prepare nv12 tensor
    HB_BPU_getHWCIndex(nv12_tensor.data_type, nullptr, &h_idx, &w_idx, &c_idx);
    {
      nv12_tensor.data_shape.ndim = 4;
      nv12_tensor.data_shape.d[0] = 1;
      nv12_tensor.data_shape.d[h_idx] = height;
      nv12_tensor.data_shape.d[w_idx] = width;
      nv12_tensor.data_shape.d[c_idx] = 3;
      nv12_tensor.aligned_shape = nv12_tensor.data_shape;

      // Copy y data to data0
      #ifdef X2
      nv12_tensor.data.virAddr = reinterpret_cast<void*>(
          pyramid_image->img.down_scale[pym_layer].y_vaddr);
      nv12_tensor.data.phyAddr =
          pyramid_image->img.down_scale[pym_layer].y_paddr;
      nv12_tensor.data.memSize =  height * width;
      // Copy uv data to data_ext
      nv12_tensor.data_ext.virAddr = reinterpret_cast<void*>(
          pyramid_image->img.down_scale[pym_layer].c_vaddr);
      nv12_tensor.data_ext.phyAddr =
          pyramid_image->img.down_scale[pym_layer].c_paddr;
      nv12_tensor.data_ext.memSize =  (height + 1) / 2 * width;
      #endif
      #ifdef X3
      nv12_tensor.data.virAddr = reinterpret_cast<void*>(
          pyramid_image->down_scale[pym_layer].y_vaddr);
      nv12_tensor.data.phyAddr = pyramid_image->down_scale[pym_layer].y_paddr;
      nv12_tensor.data.memSize =  height * width;
      // Copy uv data to data_ext
      nv12_tensor.data_ext.virAddr = reinterpret_cast<void*>(
          pyramid_image->down_scale[pym_layer].c_vaddr);
      nv12_tensor.data_ext.phyAddr =
          pyramid_image->down_scale[pym_layer].c_paddr;
      nv12_tensor.data_ext.memSize =  (height + 1) / 2 * width;
      #endif
    }
    // prepare nv12 output tensor
    float expand_roi_x1 = roi->x1_ - expansion_width;
    float expand_roi_y1 = roi->y1_ - expansion_height;
    float expand_roi_x2 = roi->x2_ + expansion_width;
    float expand_roi_y2 = roi->y2_ + expansion_height;
    int expand_roi_height =
        static_cast<int>(expand_roi_y2) - static_cast<int>(expand_roi_y1) + 1;
    int expand_roi_width =
        static_cast<int>(expand_roi_x2) - static_cast<int>(expand_roi_x1) + 1;

    float resize_ratio = std::min(model_input_height_ * 1.0 / expand_roi_height,
                                  model_input_width_ * 1.0 / expand_roi_width);

    float expand_resize_height = expand_roi_height * resize_ratio;
    float expand_resize_width = expand_roi_width * resize_ratio;

    BPU_ROI_S input_roi;
    BPU_TENSOR_S nv12_resize_tensor;
    int resize_height, resize_width;
    nv12_resize_tensor.data_type = BPU_TYPE_IMG_NV12_SEPARATE;
    float h_top = 0, h_bottom = 0, w_left = 0, w_right = 0;
    {
    RUN_PROCESS_TIME_PROFILER("BPU_CropAndResize");
    {
      if (expand_roi_x1 >= 0) {
        input_roi.x1 = expand_roi_x1;
      } else {
        w_left = -expand_roi_x1;
        input_roi.x1 = 0;
      }
      if (expand_roi_y1 >= 0) {
        input_roi.y1 = expand_roi_y1;
      } else {
        h_top = -expand_roi_y1;
        input_roi.y1 = 0;
      }
      if (expand_roi_x2 < width) {
        input_roi.x2 = expand_roi_x2;
      } else {
        w_right = expand_roi_x2 - width + 1;
        input_roi.x2 = width - 1;
      }
      if (expand_roi_y2 < height) {
        input_roi.y2 = expand_roi_y2;
      } else {
        h_bottom = expand_roi_y2 - height + 1;
        input_roi.y2 = height - 1;
      }
      nv12_resize_tensor.data_shape.ndim = 4;
      nv12_resize_tensor.data_shape.d[0] = 1;

      float need_padding_height = (h_top + h_bottom) * resize_ratio;
      float need_padding_width = (w_right + w_left) * resize_ratio;

      resize_height = expand_resize_height - need_padding_height;
      resize_width = expand_resize_width - need_padding_width;

      // need AlignEven
      if (resize_height & (static_cast<int>(0X01) != 0)) {
        resize_height++;
      }
      if (resize_width & (static_cast<int>(0X01) != 0)) {
        resize_width++;
      }
      nv12_resize_tensor.data_shape.d[h_idx] = resize_height;
      nv12_resize_tensor.data_shape.d[w_idx] = resize_width;
      nv12_resize_tensor.data_shape.d[c_idx] = 3;
      nv12_resize_tensor.aligned_shape = nv12_resize_tensor.data_shape;
      ret = HB_SYS_bpuMemAlloc(
          "in_data0", resize_height*resize_width,
          true, &nv12_resize_tensor.data);
      if (ret != 0) {
        LOGE << "alloc bpu mem failed, ret: " << ret << ": "
             << HB_BPU_getErrorName(ret);
        continue;
      }
      ret = HB_SYS_bpuMemAlloc(
          "in_data1", resize_height/2*resize_width,
          true, &nv12_resize_tensor.data_ext);
      if (ret != 0) {
        HB_SYS_bpuMemFree(&nv12_resize_tensor.data);
        LOGE << "alloc bpu mem failed, ret: " << ret << ": "
             << HB_BPU_getErrorName(ret);
        continue;
      }
    }

    BPU_RESIZE_CTRL_S ctrl_param = {
        BPU_RESIZE_TYPE_BILINEAR,
        BPU_TYPE_IMG_NV12_SEPARATE,
        -1};
    ret = HB_BPU_cropAndResize(
        &nv12_tensor, &input_roi, &nv12_resize_tensor, &ctrl_param);
    if (ret != 0) {
      LOGE << "box index: " << roi_idx
           << ", HB_BPU_cropAndResize failed, ret: " << ret << ": "
           << HB_BPU_getErrorName(ret);
      LOGE << "input_roi (x1,y1,x2,y2): "
           << input_roi.x1 << ", " << input_roi.y1 << ", "
           << input_roi.x2 << ", " << input_roi.y2;
      LOGE << "nv12_resize_tensor.shape, height: "
           << nv12_resize_tensor.data_shape.d[h_idx]
           << ", width: " << nv12_resize_tensor.data_shape.d[w_idx];
      LOGE << "nv12_tensor.shape, height: " << nv12_tensor.data_shape.d[h_idx]
           << ", width: " << nv12_tensor.data_shape.d[w_idx];
      // release nv12_resize_tensor
      HB_SYS_bpuMemFree(&nv12_resize_tensor.data);
      HB_SYS_bpuMemFree(&nv12_resize_tensor.data_ext);
      continue;
    }
    }
    // copy nv12_resize_tensor to nv12
    const uint8_t* input_roi_yuv_data[3] = {
        reinterpret_cast<uint8_t *>(nv12_resize_tensor.data.virAddr),
        reinterpret_cast<uint8_t *>(nv12_resize_tensor.data_ext.virAddr),
        nullptr};
    const int input_roi_yuv_size[3] = {
        resize_height * resize_width, resize_height * resize_width / 2, 0};
    uint8_t *padding_nv12_data = nullptr;
    int padding_size, padding_height, padding_width;
    int first_stride, second_stride;

    // need padding to model_input_size[such as: 512x512]
    int top_x = -w_left*resize_ratio;
    if (top_x & (static_cast<int>(0X01) != 0)) {
      top_x--;
    }
    int top_y = -h_top*resize_ratio;
    if (top_y & (static_cast<int>(0X01) != 0)) {
      top_y--;
    }
    int bottom_x = top_x + model_input_width_ - 1;
    int bottom_y = top_y + model_input_height_ - 1;
    {
    RUN_PROCESS_TIME_PROFILER("padding_nv12");
    ret = HobotXStreamCropYuvImageWithPaddingBlack(
        input_roi_yuv_data, input_roi_yuv_size,
        resize_width, resize_height, resize_width, resize_width,
        IMAGE_TOOLS_RAW_YUV_NV12,
        top_x, top_y,
        bottom_x, bottom_y,
        &padding_nv12_data, &padding_size,
        &padding_width, &padding_height,
        &first_stride, &second_stride);
    // release nv12_resize_tensor
    HB_SYS_bpuMemFree(&nv12_resize_tensor.data);
    HB_SYS_bpuMemFree(&nv12_resize_tensor.data_ext);
    if (ret != 0) {
      LOGE << "crop and padding nv12 failed, ret: " << ret;
      LOGE << "input size: " << resize_height << " x " << resize_width;
      LOGE << "crop scope: (" << top_x << ", " << top_y
           << ", " << bottom_x << ", " << bottom_y << ")";
      LOGE << "output size: " << padding_height << "x" << padding_width;
      continue;
    }
    }

    ret = AllocInputTensor(input_tensors[roi_idx]);
    if (ret != 0) {
      LOGE << "ROI index: " << roi_idx << ", Alloc InputTensor failed!";
      continue;
    }
    ret = AllocOutputTensor(output_tensors[roi_idx]);
    if (ret != 0) {
      LOGE << "ROI index: " << roi_idx << ", Alloc OutputTensor failed!";
      continue;
    }
    // copy input data to input_tensors
    HOBOT_CHECK(input_tensors[roi_idx].size() == 1) << "expected input num: 1";
    {
      // model input_size: 1x512x512x3, NHWC
      BPU_TENSOR_S &tensor = input_tensors[roi_idx][0];
      HOBOT_CHECK(tensor.data_type == BPU_TYPE_IMG_NV12_SEPARATE);

      HOBOT_CHECK(model_input_height_ == padding_height &&
                  model_input_width_ == padding_width);

      uint8_t* src = padding_nv12_data;
      // copy y data
      memcpy(tensor.data.virAddr, src,
             model_input_height_ * model_input_width_);
      // copy uv data
      src = padding_nv12_data + model_input_height_ * model_input_width_;
      memcpy(tensor.data_ext.virAddr, src,
             model_input_height_ * model_input_width_ / 2);
      HobotXStreamFreeImage(padding_nv12_data);
      HB_SYS_flushMemCache(&tensor.data, HB_SYS_MEM_CACHE_CLEAN);
      HB_SYS_flushMemCache(&tensor.data_ext, HB_SYS_MEM_CACHE_CLEAN);
    }
  }
  return 0;
}
}  // namespace xstream
