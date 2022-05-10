/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: MultitaskPredictMethod.cpp
 * @Brief: definition of the MultitaskPredictMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-12-03 15:45:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2029-12-03 18:17:08
 */

#include "multitask_predict_method/multitask_predict_method.h"
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <string.h>
#include <vector>
#include "dnn_async_data.h"
#include "hobotlog/hobotlog.hpp"
#include "image_utils.h"
#include "xstream/vision_type.h"

namespace xstream {

int32_t MultitaskPredictMethod::Init(const std::string &cfg_path) {
  LOGD << "MultitaskPredictMethod Init";
  DnnPredictMethod::Init(cfg_path);

  // 获取金字塔层数
  pyramid_layer_ = config_["pyramid_layer"].isInt()
                       ? config_["pyramid_layer"].asInt()
                       : pyramid_layer_;
  model_input_width_ = config_["model_input_width"].isInt()
                           ? config_["model_input_width"].asInt()
                           : model_input_width_;
  model_input_height_ = config_["model_input_height"].isInt()
                            ? config_["model_input_height"].asInt()
                            : model_input_height_;
  LOGD << "pyramid layer: " << pyramid_layer_
       << ", model input width: " << model_input_width_
       << ", model input height: " << model_input_height_;
  return 0;
}

int MultitaskPredictMethod::GetSrcImageSize(
    const std::vector<BaseDataPtr> &input, int &src_image_height,
    int &src_image_width) {
  HOBOT_CHECK(input.size() == 1);  // image
  auto xstream_img = std::static_pointer_cast<ImageFrame>(input[0]);

  std::string img_type = xstream_img->type_;
  if (img_type == "PyramidImageFrame") {
    auto pyramid_image =
        std::static_pointer_cast<PyramidImageFrame>(xstream_img);

    src_image_height = pyramid_image->img_.down_scale[0].height;
    src_image_width = pyramid_image->img_.down_scale[0].width;

  } else if (img_type == "RawDataImageFrame") {
    auto image = std::static_pointer_cast<RawDataImageFrame>(xstream_img);
    src_image_height = image->Height();
    src_image_width = image->Width();
  } else {
    LOGE << "not support " << img_type;
    return -1;
  }
  LOGD << "src image height: " << src_image_height
       << ", src image width: " << src_image_width;
  return 0;
}

int MultitaskPredictMethod::PrepareInputData(
    const std::vector<BaseDataPtr> &input,
    const InputParamPtr param,
    std::vector<std::vector<BPU_TENSOR_S>> &input_tensors,
    std::vector<std::vector<BPU_TENSOR_S>> &output_tensors) {
  LOGD << "MultitaskPredictMethod PrepareInputData";
  HOBOT_CHECK(input.size() == 1);  // image
  auto xstream_img = std::static_pointer_cast<ImageFrame>(input[0]);

  std::string img_type = xstream_img->type_;

  // 全图检测,仅做一次预测
  input_tensors.resize(1);
  output_tensors.resize(1);
  // CVImageFrame TODO
  if (img_type == "PyramidImageFrame") {
    auto pyramid_image = std::static_pointer_cast<PyramidImageFrame>(
        xstream_img);

    int target_pym_layer_height =
        pyramid_image->img_.down_scale[pyramid_layer_].height;
    int target_pym_layer_width =
        pyramid_image->img_.down_scale[pyramid_layer_].width;

    int ret = 0;
    uint8_t *pOutputImg = nullptr;  // padding_img_data
    int output_img_size = 0;

    if (model_input_height_ != target_pym_layer_height ||
        model_input_width_ != target_pym_layer_width) {
      // get pym level 4 (960*540), then pad to 960*544
      // int output_img_size = 0;
      int output_img_width = 0;
      int output_img_height = 0;
      int first_stride = 0;
      int second_stride = 0;

      auto input_img = pyramid_image->img_.down_scale[pyramid_layer_];
      int y_size = input_img.height * input_img.step;
      int uv_size = y_size >> 1;
      const uint8_t *input_nv12_data[3] = {
          reinterpret_cast<uint8_t *>(input_img.y_vaddr),
          reinterpret_cast<uint8_t *>(input_img.c_vaddr), nullptr};
      const int input_nv12_size[3] = {y_size, uv_size, 0};
      ret = HobotXStreamCropYuvImageWithPaddingBlack(
          input_nv12_data, input_nv12_size, input_img.width, input_img.height,
          input_img.step, input_img.step, IMAGE_TOOLS_RAW_YUV_NV12, 0, 0,
          model_input_width_ - 1, model_input_height_ - 1, &pOutputImg,
          &output_img_size, &output_img_width, &output_img_height,
          &first_stride, &second_stride);

      if (ret < 0) {
        LOGE << "fail to crop image";
        free(pOutputImg);
        return -1;
      }
      HOBOT_CHECK(output_img_width == model_input_width_)
          << "cropped image width " << output_img_width
          << " not equals to model input width " << model_input_width_;
      HOBOT_CHECK(output_img_height == model_input_height_)
          << "cropped image height " << output_img_height
          << " not equals to model input height " << model_input_height_;
    }  // end padding image

    // 1. alloc input_tensors
    ret = AllocInputTensor(input_tensors[0]);
    if (ret != 0) {
      LOGE << "Alloc InputTensor failed!";
      return -1;
    }
    // 2. alloc output_tensors
    ret = AllocOutputTensor(output_tensors[0]);
    if (ret != 0) {
      LOGE << "Alloc OutputTensor failed!";
      return -1;
    }
    // 3. copy data to input_tensors
    HOBOT_CHECK(input_tensors[0].size() == 1);  // 1层输入
    HOBOT_CHECK(input_tensors[0][0].data_type == BPU_TYPE_IMG_NV12_SEPARATE);
    {
      int img_len = model_input_width_ * model_input_height_ * 3 / 2;
      uint8_t *input_y_data, *input_uv_data;

      BPU_TENSOR_S &tensor = input_tensors[0][0];
      int height = tensor.data_shape.d[input_h_idx_];
      int width = tensor.data_shape.d[input_w_idx_];
      int stride = tensor.aligned_shape.d[input_w_idx_];

      if (pOutputImg == nullptr) {  // pyramid data
        input_y_data = reinterpret_cast<uint8_t *>(
            pyramid_image->img_.down_scale[pyramid_layer_].y_vaddr);
        input_uv_data = reinterpret_cast<uint8_t *>(
            pyramid_image->img_.down_scale[pyramid_layer_].c_vaddr);
      } else {  // padding data
        HOBOT_CHECK(img_len == output_img_size);
        input_y_data = pOutputImg;
        input_uv_data = pOutputImg + model_input_width_ * model_input_height_;
      }

      // Copy y data to data0
      uint8_t *y = reinterpret_cast<uint8_t *>(tensor.data.virAddr);
      for (int h = 0; h < height; ++h) {
        auto *raw = y + h * stride;
        memcpy(raw, input_y_data, width);
        input_y_data += width;
      }
      HB_SYS_flushMemCache(&tensor.data, HB_SYS_MEM_CACHE_CLEAN);

      // Copy uv data to data_ext
      uint8_t *uv = reinterpret_cast<uint8_t *>(tensor.data_ext.virAddr);
      int uv_height = height / 2;
      for (int i = 0; i < uv_height; ++i) {
        auto *raw = uv + i * stride;
        memcpy(raw, input_uv_data, width);
        input_uv_data += width;
      }
      HB_SYS_flushMemCache(&tensor.data_ext, HB_SYS_MEM_CACHE_CLEAN);
    }

    if (pOutputImg != nullptr) {  // padding data
      LOGW << "about to call HobotXStreamFreeImage";
      HobotXStreamFreeImage(pOutputImg);
      LOGW << "HobotXStreamFreeImage Succeed";
    }
  }
  return 0;
}
}  // namespace xstream
