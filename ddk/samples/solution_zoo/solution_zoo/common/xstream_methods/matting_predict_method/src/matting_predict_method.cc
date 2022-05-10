/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: matting_predict_method.cc
 * @Brief: definition of the MattingPredictMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-11-23 11:15:05
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-12-14 16:17:08
 */

#include "matting_predict_method/matting_predict_method.h"
#include <string>
#include <vector>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include "xstream/profiler.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "xstream/vision_type.h"
#include "image_utils.h"
#include "hobotlog/hobotlog.hpp"
#include "dnn_async_data.h"

namespace xstream {

int MattingPredictMethod::Init(const std::string &cfg_path) {
  DnnPredictMethod::Init(cfg_path);
  erode_kernel_ = config_["erode_kernel"].isInt() ?
                  config_["erode_kernel"].asInt() : erode_kernel_;
  dilate_kernel_ = config_["dilate_kernel"].isInt() ?
                   config_["dilate_kernel"].asInt() : dilate_kernel_;
  return 0;
}

int MattingPredictMethod::PrepareInputData(
      const std::vector<BaseDataPtr> &input,
      const InputParamPtr param,
      std::vector<std::vector<BPU_TENSOR_S>> &input_tensors,
      std::vector<std::vector<BPU_TENSOR_S>> &output_tensors) {
  LOGD << "MattingPredictMethod PrepareInputData";
  HOBOT_CHECK(input.size() == 4);  // image, box, kps, mask

  auto xstream_img = std::static_pointer_cast<ImageFrame>(input[0]);  // image
  auto rois = std::static_pointer_cast<BaseDataVector>(input[1]);   // body_box
  auto kpses = std::static_pointer_cast<BaseDataVector>(input[2]);  // body_kps
  auto masks = std::static_pointer_cast<BaseDataVector>(input[3]);  // body_mask

  std::string img_type = xstream_img->type_;
  // CVImageFrame TODO
  // cv::Mat rgb_img;
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
    auto roi = std::static_pointer_cast<BBox>(
        rois->datas_[roi_idx]);
    if (roi->state_ != xstream::DataState::VALID) {  // TODO(sz) track_id过滤
      continue;
    }

    // 2. kps
    auto kps = std::static_pointer_cast<Landmarks>(
        kpses->datas_[roi_idx]);
    if (kps->state_ != xstream::DataState::VALID) {
      continue;
    }

    // 3. mask
    auto mask = std::static_pointer_cast<Segmentation>(
        masks->datas_[roi_idx]);
    if (mask->state_ != xstream::DataState::VALID) {
      continue;
    }

    int ret = 0;
    // model input_size: 1x4x256x256, NCHW
    // 1. prepare rgb_roi_data
    // prepare nv12 tensor
    BPU_TENSOR_S nv12_tensor;
    nv12_tensor.data_type = BPU_TYPE_IMG_NV12_SEPARATE;
    int h_idx, w_idx, c_idx;
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
    float roi_height = roi->Height();
    float roi_width = roi->Width();
    float roi_x1 = roi->x1_ - 16 * roi_width / 224;
    float roi_y1 = roi->y1_ - 16 * roi_height / 224;
    float roi_x2 = roi->x2_ + 16 * roi_width/ 224;
    float roi_y2 = roi->y2_ + 16 * roi_height / 224;
    BPU_ROI_S input_roi;
    BPU_TENSOR_S nv12_resize_tensor;
    int resize_height, resize_width;
    nv12_resize_tensor.data_type = BPU_TYPE_IMG_NV12_SEPARATE;
    float h_top = 0, h_bottom = 0, w_left = 0, w_right = 0;
    {
    RUN_PROCESS_TIME_PROFILER("BPU_CropAndResize");
    {
      if (roi_x1 >= 0) {
        input_roi.x1 = roi_x1;
      } else {
        w_left = -roi_x1;
        input_roi.x1 = 0;
      }
      if (roi_y1 >= 0) {
        input_roi.y1 = roi_y1;
      } else {
        h_top = -roi_y1;
        input_roi.y1 = 0;
      }
      if (roi_x2 < width) {
        input_roi.x2 = roi_x2;
      } else {
        w_right = roi_x2 - width + 1;
        input_roi.x2 = width - 1;
      }
      if (roi_y2 < height) {
        input_roi.y2 = roi_y2;
      } else {
        h_bottom = roi_y2 - height + 1;
        input_roi.y2 = height - 1;
      }

      nv12_resize_tensor.data_shape.ndim = 4;
      nv12_resize_tensor.data_shape.d[0] = 1;
      resize_height = 256 - (h_top + h_bottom) * 224.0 / roi_height;
      if (resize_height & (static_cast<int>(0X01) != 0)) {
        resize_height++;
      }
      resize_width = 256 - (w_right + w_left) * 224.0 / roi_width;
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
    uint8_t *padding_data = nullptr;
    int padding_size = 0, padding_width = 0, padding_height = 0;
    int padding_stride1 = 0, padding_stride2 = 0;
    {
    RUN_PROCESS_TIME_PROFILER("padding_nv12");
    int top_x = -w_left * 224.0 / roi_width;
    int top_y = -h_top * 224.0 / roi_height;
    if (top_x & (static_cast<int>(0X01) != 0)) {
      top_x--;
    }
    if (top_y & (static_cast<int>(0X01) != 0)) {
      top_y--;
    }
    int bottom_x = 255 + top_x, bottom_y = 255 + top_y;
    ret = HobotXStreamCropYuvImageWithPaddingBlack(
        input_roi_yuv_data, input_roi_yuv_size,
        resize_width, resize_height, resize_width, resize_width,
        IMAGE_TOOLS_RAW_YUV_NV12,
        top_x, top_y, bottom_x, bottom_y,
        &padding_data, &padding_size, &padding_width, &padding_height,
        &padding_stride1, &padding_stride2);
    // release nv12_resize_tensor
    HB_SYS_bpuMemFree(&nv12_resize_tensor.data);
    HB_SYS_bpuMemFree(&nv12_resize_tensor.data_ext);
    if (ret != 0) {
      LOGE << "padding nv12 failed, roi_index: " << roi_idx;
      continue;
    }
    // debug dump nv12
    #if 0
      std::fstream outfile;
      static int index_nv12 = 0;
      outfile.open("nv12_" + std::to_string(index_nv12++) + ".nv12",
                   std::ios_base::out|std::ios_base::trunc|std::ios::binary);
      outfile.write(reinterpret_cast<char*>(padding_data), padding_size);
      outfile.close();
    #endif
    }

    cv::Mat ske_kps(256, 256, CV_8UC1, cv::Scalar::all(0));
    int roi_origin_x = roi->x1_;
    int roi_origin_y = roi->y1_;
    float ratio_x = 224.0 / roi->Width();
    float ratio_y = 224.0 / roi->Height();
    LOGD << "box " << roi_idx << " x1: " << roi_origin_x
         << ", y1: " << roi_origin_y;
    LOGD << "box " << roi_idx << " width: " << roi->Width()
         << ", height: " << roi->Height();

    std::vector<std::vector<int>> kps_index = {
        {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12},
        {5, 11}, {6, 12}, {5, 6}, {5, 7}, {6, 8}, {7, 9},
        {8, 10}, {1, 2}, {0, 1}, {0, 2}, {1, 3}, {2, 4}};
    for (size_t i = 0; i < kps_index.size(); i++) {
      int start_index = kps_index[i][0];
      int end_index = kps_index[i][1];
      cv::line(
          ske_kps,
          cv::Point(
              (kps->values_[start_index].x_ - roi_origin_x) * ratio_x + 16,
              (kps->values_[start_index].y_ - roi_origin_y) * ratio_y + 16),
          cv::Point(
              (kps->values_[end_index].x_ - roi_origin_x) * ratio_x + 16,
              (kps->values_[end_index].y_ - roi_origin_y) * ratio_y + 16),
          cv::Scalar::all(255), 1, CV_AA);
    }

    ske_kps.convertTo(ske_kps, CV_32FC1);

    int h_w = sqrt(mask->values_.size());
    cv::Mat mask_224(h_w, h_w, CV_32SC1, mask->values_.data());
    cv::resize(mask_224, mask_224, cv::Size(224, 224));
    cv::Mat mask_mat(256, 256, CV_8UC1, cv::Scalar(0));
    mask_224.copyTo(mask_mat(cv::Rect(16, 16, 224, 224)));

    cv::Mat gray_mask;    // gray_mask: bg 0, fg 255
    cv::compare(mask_mat, 0, gray_mask, cv::CMP_GT);

    cv::Mat big_k = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(erode_kernel_, erode_kernel_));
    cv::Mat small_k = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(dilate_kernel_, dilate_kernel_));
    cv::Mat fg_mask, bg_mask, unknown_mask;
    cv::erode(gray_mask, fg_mask, big_k);     // 腐蚀
    cv::dilate(gray_mask, bg_mask, small_k);  // 膨胀
    fg_mask.convertTo(fg_mask, CV_32FC1);
    bg_mask.convertTo(bg_mask, CV_32FC1);
    unknown_mask = (bg_mask - fg_mask) / 255.0 * 128;

    cv::Mat trimap = fg_mask + unknown_mask + ske_kps;
    // cv::min(max(trimap, 0), 255, trimap);  // clip(0, 255)
    trimap.convertTo(trimap, CV_8UC1);
    trimap.convertTo(trimap, CV_32FC1);

    // 3. rgb_split(256x256x3) + trimap(256x256), NCHW
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
    HOBOT_CHECK(input_tensors[roi_idx].size() == 2) << "expected input num: 2";
    {
      // nv12 data
      BPU_TENSOR_S &tensor1 = input_tensors[roi_idx][1];
      HOBOT_CHECK(tensor1.data_type == BPU_TYPE_IMG_NV12_SEPARATE);
      HOBOT_CHECK(model_input_height_ == padding_height &&
                  model_input_width_ == padding_width);
      uint8_t* src = padding_data;
      // copy y data
      memcpy(tensor1.data.virAddr, src,
             model_input_height_ * model_input_width_);
      // copy uv data
      src = padding_data + model_input_height_ * model_input_width_;
      memcpy(tensor1.data_ext.virAddr, src,
             model_input_height_ * model_input_width_ >> 1);
      HobotXStreamFreeImage(padding_data);
      HB_SYS_flushMemCache(&tensor1.data, HB_SYS_MEM_CACHE_CLEAN);
      HB_SYS_flushMemCache(&tensor1.data_ext, HB_SYS_MEM_CACHE_CLEAN);

      // trimap data
      BPU_TENSOR_S &tensor0 = input_tensors[roi_idx][0];
      memcpy(tensor0.data.virAddr, trimap.data, 256*256*4);
      HB_SYS_flushMemCache(&tensor0.data, HB_SYS_MEM_CACHE_CLEAN);
    }
  }
  return 0;
}
}  // namespace xstream
