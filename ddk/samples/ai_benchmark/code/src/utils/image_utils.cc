// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "utils/image_utils.h"

#include <fstream>
#include <iostream>
#include <string>

#include "glog/logging.h"
#include "utils/data_transformer.h"

void short_side_resize(cv::Mat &output_mat,
                       cv::Mat &input_mat,
                       int short_size) {
  int resize_height = short_size;
  int resize_width = short_size;
  int ori_width = input_mat.cols;
  int ori_height = input_mat.rows;
  if (ori_width > ori_height) {
    resize_width = (ori_width / ori_height) * short_size;
    output_mat.create(resize_height, resize_width, input_mat.type());
  } else {
    resize_height = (ori_height / ori_width) * short_size;
    output_mat.create(resize_width, resize_height, input_mat.type());
  }
  cv::resize(input_mat, output_mat, output_mat.size(), 0, 0);
}

void centor_crop(cv::Mat &output_mat, cv::Mat &input_mat, int crop_size) {
  cv::Rect crop_info;
  crop_info.x = input_mat.cols / 2 - crop_size / 2;
  crop_info.y = input_mat.rows / 2 - crop_size / 2;
  crop_info.width = crop_size;
  crop_info.height = crop_size;
  cv::Mat roi(input_mat, crop_info);
  roi.copyTo(output_mat);
}

void padding_resize(ImageTensor *image_tensor,
                    cv::Mat &output_mat,
                    cv::Mat &input_mat) {
  int input_height = input_mat.rows;
  int input_width = input_mat.cols;
  int target_height = output_mat.rows;
  int target_width = output_mat.cols;
  float scale = std::min(target_width * 1.0 / input_width,
                         target_height * 1.0 / input_height);

  int resize_height = scale * input_height;
  int resize_width = scale * input_width;
  resize_height = resize_height % 2 == 0 ? resize_height : resize_height + 1;
  resize_width = resize_width % 2 == 0 ? resize_width : resize_width + 1;

  cv::Mat resize_mat;
  resize_mat.create(resize_height, resize_width, input_mat.type());
  cv::resize(input_mat, resize_mat, resize_mat.size(), 0, 0);

  cv::copyMakeBorder(resize_mat,
                     output_mat,
                     (target_height - resize_height) / 2,
                     (target_height - resize_height) / 2,
                     (target_width - resize_width) / 2,
                     (target_width - resize_width) / 2,
                     cv::BORDER_CONSTANT,
                     cv::Scalar(127, 127, 127));
  image_tensor->is_pad_resize = true;
}

void padded_center_crop(cv::Mat &output_mat,
                        cv::Mat &input_mat,
                        float target_size,
                        int crop_pad) {
  int input_height = input_mat.rows;
  int input_width = input_mat.cols;
  int short_size = std::min(input_height, input_width);
  float scale = target_size / (target_size + crop_pad);
  float padded_center_crop_size = scale * short_size;
  int offset_height = ((input_height - padded_center_crop_size) + 1) / 2;
  int offset_width = ((input_width - padded_center_crop_size) + 1) / 2;

  cv::Rect crop_info;
  crop_info.x = offset_width;
  crop_info.y = offset_height;
  crop_info.width = input_height - 2 * offset_width;
  crop_info.height = input_height - 2 * offset_height;
  cv::Mat roi(input_mat, crop_info);
  roi.copyTo(output_mat);
}

void bgr_to_nv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);

  uint8_t *yuv = yuv_mat.ptr<uint8_t>();
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  uint8_t *ynv12 = img_nv12.ptr<uint8_t>();

  int uv_height = height / 2;
  int uv_width = width / 2;

  // copy y data
  int y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
}

int draw_perception(ImageTensor *frame, Perception *perception, cv::Mat &mat) {
  // ori_image_path is non-empty only in image_list input mode
  if (frame->ori_image_path.empty()) {
    if (image_tensor_to_mat(frame, mat) != 0) {
      return -1;
    }
  } else {
    mat = cv::imread(frame->ori_image_path);
  }

  static cv::Scalar colors[] = {
      cv::Scalar(255, 0, 0),     // red
      cv::Scalar(255, 165, 0),   // orange
      cv::Scalar(255, 255, 0),   // yellow
      cv::Scalar(0, 255, 0),     // green
      cv::Scalar(0, 0, 255),     // blue
      cv::Scalar(75, 0, 130),    // indigo
      cv::Scalar(238, 130, 238)  // violet
  };
  static uint8_t bgr_putpalette[] = {
      128, 64,  128, 244, 35,  232, 70,  70,  70,  102, 102, 156, 190, 153, 153,
      153, 153, 153, 250, 170, 30,  220, 220, 0,   107, 142, 35,  152, 251, 152,
      0,   130, 180, 220, 20,  60,  255, 0,   0,   0,   0,   142, 0,   0,   70,
      0,   60,  100, 0,   80,  100, 0,   0,   230, 119, 11,  32};

  if (perception->type == Perception::DET) {
    auto &det = perception->det;
    for (int i = 0; i < det.size(); i++) {
      auto &color = colors[det[i].id % 7];
      Bbox &bbox = det[i].bbox;
      auto w_base = perception->w_base;
      auto h_base = perception->h_base;
      cv::rectangle(mat,
                    cv::Point(bbox.xmin * w_base, bbox.ymin * h_base),
                    cv::Point(bbox.xmax * w_base, bbox.ymax * h_base),
                    color);
      std::stringstream text_ss;
      std::string class_name =
          det[i].class_name == nullptr ? "empty" : det[i].class_name;
      text_ss << det[i].id << " " << class_name << ":" << std::fixed
              << std::setprecision(4) << det[i].score;
      cv::putText(
          mat,
          text_ss.str(),
          cv::Point(bbox.xmin * w_base, std::abs(bbox.ymin * h_base - 5)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.5,
          color,
          1,
          cv::LINE_AA);
    }
  } else if (perception->type == Perception::MASK) {
    auto &det = perception->mask.det_info;
    for (int i = 0; i < det.size(); i++) {
      auto &color = colors[det[i].id % 7];
      Bbox &bbox = det[i].bbox;
      auto w_base = perception->mask.w_base;
      auto h_base = perception->mask.h_base;
      cv::rectangle(mat,
                    cv::Point(bbox.xmin * w_base, bbox.ymin * h_base),
                    cv::Point(bbox.xmax * w_base, bbox.ymax * h_base),
                    color);
      std::stringstream text_ss;
      std::string class_name =
          det[i].class_name == nullptr ? "empty" : det[i].class_name;
      text_ss << det[i].id << " " << class_name << ":" << std::fixed
              << std::setprecision(4) << det[i].score;
      cv::putText(
          mat,
          text_ss.str(),
          cv::Point(bbox.xmin * w_base, std::abs(bbox.ymin * h_base - 5)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.5,
          color,
          1,
          cv::LINE_AA);
    }
  } else if (perception->type == Perception::SEG) {
    auto result_ptr = perception->seg.seg.data();
    int parsing_width = perception->seg.width;
    int parsing_height = perception->seg.height;

    cv::Mat parsing_img(parsing_height, parsing_width, CV_8UC3);
    uint8_t *parsing_img_ptr = parsing_img.ptr<uint8_t>();
    auto w_base = perception->w_base;
    auto h_base = perception->h_base;
    // set parsing bgr
    for (int i = 0; i < parsing_height; ++i) {
      for (int j = 0; j < parsing_width; ++j) {
        int src_h = i * h_base;
        int src_w = j * w_base;
        int8_t id = result_ptr[src_h * parsing_width + src_w];
        if (id >= 19) continue;
        *parsing_img_ptr++ = bgr_putpalette[id * 3];
        *parsing_img_ptr++ = bgr_putpalette[id * 3 + 1];
        *parsing_img_ptr++ = bgr_putpalette[id * 3 + 2];
      }
    }

    // resize parsing image
    cv::resize(parsing_img, parsing_img, mat.size(), 0, 0, cv::INTER_NEAREST);

    // alpha blending
    float alpha_f = 0.5;
    cv::Mat dst;

    addWeighted(mat, alpha_f, parsing_img, 1 - alpha_f, 0.0, dst);
    mat = std::move(dst);
  } else {
    auto &cls = perception->cls;
    for (int i = 0; i < cls.size(); i++) {
      auto &c = cls[i];
      auto &color = colors[c.id % 7];
      std::stringstream text_ss;
      text_ss << c.id << ":" << std::fixed << std::setprecision(5) << c.score;
      cv::putText(mat,
                  text_ss.str(),
                  cv::Point(5, 20 + 10 * i),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5,
                  color,
                  1,
                  cv::LINE_AA);
    }
  }
  return 0;
}

int image_tensor_to_mat(ImageTensor *image_tensor, cv::Mat &mat) {
  auto &tensor = image_tensor->tensor;
  auto data_type = tensor.properties.tensorType;
  int h_idx, w_idx, c_idx;
  get_tensor_hwc_index(&image_tensor->tensor, &h_idx, &w_idx, &c_idx);
  auto height = tensor.properties.validShape.dimensionSize[h_idx];
  auto width = tensor.properties.validShape.dimensionSize[w_idx];
  auto stride = tensor.properties.alignedShape.dimensionSize[w_idx];

  if (data_type == HB_DNN_IMG_TYPE_YUV444) {
    mat.create(
        image_tensor->ori_image_height, image_tensor->ori_image_width, CV_8UC3);
    cv::Mat resized(height, width, CV_8UC3);
    if (tensor.properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
      memcpy(resized.data, tensor.sysMem[0].virAddr, height * width * 3);
    } else if (tensor.properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
      int channel_size = height * width;
      uint8_t *mem = reinterpret_cast<uint8_t *>(tensor.sysMem[0].virAddr);
      nchw_to_nhwc(resized.data,
                   mem,
                   mem + channel_size,
                   mem + channel_size * 2,
                   height,
                   width);
    }
    cv::Mat yuv;
    cv::resize(resized, yuv, mat.size(), 0, 0);
    cv::cvtColor(yuv, mat, CV_YUV2BGR);
  } else if (data_type == HB_DNN_IMG_TYPE_Y) {
    mat.create(
        image_tensor->ori_image_height, image_tensor->ori_image_width, CV_8UC1);
    cv::Mat resized(height, width, CV_8UC1);

    uint8_t *data = resized.data;
    uint8_t *data0 = reinterpret_cast<uint8_t *>(tensor.sysMem[0].virAddr);

    for (int h = 0; h < height; h++) {
      for (int w = 0; w < width; w++) {
        *data++ = data0[h * stride + w];
      }
    }
    cv::resize(resized, mat, mat.size(), 0, 0);
  } else if (data_type == HB_DNN_IMG_TYPE_NV12) {
    mat.create(
        image_tensor->ori_image_height, image_tensor->ori_image_width, CV_8UC3);
    cv::Mat nv12(height * 3 / 2, width, CV_8UC1);
    cv::Mat resized;

    uint8_t *data = nv12.data;
    uint8_t *data0 = reinterpret_cast<uint8_t *>(tensor.sysMem[0].virAddr);
    uint8_t *data1 = data0 + height * stride;
    for (int h = 0; h < height; h++) {
      for (int w = 0; w < width; w++) {
        *data++ = data0[h * stride + w];
      }
    }
    for (int i = 0; i < height * width / 2; i++) {
      *data++ = *data1++;
    }
    cv::cvtColor(nv12, resized, cv::COLOR_YUV2BGR_NV12);
    cv::resize(resized, mat, mat.size(), 0, 0);
  } else if (data_type == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
    mat.create(
        image_tensor->ori_image_height, image_tensor->ori_image_width, CV_8UC3);
    cv::Mat nv12(height * 3 / 2, width, CV_8UC1);
    cv::Mat resized;

    uint8_t *data = nv12.data;
    uint8_t *data0 = reinterpret_cast<uint8_t *>(tensor.sysMem[0].virAddr);
    uint8_t *data1 = reinterpret_cast<uint8_t *>(tensor.sysMem[1].virAddr);
    for (int h = 0; h < height; h++) {
      for (int w = 0; w < width; w++) {
        *data++ = data0[h * stride + w];
      }
    }
    for (int i = 0; i < height * width / 2; i++) {
      *data++ = *data1++;
    }
    cv::cvtColor(nv12, resized, cv::COLOR_YUV2BGR_NV12);
    cv::resize(resized, mat, mat.size(), 0, 0);
  } else if ((data_type == HB_DNN_IMG_TYPE_BGR ||
              data_type == HB_DNN_IMG_TYPE_RGB) &&
             tensor.properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    mat.create(
        image_tensor->ori_image_height, image_tensor->ori_image_width, CV_8UC3);
    cv::Mat resized(height, width, CV_8UC3);
    int offset = height * width;
    nchw_to_nhwc(
        resized.ptr<uint8_t>(),
        reinterpret_cast<uint8_t *>(tensor.sysMem[0].virAddr),
        reinterpret_cast<uint8_t *>(tensor.sysMem[0].virAddr) + offset,
        reinterpret_cast<uint8_t *>(tensor.sysMem[0].virAddr) + offset * 2,
        height,
        width);

    if (data_type == HB_DNN_IMG_TYPE_RGB) {
      cv::Mat bgr;
      cv::cvtColor(resized, bgr, CV_RGB2BGR);
      cv::resize(bgr, mat, mat.size(), 0, 0);
    } else {
      cv::resize(resized, mat, mat.size(), 0, 0);
    }
  } else {
    VLOG(EXAMPLE_SYSTEM) << "Not implemented for " << data_type << " yet";
    return -1;
  }
  return 0;
}
