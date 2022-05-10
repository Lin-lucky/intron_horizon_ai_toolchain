// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

// This example demonstrates how to crop an roi from an image and then
// resize it to specified size. Should be noted that , only bgr is included
// in this example, it's similar for other image type.

#include <iostream>

#include "dnn/hb_dnn.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

DEFINE_string(image_file,
              "../data/ILSVRC2012_val_00000002.JPEG",
              "Image to be resize");
DEFINE_int32(crop_x1, 0, "Crop region coordinate left");  // random coordinates
DEFINE_int32(crop_y1, 0, "Crop region coordinate top");
DEFINE_int32(crop_x2, 0, "Crop region coordinate right");
DEFINE_int32(crop_y2, 0, "Crop region coordinate bottom");
DEFINE_int32(resize_height, 416, "Resize image height to");
DEFINE_int32(resize_width, 416, "Resize image width to");
DEFINE_string(resized_image, "resized_rgb.jpg", "Resized image output path");
DEFINE_int32(log_level,
             3,
             "Logging level (SYSTEM=0, REPORT=1, DETAIL=2, DEBUG=3)");

enum VLOG_LEVEL {
  EXAMPLE_SYSTEM = 0,
  EXAMPLE_REPORT = 1,
  EXAMPLE_DETAIL = 2,
  EXAMPLE_DEBUG = 3
};

#define HB_CHECK_SUCCESS(value, errmsg)                              \
  do {                                                               \
    /*value can be call of function*/                                \
    auto ret_code = value;                                           \
    if (ret_code != 0) {                                             \
      VLOG(EXAMPLE_SYSTEM) << errmsg << ", error code:" << ret_code; \
      return ret_code;                                               \
    }                                                                \
  } while (0);

/**
 * Prepare input tensor
 * @param[in] image_data: bgr data
 * @param[in] image_height: image height
 * @param[in] image_width: image width
 * @param[out] tensor: tensor to be prepared and filled
 * @return: 0 if success, and -1 if failed
 */
int32_t prepare_bgr_tensor(uint8_t *image_data,
                           int image_height,
                           int image_width,
                           hbDNNTensor *tensor);

/**
 * Prepare output tensor
 * @param[in] image_height: image height
 * @param[in] image_width: image width
 * @param[out] tensor: tensor to be prepared
 * @return: 0 if success, and -1 if failed
 */
int32_t prepare_bgr_tensor(int image_height,
                           int image_width,
                           hbDNNTensor *tensor);

/**
 * Release bgr tensor
 * @param[in] tensor: tensor to be released
 * @return: 0 if success, and -1 if failed
 */
int32_t free_bgr_tensor(hbDNNTensor *tensor);

int main(int argc, char **argv) {
  // Parsing command line arguments
  gflags::SetUsageMessage(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << gflags::GetArgv() << std::endl;

  // Init logging
  google::InitGoogleLogging("");
  google::SetStderrLogging(0);
  FLAGS_colorlogtostderr = true;
  google::SetVLOGLevel("*", FLAGS_log_level);

  FLAGS_max_log_size = 200;
  FLAGS_logbufsecs = 0;
  FLAGS_logtostderr = true;

  // Prepare input tensor
  cv::Mat ori_bgr = cv::imread(FLAGS_image_file, 1);

  hbDNNTensor input_tensor;
  HB_CHECK_SUCCESS(prepare_bgr_tensor(
                       ori_bgr.data, ori_bgr.rows, ori_bgr.cols, &input_tensor),
                   "prepare_bgr_tensor failed");

  // Prepare output tensor
  hbDNNTensor output_tensor;
  HB_CHECK_SUCCESS(prepare_bgr_tensor(
                       FLAGS_resize_height, FLAGS_resize_width, &output_tensor),
                   "prepare_bgr_tensor failed");

  // Resize
  VLOG(EXAMPLE_DEBUG) << "Original shape: " << ori_bgr.cols << "x"
                      << ori_bgr.rows << " ,dest shape:" << FLAGS_resize_width
                      << "x" << FLAGS_resize_height << " ,aligned shape:"
                      << output_tensor.properties.alignedShape.dimensionSize[2]
                      << "x"
                      << output_tensor.properties.alignedShape.dimensionSize[1];

  hbDNNResizeCtrlParam ctrl;
  HB_DNN_INITIALIZE_RESIZE_CTRL_PARAM(&ctrl);

  hbDNNTaskHandle_t task_handle;
  hbDNNRoi roi;
  if (FLAGS_crop_x2 == 0 && FLAGS_crop_y2 == 0) {
    HB_CHECK_SUCCESS(
        hbDNNResize(
            &task_handle, &output_tensor, &input_tensor, nullptr, &ctrl),
        "hbDNNResize failed");
  } else {
    roi = {FLAGS_crop_x1, FLAGS_crop_y1, FLAGS_crop_x2, FLAGS_crop_y2};
    HB_CHECK_SUCCESS(
        hbDNNResize(&task_handle, &output_tensor, &input_tensor, &roi, &ctrl),
        "hbDNNResize failed");
  }
  VLOG(EXAMPLE_DETAIL) << "resize success!";
  HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                   "hbDNNWaitTaskDone failed");

  VLOG(EXAMPLE_DETAIL) << "wait task done finished!";
  HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");

  // Save resized image to file disk
  auto resized_shape = output_tensor.properties.alignedShape.dimensionSize;
  cv::Mat crop_resized_bgr(resized_shape[1], resized_shape[2], CV_8UC3);
  memcpy(crop_resized_bgr.data,
         output_tensor.sysMem[0].virAddr,
         resized_shape[1] * resized_shape[2] * resized_shape[3]);
  cv::imwrite(FLAGS_resized_image, crop_resized_bgr);

  // Release tensor
  free_bgr_tensor(&input_tensor);
  free_bgr_tensor(&output_tensor);
}

int32_t prepare_bgr_tensor(uint8_t *image_data,
                           int image_height,
                           int image_width,
                           hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_BGR;
  properties.tensorLayout = HB_DNN_LAYOUT_NHWC;

  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = image_height;
  valid_shape.dimensionSize[2] = image_width;
  valid_shape.dimensionSize[3] = 3;

  auto &aligned_shape = properties.alignedShape;
  aligned_shape = valid_shape;

  int image_length = aligned_shape.dimensionSize[1] *
                     aligned_shape.dimensionSize[2] *
                     aligned_shape.dimensionSize[3];
  HB_CHECK_SUCCESS(hbSysAllocCachedMem(&tensor->sysMem[0], image_length),
                   "hbSysAllocCachedMem failed");

  void *data0 = tensor->sysMem[0].virAddr;
  memcpy(data0, image_data, image_length);
  HB_CHECK_SUCCESS(hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN),
                   "hbSysFlushMem failed");

  return 0;
}

int32_t prepare_bgr_tensor(int image_height,
                           int image_width,
                           hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_BGR;
  properties.tensorLayout = HB_DNN_LAYOUT_NHWC;

  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = image_height;
  valid_shape.dimensionSize[2] = image_width;
  valid_shape.dimensionSize[3] = 3;

  auto &aligned_shape = properties.alignedShape;
  aligned_shape = valid_shape;
  int image_length = aligned_shape.dimensionSize[1] *
                     aligned_shape.dimensionSize[2] *
                     aligned_shape.dimensionSize[3];
  HB_CHECK_SUCCESS(hbSysAllocCachedMem(&tensor->sysMem[0], image_length),
                   "hbSysAllocCachedMem failed");
  return 0;
}

int32_t free_bgr_tensor(hbDNNTensor *tensor) {
  HB_CHECK_SUCCESS(hbSysFreeMem(&(tensor->sysMem[0])), "hbSysFreeMem failed");
  return 0;
}
