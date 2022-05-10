// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <iostream>
#include <string>

#include "dnn/hb_dnn.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "time.h"
/**
 * Align by 16
 */

DEFINE_string(image_file, "../data/cat_cls.jpg", "Image to be resize");
DEFINE_int32(resize_height, 224, "Resize image height to");
DEFINE_int32(resize_width, 224, "Resize image width to");
DEFINE_string(resized_image, "224x224.jpg", "Resized image output path");
DEFINE_int32(log_level,
             3,
             "Logging level (SYSTEM=0, REPORT=1, DETAIL=2, DEBUG=3)");

#define ALIGNED_2E(w, alignment) ((w) + (alignment - 1)) & (~(alignment - 1))
#define ALIGN_16(w) ALIGNED_2E(w, 16)

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
 * @param[in] image_data: image y data
 * @param[in] image_height: image height
 * @param[in] image_width: image width
 * @param[out] tensor: tensor to be prepared and filled
 * @return: 0 if success, and -1 if failed
 */
int prepare_y_tensor(uint8_t *image_data,
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
int prepare_y_tensor(int image_height, int image_width, hbDNNTensor *tensor);

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
  cv::Mat gray = cv::imread(FLAGS_image_file, 0);

  hbDNNTensor input_tensor;
  HB_CHECK_SUCCESS(
      prepare_y_tensor(gray.data, gray.rows, gray.cols, &input_tensor),
      "prepare_y_tensor failed");

  // Prepare output tensor
  hbDNNTensor output_tensor;
  HB_CHECK_SUCCESS(
      prepare_y_tensor(FLAGS_resize_height, FLAGS_resize_width, &output_tensor),
      "prepare_y_tensor failed");
  VLOG(EXAMPLE_DEBUG) << "Original shape: " << gray.cols << "x" << gray.rows
                      << " ,dest shape:" << FLAGS_resize_width << "x"
                      << FLAGS_resize_height << " ,aligned shape:"
                      << output_tensor.properties.alignedShape.dimensionSize[3]
                      << "x"
                      << output_tensor.properties.alignedShape.dimensionSize[2];

  hbDNNResizeCtrlParam ctrl;
  HB_DNN_INITIALIZE_RESIZE_CTRL_PARAM(&ctrl);
  // Resize
  hbDNNTaskHandle_t task_handle;
  clock_t start, end;
  start = clock();
  HB_CHECK_SUCCESS(
      hbDNNResize(&task_handle, &output_tensor, &input_tensor, nullptr, &ctrl),
      "hbDNNResize failed");

  VLOG(EXAMPLE_DETAIL) << "resize success";
  HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                   "hbDNNWaitTaskDone failed");

  VLOG(EXAMPLE_DETAIL) << "wait resize success";

  HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");

  end = clock();
  VLOG(EXAMPLE_DEBUG) << "spent time: "
                      << (double)(end - start) / CLOCKS_PER_SEC;
  // Save resized image to file disk
  auto resized_shape = output_tensor.properties.alignedShape.dimensionSize;
  cv::Mat y_mat(resized_shape[2], resized_shape[3], CV_8UC1);
  memcpy(y_mat.data,
         output_tensor.sysMem[0].virAddr,
         resized_shape[2] * resized_shape[3]);
  cv::imwrite(FLAGS_resized_image, y_mat);

  // Release tensor
  HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensor.sysMem[0])),
                   "hbSysFreeMem failed");
  HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensor.sysMem[0])),
                   "hbSysFreeMem failed");
  return 0;
}

int prepare_y_tensor(uint8_t *image_data,
                     int image_height,
                     int image_width,
                     hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_Y;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 1;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  // Align by 16 bytes
  int stride = ALIGN_16(image_width);
  auto &aligned_shape = properties.alignedShape;
  aligned_shape.numDimensions = 4;
  aligned_shape.dimensionSize[0] = 1;
  aligned_shape.dimensionSize[1] = 1;
  aligned_shape.dimensionSize[2] = image_height;
  aligned_shape.dimensionSize[3] = stride;

  int image_length = aligned_shape.dimensionSize[1] *
                     aligned_shape.dimensionSize[2] *
                     aligned_shape.dimensionSize[3];
  HB_CHECK_SUCCESS(hbSysAllocCachedMem(&tensor->sysMem[0], image_length),
                   "hbSysAllocCachedMem failed");
  uint8_t *data0 = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
  for (int h = 0; h < image_height; ++h) {
    auto *raw = data0 + h * stride;
    for (int w = 0; w < image_width; ++w) {
      *raw++ = *image_data++;
    }
  }

  HB_CHECK_SUCCESS(hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN),
                   "hbSysFlushMem failed");
  return 0;
}

int prepare_y_tensor(int image_height, int image_width, hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_Y;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;

  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 1;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  auto &aligned_shape = properties.alignedShape;
  aligned_shape = valid_shape;
  int image_length = image_height * image_width;
  HB_CHECK_SUCCESS(hbSysAllocCachedMem(&tensor->sysMem[0], image_length),
                   "hbSysAllocCachedMem failed");
  return 0;
}
