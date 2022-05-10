// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

// This is a simple program that describes how to run model which has custom op

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <queue>
#include <utility>

#include "custom_identity.h"
#include "dnn/hb_dnn.h"
#include "dnn/plugin/hb_dnn_plugin.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#define EMPTY ""

DEFINE_string(model_file, EMPTY, "model file path");
DEFINE_string(image_file, EMPTY, "Test image path");
DEFINE_int32(image_height, 224, "Image Height, 224 by default");
DEFINE_int32(image_width, 224, "Image Width, 224 by default");
DEFINE_int32(top_k, 5, "Top k classes, 5 by default");

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

typedef struct Classification {
  int id;
  float score;
  const char *class_name;

  Classification() : class_name(0) {}
  Classification(int id, float score, const char *class_name)
      : id(id), score(score), class_name(class_name) {}

  friend bool operator>(const Classification &lhs, const Classification &rhs) {
    return (lhs.score > rhs.score);
  }

  ~Classification() {}
} Classification;

/**
 * Read image and convert it to format NV12
 * @param[in] image_file: input image path
 * @param[in] yuv_height: yuv image height
 * @param[in] yuv_width: yuv image width
 * @param[out] img_nv12: nv12 image
 * @return: 0 if success, and -1 if failed
 */
int32_t read_image_2_nv12(std::string &image_file,
                          int yuv_height,
                          int yuv_width,
                          cv::Mat &img_nv12);

/**
 * Prepare tensor and fill with yuv data
 * @param[out] input_tensor: tensor to be prepared
 * @param[in] yuv_data: the yuv data
 * @param[in] dnn_handle: dnn handle
 * @param[in] image_height: image height
 * @param[in] image_width: image width
 * @return 0 if success otherwise -1
 */
int prepare_nv12_tensor(hbDNNTensor *input_tensor,
                        void *yuv_data,
                        hbDNNHandle_t dnn_handle,
                        int image_height,
                        int image_width);

/**
 * Prepare tensor according to node info
 * @param[out] tensor: tensor to be prepared
 * @param[out] dnn_handle: dnn handle
 * @param[in] node: model input or output node info
 * @return 0 if success otherwise -1
 */
int prepare_tensor(hbDNNTensor **tensor,
                   hbDNNHandle_t dnn_handle,
                   int output_count);

/**
 * Bgr image to nv12
 * @param[in] bgr_mat
 * @param[in] img_nv12
 */
int32_t bgr_to_nv12(cv::Mat &bgr_mat, cv::Mat &img_nv12);

void get_topk_result(hbDNNTensor *tensor,
                     std::vector<Classification> &top_k_cls,
                     int top_k);

int main(int argc, char **argv) {
  // Parsing command line arguments
  gflags::SetUsageMessage(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << gflags::GetArgv() << std::endl;

  // Init logging
  google::InitGoogleLogging("");
  google::SetStderrLogging(0);
  google::SetVLOGLevel("*", 3);
  FLAGS_colorlogtostderr = true;
  FLAGS_minloglevel = google::INFO;
  FLAGS_logtostderr = true;

  // register custom layer
  HB_CHECK_SUCCESS(
      hbDNNRegisterLayerCreator("CustomIdentity",
                                hobot::dnn::CustomIdentity_layer_creator),
      "hbDNNRegisterLayerCreator failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNRegisterLayerCreator success";

  // load model
  hbPackedDNNHandle_t packed_dnn_handle;
  auto modelFileName = FLAGS_model_file.c_str();
  HB_CHECK_SUCCESS(
      hbDNNInitializeFromFiles(&packed_dnn_handle, &modelFileName, 1),
      "hbDNNInitializeFromFiles failed");

  // get dnn handle
  const char **model_name_list;
  int model_count = 0;
  HB_CHECK_SUCCESS(
      hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
      "hbDNNGetModelNameList failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNGetModelNameList success";

  // get model handle
  hbDNNHandle_t dnn_handle;
  HB_CHECK_SUCCESS(
      hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]),
      "hbDNNGetModelHandle failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNGetModelHandle success";

  // Prepare input. For MobileNetV1_224x224
  // only one input, and the shape is 224x224
  cv::Mat yuv_mat;
  HB_CHECK_SUCCESS(
      read_image_2_nv12(
          FLAGS_image_file, FLAGS_image_height, FLAGS_image_width, yuv_mat),
      "read_image_2_nv12 failed");
  VLOG(EXAMPLE_DEBUG) << "read image to nv12 success";

  // prepare for input
  hbDNNTensor input;
  HB_CHECK_SUCCESS(prepare_nv12_tensor(&input,
                                       yuv_mat.ptr<uint8_t>(),
                                       dnn_handle,
                                       FLAGS_image_height,
                                       FLAGS_image_width),
                   "prepare_nv12_tensor failed");
  VLOG(EXAMPLE_DEBUG) << "prepare nv12 tensor success";

  // prepare for output
  int input_count, output_count;
  hbDNNGetInputCount(&input_count, dnn_handle);
  hbDNNGetOutputCount(&output_count, dnn_handle);
  hbDNNTensor *output = new hbDNNTensor[output_count];

  HB_CHECK_SUCCESS(prepare_tensor(&output, dnn_handle, output_count),
                   "prepare_tensor failed");
  VLOG(EXAMPLE_DEBUG) << "prepare tensor success";

  // Run inference
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

  HB_CHECK_SUCCESS(
      hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param),
      "hbDNNInfer failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNInfer success";

  // wait task done
  HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                   "hbDNNWaitTaskDone failed");
  VLOG(EXAMPLE_DEBUG) << "task done";

  // post process
  std::vector<Classification> top_k_cls;
  get_topk_result(output, top_k_cls, FLAGS_top_k);
  VLOG(EXAMPLE_DEBUG) << "task post process success";

  // release task handle
  HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
  task_handle = nullptr;
  for (int i = 0; i < FLAGS_top_k; i++) {
    VLOG(EXAMPLE_REPORT) << "TOP " << i << " result id: " << top_k_cls[i].id;
  }

  // free input mem
  HB_CHECK_SUCCESS(hbSysFreeMem(&(input.sysMem[0])), "hbSysFreeMem failed");

  // free output mem
  for (int i = 0; i < output_count; i++) {
    HB_CHECK_SUCCESS(hbSysFreeMem(&(output[i].sysMem[0])),
                     "hbSysFreeMem failed");
  }

  delete[] output;

  // release model
  HB_CHECK_SUCCESS(hbDNNRelease(packed_dnn_handle), "hbDNNRelease failed");
  return 0;
}

int prepare_nv12_tensor(hbDNNTensor *input_tensor,
                        void *yuv_data,
                        hbDNNHandle_t dnn_handle,
                        int image_height,
                        int image_width) {
  // get input[0] properties
  int input_count = 0;
  hbDNNTensorProperties input_properties;
  HB_CHECK_SUCCESS(
      hbDNNGetInputTensorProperties(&input_properties, dnn_handle, input_count),
      "hbDNNGetInputTensorProperties failed");
  auto &mem = input_tensor->sysMem[0];
  input_tensor->properties = input_properties;
  // the input data is already aligned
  int yuv_length = image_height * image_width * 3 / 2;
  HB_CHECK_SUCCESS(hbSysAllocCachedMem(&mem, yuv_length),
                   "hbSysAllocCachedMem failed");
  memcpy(mem.virAddr, yuv_data, yuv_length);
  HB_CHECK_SUCCESS(hbSysFlushMem(&mem, HB_SYS_MEM_CACHE_CLEAN),
                   "hbSysFlushMem failed");
  return 0;
}

int32_t read_image_2_nv12(std::string &image_file,
                          int32_t yuv_height,
                          int32_t yuv_width,
                          cv::Mat &img_nv12) {
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  if (bgr_mat.empty()) {
    VLOG(EXAMPLE_SYSTEM) << "image file not exist!";
    return -1;
  }
  // resize
  cv::Mat mat;
  mat.create(yuv_height, yuv_width, bgr_mat.type());
  cv::resize(bgr_mat, mat, mat.size(), 0, 0);

  auto ret = bgr_to_nv12(mat, img_nv12);
  return ret;
}

int prepare_tensor(hbDNNTensor **output_tensor,
                   hbDNNHandle_t dnn_handle,
                   int output_count) {
  hbDNNTensor *output = *output_tensor;
  for (int i = 0; i < output_count; i++) {
    hbDNNTensorProperties &output_properties = output[i].properties;
    HB_CHECK_SUCCESS(
        hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i),
        "hbDNNGetOutputTensorProperties failed");

    // get all aligned size
    int aligned_shape_num = output_properties.alignedShape.numDimensions;
    int out_aligned_size = output_properties.alignedByteSize;
    hbSysMem &mem = output[i].sysMem[0];
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&mem, out_aligned_size),
                     "hbSysAllocCachedMem failed");
  }
  return 0;
}

int32_t bgr_to_nv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    VLOG(EXAMPLE_SYSTEM) << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);

  uint8_t *yuv = yuv_mat.ptr<uint8_t>();
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  uint8_t *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  // copy y data
  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}

void get_topk_result(hbDNNTensor *tensor,
                     std::vector<Classification> &top_k_cls,
                     int top_k) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  std::priority_queue<Classification,
                      std::vector<Classification>,
                      std::greater<Classification>>
      queue;
  int *shape = tensor->properties.validShape.dimensionSize;
  auto data = reinterpret_cast<int32_t *>(tensor->sysMem[0].virAddr);
  auto shift = tensor->properties.shift.shiftData;
  auto scale = tensor->properties.scale.scaleData;
  for (auto i = 0; i < shape[1] * shape[2] * shape[3]; i++) {
    float score;
    if (tensor->properties.quantiType == SHIFT) {
      score = static_cast<float>(data[i]) / static_cast<float>(1 << shift[i]);
    } else if (tensor->properties.quantiType == SCALE) {
      score = static_cast<float>(data[i]) * scale[i];
    } else {
      score = static_cast<float>(data[i]);
    }
    queue.push(Classification(i, score, ""));
    if (queue.size() > top_k) {
      queue.pop();
    }
  }
  while (!queue.empty()) {
    top_k_cls.emplace_back(queue.top());
    queue.pop();
  }
  std::reverse(top_k_cls.begin(), top_k_cls.end());
}
