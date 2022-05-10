// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

// This is a simple program that describes how to run MobileNetV1 classification
// on an image and get its top k results by predict score.
// Should be noted: Only mobileNetV1-nv12 is supported here.

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <utility>

#include "dnn/hb_dnn.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#define EMPTY ""

DEFINE_string(model_file, EMPTY, "model file path");
DEFINE_string(image_file, EMPTY, "Test image path");
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

std::map<int32_t, int32_t> element_size{{HB_DNN_IMG_TYPE_Y, 1},
                                        {HB_DNN_IMG_TYPE_NV12, 1},
                                        {HB_DNN_IMG_TYPE_NV12_SEPARATE, 1},
                                        {HB_DNN_IMG_TYPE_YUV444, 1},
                                        {HB_DNN_IMG_TYPE_RGB, 1},
                                        {HB_DNN_IMG_TYPE_BGR, 1},
                                        {HB_DNN_TENSOR_TYPE_S8, 1},
                                        {HB_DNN_TENSOR_TYPE_U8, 1},
                                        {HB_DNN_TENSOR_TYPE_F16, 2},
                                        {HB_DNN_TENSOR_TYPE_S16, 2},
                                        {HB_DNN_TENSOR_TYPE_U16, 2},
                                        {HB_DNN_TENSOR_TYPE_F32, 4},
                                        {HB_DNN_TENSOR_TYPE_S32, 4},
                                        {HB_DNN_TENSOR_TYPE_U32, 4},
                                        {HB_DNN_TENSOR_TYPE_F64, 8},
                                        {HB_DNN_TENSOR_TYPE_S64, 8},
                                        {HB_DNN_TENSOR_TYPE_U64, 8}};

int prepare_tensor(hbDNNTensor *input_tensor,
                   hbDNNTensor *output_tensor,
                   hbDNNHandle_t dnn_handle);

int32_t read_image_2_tensor_as_nv12(std::string &image_file,
                                    hbDNNTensor *input_tensor);

void get_topk_result(hbDNNTensor *tensor,
                     std::vector<Classification> &top_k_cls,
                     int top_k);

/**
 * Step1: get model handle
 * Step2: prepare input and output tensor
 * Step3: set input data to input tensor
 * Step4: run inference
 * Step5: do postprocess with output data
 * Step6: release resources
 */
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

  hbPackedDNNHandle_t packed_dnn_handle;
  hbDNNHandle_t dnn_handle;
  const char **model_name_list;
  auto modelFileName = FLAGS_model_file.c_str();
  int model_count = 0;
  // Step1: get model handle
  {
    HB_CHECK_SUCCESS(
        hbDNNInitializeFromFiles(&packed_dnn_handle, &modelFileName, 1),
        "hbDNNInitializeFromFiles failed");
    HB_CHECK_SUCCESS(hbDNNGetModelNameList(
                         &model_name_list, &model_count, packed_dnn_handle),
                     "hbDNNGetModelNameList failed");
    HB_CHECK_SUCCESS(
        hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]),
        "hbDNNGetModelHandle failed");
  }

  std::vector<hbDNNTensor> input_tensors;
  std::vector<hbDNNTensor> output_tensors;
  int input_count = 0;
  int output_count = 0;
  // Step2: prepare input and output tensor
  {
    HB_CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnn_handle),
                     "hbDNNGetInputCount failed");
    HB_CHECK_SUCCESS(hbDNNGetOutputCount(&output_count, dnn_handle),
                     "hbDNNGetInputCount failed");
    input_tensors.resize(input_count);
    output_tensors.resize(output_count);
    prepare_tensor(input_tensors.data(), output_tensors.data(), dnn_handle);
  }

  // Step3: set input data to input tensor
  {
    // read a single picture for input_tensor[0], for multi_input model, you
    // should set other input data according to model input properties.
    HB_CHECK_SUCCESS(
        read_image_2_tensor_as_nv12(FLAGS_image_file, input_tensors.data()),
        "read_image_2_tensor_as_nv12 failed");
    VLOG(EXAMPLE_DEBUG) << "read image to tensor as nv12 success";
  }

  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNTensor *output = output_tensors.data();
  // Step4: run inference
  {
    // make sure memory data is flushed to DDR before inference
    for (int i = 0; i < input_count; i++) {
      hbSysFlushMem(&input_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    }

    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,
                                &output,
                                input_tensors.data(),
                                dnn_handle,
                                &infer_ctrl_param),
                     "hbDNNInfer failed");
    // wait task done
    HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                     "hbDNNWaitTaskDone failed");
  }

  // Step5: do postprocess with output data
  std::vector<Classification> top_k_cls;
  {
    // make sure CPU read data from DDR before using output tensor data
    for (int i = 0; i < output_count; i++) {
      hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    }

    get_topk_result(output, top_k_cls, FLAGS_top_k);
    for (int i = 0; i < FLAGS_top_k; i++) {
      VLOG(EXAMPLE_REPORT) << "TOP " << i << " result id: " << top_k_cls[i].id;
    }
  }

  // Step6: release resources
  {
    // release task handle
    HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
    // free input mem
    for (int i = 0; i < input_count; i++) {
      HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensors[i].sysMem[0])),
                       "hbSysFreeMem failed");
    }
    // free output mem
    for (int i = 0; i < output_count; i++) {
      HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensors[i].sysMem[0])),
                       "hbSysFreeMem failed");
    }
    // release model
    HB_CHECK_SUCCESS(hbDNNRelease(packed_dnn_handle), "hbDNNRelease failed");
  }

  return 0;
}

int prepare_tensor(hbDNNTensor *input_tensor,
                   hbDNNTensor *output_tensor,
                   hbDNNHandle_t dnn_handle) {
  int input_count = 0;
  int output_count = 0;
  hbDNNGetInputCount(&input_count, dnn_handle);
  hbDNNGetOutputCount(&output_count, dnn_handle);

  /** Tips:
   * For input memory size:
   * *   input_memSize = input[i].properties.alignedByteSize
   * For output memory size:
   * *   output_memSize = output[i].properties.alignedByteSize
   */
  hbDNNTensor *input = input_tensor;
  for (int i = 0; i < input_count; i++) {
    HB_CHECK_SUCCESS(
        hbDNNGetInputTensorProperties(&input[i].properties, dnn_handle, i),
        "hbDNNGetInputTensorProperties failed");
    int input_memSize = input[i].properties.alignedByteSize;
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&input[i].sysMem[0], input_memSize),
                     "hbSysAllocCachedMem failed");
    /** Tips:
     * For input tensor, aligned shape should always be equal to the real
     * shape of the user's data. If you are going to set your input data with
     * padding, this step is not necessary.
     * */
    input[i].properties.alignedShape = input[i].properties.validShape;
  }

  hbDNNTensor *output = output_tensor;
  for (int i = 0; i < output_count; i++) {
    HB_CHECK_SUCCESS(
        hbDNNGetOutputTensorProperties(&output[i].properties, dnn_handle, i),
        "hbDNNGetOutputTensorProperties failed");
    int output_memSize = output[i].properties.alignedByteSize;
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&output[i].sysMem[0], output_memSize),
                     "hbSysAllocCachedMem failed");
  }
  return 0;
}

/** You can define read_image_2_tensor_as_other_type to prepare your data **/
int32_t read_image_2_tensor_as_nv12(std::string &image_file,
                                    hbDNNTensor *input_tensor) {
  hbDNNTensor *input = input_tensor;
  hbDNNTensorProperties Properties = input->properties;
  int tensor_id = 0;
  int input_h = Properties.validShape.dimensionSize[1];
  int input_w = Properties.validShape.dimensionSize[2];
  if (Properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    input_h = Properties.validShape.dimensionSize[2];
    input_w = Properties.validShape.dimensionSize[3];
  }

  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  if (bgr_mat.empty()) {
    VLOG(EXAMPLE_SYSTEM) << "image file not exist!";
    return -1;
  }
  // resize
  cv::Mat mat;
  mat.create(input_h, input_w, bgr_mat.type());
  cv::resize(bgr_mat, mat, mat.size(), 0, 0);
  // convert to YUV420
  if (input_h % 2 || input_w % 2) {
    VLOG(EXAMPLE_SYSTEM) << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  uint8_t *nv12_data = yuv_mat.ptr<uint8_t>();

  // copy y data
  auto data = input->sysMem[0].virAddr;
  int32_t y_size = input_h * input_w;
  memcpy(reinterpret_cast<uint8_t *>(data), nv12_data, y_size);

  // copy uv data
  int32_t uv_height = input_h / 2;
  int32_t uv_width = input_w / 2;
  uint8_t *nv12 = reinterpret_cast<uint8_t *>(data) + y_size;
  uint8_t *u_data = nv12_data + y_size;
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
  // The type reinterpret_cast should be determined according to the output type
  // For example: HB_DNN_TENSOR_TYPE_F32 is float
  auto data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  auto shift = tensor->properties.shift.shiftData;
  auto scale = tensor->properties.scale.scaleData;
  int tensor_len = shape[0] * shape[1] * shape[2] * shape[3];
  for (auto i = 0; i < tensor_len; i++) {
    float score = 0.0;
    if (tensor->properties.quantiType == SHIFT) {
      score = data[i] / (1 << shift[i]);
    } else if (tensor->properties.quantiType == SCALE) {
      score = data[i] * scale[i];
    } else {
      score = data[i];
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
