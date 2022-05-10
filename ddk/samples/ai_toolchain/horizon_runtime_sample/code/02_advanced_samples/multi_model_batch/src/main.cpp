// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <iostream>
#include <queue>
#include <vector>

#include "dnn/hb_dnn.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#define EMPTY ""

DEFINE_string(model_file, EMPTY, "model file path");
DEFINE_string(input_file, EMPTY, "input image path");

enum VLOG_LEVEL {
  EXAMPLE_SYSTEM = 0,
  EXAMPLE_REPORT = 1,
  EXAMPLE_DETAIL = 2,
  EXAMPLE_DEBUG = 3
};

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

#define HB_CHECK_SUCCESS(value, errmsg)                              \
  do {                                                               \
    /*value can be call of function*/                                \
    auto ret_code = value;                                           \
    if (ret_code != 0) {                                             \
      VLOG(EXAMPLE_SYSTEM) << errmsg << ", error code:" << ret_code; \
      return ret_code;                                               \
    }                                                                \
  } while (0);

void split(std::string &str,
           char sep,
           std::vector<std::string> &tokens,
           int32_t limit = -1);

int32_t read_image_2_nv12(std::string &image_file,
                          int32_t yuv_height,
                          int32_t yuv_width,
                          cv::Mat &img_nv12);

/**
 * Bgr image to nv12
 * @param[in] bgr_mat
 * @param[in] img_nv12
 */
int32_t bgr_to_nv12(cv::Mat &bgr_mat, cv::Mat &img_nv12);

int prepare_nv12_tensor(hbDNNTensor *input_tensor,
                        void *yuv_data,
                        hbDNNHandle_t dnn_handle,
                        int image_height,
                        int image_width);

int prepare_tensor(hbDNNTensor **output_tensor,
                   hbDNNHandle_t dnn_handle,
                   int output_count);

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

  // parse model file paths
  std::vector<std::string> model_file_path;
  split(FLAGS_model_file, ',', model_file_path);

  std::vector<const char *> model_file_names;
  for (int32_t i = 0; i < model_file_path.size(); i++) {
    model_file_names.push_back(model_file_path[i].c_str());
  }

  // load model
  hbPackedDNNHandle_t packed_dnn_handle;
  HB_CHECK_SUCCESS(
      hbDNNInitializeFromFiles(
          &packed_dnn_handle, model_file_names.data(), model_file_names.size()),
      "hbDNNInitializeFromFiles failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNInitializeFromFiles success";

  // get name list
  const char **model_name_list;
  int model_count = 0;
  HB_CHECK_SUCCESS(
      hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
      "hbDNNGetModelNameList failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNGetModelNameList success";

  // get model handle
  hbDNNHandle_t dnn_handle_googlenet;
  HB_CHECK_SUCCESS(
      hbDNNGetModelHandle(
          &dnn_handle_googlenet, packed_dnn_handle, model_name_list[0]),
      "hbDNNGetModelHandle failed");

  hbDNNHandle_t dnn_handle_mobilenetv2;
  HB_CHECK_SUCCESS(
      hbDNNGetModelHandle(
          &dnn_handle_mobilenetv2, packed_dnn_handle, model_name_list[1]),
      "hbDNNGetModelHandle failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNGetModelHandle success";

  // read input file and convert img to nv12 format
  std::vector<std::string> input_file_path;
  split(FLAGS_input_file, ',', input_file_path);

  cv::Mat nv12_mat_googlenet;
  HB_CHECK_SUCCESS(
      read_image_2_nv12(input_file_path[0], 224, 224, nv12_mat_googlenet),
      "read_image_2_nv12 failed");
  cv::Mat nv12_mat_mobilenetv2;
  HB_CHECK_SUCCESS(
      read_image_2_nv12(input_file_path[1], 224, 224, nv12_mat_mobilenetv2),
      "read_image_2_nv12 failed");
  VLOG(EXAMPLE_DEBUG) << "read image to nv12 success";

  // prepare input tensor
  hbDNNTensor input_tensor_googlenet;
  HB_CHECK_SUCCESS(prepare_nv12_tensor(&input_tensor_googlenet,
                                       nv12_mat_googlenet.ptr<uint8_t>(),
                                       dnn_handle_googlenet,
                                       224,
                                       224),
                   "prepare input tensor failed");
  hbDNNTensor input_tensor_mobilenetv2;
  HB_CHECK_SUCCESS(prepare_nv12_tensor(&input_tensor_mobilenetv2,
                                       nv12_mat_mobilenetv2.ptr<uint8_t>(),
                                       dnn_handle_mobilenetv2,
                                       224,
                                       224),
                   "prepare input tensor failed");
  VLOG(EXAMPLE_DEBUG) << "prepare input tensor success";

  // prepare output tensor
  hbDNNTensor *output_tensor_googlenet = new hbDNNTensor();
  HB_CHECK_SUCCESS(
      prepare_tensor(&output_tensor_googlenet, dnn_handle_googlenet, 1),
      "prepare output tensor failed");

  hbDNNTensor *output_tensor_mobilenetv2 = new hbDNNTensor();
  HB_CHECK_SUCCESS(
      prepare_tensor(&output_tensor_mobilenetv2, dnn_handle_mobilenetv2, 1),
      "prepare output tensor failed");
  VLOG(EXAMPLE_DEBUG) << "prepare output tensor success";

  // Run inference
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

  // The running time of the small model is short,
  // and the task scheduling time is relatively long.
  // Therefore, when multiple small models are run together,
  // multiple small models can be run as one task
  // to reduce the proportion of framework scheduling time.

  // For example, if we have n small model tasks,
  // we only need to set hbDNNInferCtrlParam::more of the first n-1 model tasks
  // to 1. Set hbDNNInferCtrlParam::more of the last model task to 0, and use
  // the hbDNNInfer interface to submit the tasks in turn.

  // submit first model task
  infer_ctrl_param.more = 1;
  HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,
                              &output_tensor_googlenet,
                              &input_tensor_googlenet,
                              dnn_handle_googlenet,
                              &infer_ctrl_param),
                   "hbDNNInfer failed");
  // submit second model task
  infer_ctrl_param.more = 0;
  HB_CHECK_SUCCESS(hbDNNInfer(&task_handle,
                              &output_tensor_mobilenetv2,
                              &input_tensor_mobilenetv2,
                              dnn_handle_mobilenetv2,
                              &infer_ctrl_param),
                   "hbDNNInfer failed");
  VLOG(EXAMPLE_DEBUG) << "infer success";

  // wait task done
  HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                   "hbDNNWaitTaskDone failed");
  VLOG(EXAMPLE_DEBUG) << "task done";

  // post process
  std::vector<Classification> top_k_cls;
  get_topk_result(output_tensor_googlenet, top_k_cls, 1);
  VLOG(EXAMPLE_REPORT) << "googlenet class result id: " << top_k_cls[0].id;

  top_k_cls.clear();
  get_topk_result(output_tensor_mobilenetv2, top_k_cls, 1);
  VLOG(EXAMPLE_REPORT) << "mobilenetv2 class result id: " << top_k_cls[0].id;

  // release task handle
  HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
  VLOG(EXAMPLE_DEBUG) << "release task success";

  // free input mem
  HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensor_googlenet.sysMem[0])),
                   "hbSysFreeMem failed");
  HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensor_mobilenetv2.sysMem[0])),
                   "hbSysFreeMem failed");

  // free output mem
  HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensor_googlenet->sysMem[0])),
                   "hbSysFreeMem failed");
  HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensor_mobilenetv2->sysMem[0])),
                   "hbSysFreeMem failed");
  delete output_tensor_googlenet;
  delete output_tensor_mobilenetv2;

  // release model
  HB_CHECK_SUCCESS(hbDNNRelease(packed_dnn_handle), "hbDNNRelease failed");
  return 0;
}

void split(std::string &str,
           char sep,
           std::vector<std::string> &tokens,
           int32_t limit) {
  int32_t pos = -1;
  while (true) {
    int32_t next_pos = str.find(sep, pos + 1);
    if (next_pos == std::string::npos) {
      tokens.emplace_back(str.substr(pos + 1));
      break;
    }
    tokens.emplace_back(str.substr(pos + 1, next_pos - pos - 1));
    if (tokens.size() == limit - 1) {
      tokens.emplace_back(str.substr(next_pos + 1));
      break;
    }
    pos = next_pos;
  }
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
