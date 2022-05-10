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
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <queue>

#include "dnn/hb_dnn.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#define EMPTY ""

DEFINE_string(model_file, EMPTY, "model file path");
DEFINE_string(data_file, EMPTY, "Test data path");
DEFINE_int32(image_height, 28, "Image Height, 28 by default");
DEFINE_int32(image_width, 28, "Image Width, 28 by default");
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

#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

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
 * Read model file
 * @param[in] file_path: file path
 * @param[out] bin: file binary content
 * @param[out] length: bin length
 * @return 0 if success otherwise -1
 */
int read_binary_file(std::string &file_path, char **bin, int *length);

/**
 * Prepare tensor and fill with yuv data
 * @param[out] input_tensor: tensor to be prepared
 * @param[in] dnn_handle: dnn handle
 * @param[in] image_height: image height
 * @param[in] image_width: image width
 * @return 0 if success otherwise -1
 */
int prepare_y_tensor(hbDNNTensor *input_tensor,
                     hbDNNHandle_t dnn_handle,
                     int image_height,
                     int image_width);

/**
 *
 * @param[in] tensor
 * @param[out] h_index
 * @param[out] w_index
 * @param[out] c_index
 * @return 0f if success
 */
int get_tensor_hwc_index(hbDNNTensor *tensor,
                         int *h_index,
                         int *w_index,
                         int *c_index);

/**
 * tensor padding for y
 * @param[in] tensor: tensor without data
 * @param[in] data: input data
 */
void tensor_padding_y(hbDNNTensor *tensor, char *data);

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

  // load model
  hbPackedDNNHandle_t packed_dnn_handle;
  auto modelFileName = FLAGS_model_file.c_str();
  HB_CHECK_SUCCESS(
      hbDNNInitializeFromFiles(&packed_dnn_handle, &modelFileName, 1),
      "hbDNNInitializeFromFiles failed");
  VLOG(EXAMPLE_DEBUG) << "hbDNNInitializeFromFiles success";

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

  // read bin file
  int32_t data_length = 0;
  char *data = nullptr;
  auto ret = read_binary_file(FLAGS_data_file, &data, &data_length);
  if (ret != 0) {
    return 0;
  }

  // prepare for input
  hbDNNTensor input;
  HB_CHECK_SUCCESS(
      prepare_y_tensor(
          &input, dnn_handle, FLAGS_image_height, FLAGS_image_width),
      "prepare y tensor failed");
  VLOG(EXAMPLE_DEBUG) << "prepare y tensor success";

  // data padding
  tensor_padding_y(&input, data);

  // prepare for output
  int input_count, output_count;
  hbDNNGetInputCount(&input_count, dnn_handle);
  hbDNNGetOutputCount(&output_count, dnn_handle);
  hbDNNTensor *output = new hbDNNTensor[output_count];

  HB_CHECK_SUCCESS(prepare_tensor(&output, dnn_handle, output_count),
                   "prepare tensor failed");
  VLOG(EXAMPLE_DEBUG) << "prepare tensor success";

  // Run inference
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

  HB_CHECK_SUCCESS(
      hbDNNInfer(&task_handle, &output, &input, dnn_handle, &infer_ctrl_param),
      "hbDNNInfer failed");
  VLOG(EXAMPLE_DEBUG) << "infer success";

  // wait task done
  HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                   "hbDNNWaitTaskDone failed");
  VLOG(EXAMPLE_DEBUG) << "task done";

  // post process
  std::vector<Classification> top_k_cls;
  get_topk_result(output, top_k_cls, FLAGS_top_k);
  VLOG(EXAMPLE_DEBUG) << "task post process finished";

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

int read_binary_file(std::string &file_path, char **bin, int *length) {
  std::ifstream ifs(file_path.c_str(), std::ios::in | std::ios::binary);
  if (!ifs) {
    VLOG(EXAMPLE_SYSTEM) << "Open " << file_path << " failed";
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  *length = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  *bin = new char[sizeof(char) * (*length)];
  ifs.read(*bin, *length);
  ifs.close();
  return 0;
}

int prepare_y_tensor(hbDNNTensor *input_tensor,
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
  int mem_size = input_properties.alignedByteSize;
  input_tensor->properties = input_properties;
  HB_CHECK_SUCCESS(hbSysAllocMem(&mem, mem_size), "hbSysAllocCachedMem failed");
  return 0;
}

void tensor_padding_y(hbDNNTensor *tensor, char *data) {
  auto &tensor_property = tensor->properties;
  auto &valid_shape = tensor_property.validShape;
  auto &aligned_shape = tensor_property.alignedShape;
  int32_t h_idx, w_idx, c_idx;
  auto ret = get_tensor_hwc_index(tensor, &h_idx, &w_idx, &c_idx);
  auto height = valid_shape.dimensionSize[h_idx];
  auto width = valid_shape.dimensionSize[w_idx];
  auto w_stride = aligned_shape.dimensionSize[w_idx];
  if (width == w_stride) {
    VLOG(EXAMPLE_DETAIL) << "do not need padding for y!";
    char *vir_addr = reinterpret_cast<char *>(tensor->sysMem[0].virAddr);
    memcpy(vir_addr, data, height * width);
  } else {
    // padding Y
    uint8_t *y_data = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
    uint8_t *image_data = reinterpret_cast<uint8_t *>(data);
    for (int32_t h = 0; h < height; ++h) {
      auto *raw = y_data + h * w_stride;
      for (int32_t w = 0; w < width; ++w) {
        *raw++ = *image_data++;
      }
    }
  }
}

int get_tensor_hwc_index(hbDNNTensor *tensor,
                         int *h_index,
                         int *w_index,
                         int *c_index) {
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    *h_index = 1;
    *w_index = 2;
    *c_index = 3;
  } else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    *c_index = 1;
    *h_index = 2;
    *w_index = 3;
  } else {
    return -1;
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
    HB_CHECK_SUCCESS(hbSysAllocMem(&mem, out_aligned_size),
                     "hbSysAllocMem failed");
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
