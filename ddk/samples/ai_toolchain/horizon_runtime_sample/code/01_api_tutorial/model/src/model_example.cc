// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <iostream>
#include <iterator>
#include <vector>

#include "dnn/hb_dnn.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(model_file_list, "", "Model file path");
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

int print_model_info(hbPackedDNNHandle_t packed_dnn_handle);

/**
 * Split str by sep
 * @param[in] str: str to split
 * @param[in] sep:
 * @param[out] tokens:
 * @param[in] limit:
 */
void split(std::string &str,
           char sep,
           std::vector<std::string> &tokens,
           int limit = -1);

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

  std::vector<std::string> model_file;
  split(FLAGS_model_file_list, ',', model_file);

  std::vector<const char *> model_file_c;
  for (int i = 0; i < model_file.size(); i++) {
    model_file_c.push_back(model_file[i].c_str());
  }

  // load model
  hbPackedDNNHandle_t packed_dnn_handle;
  HB_CHECK_SUCCESS(
      hbDNNInitializeFromFiles(
          &packed_dnn_handle, model_file_c.data(), model_file_c.size()),
      "hbDNNInitializeFromFiles failed");

  // print model info
  print_model_info(packed_dnn_handle);

  // release model
  HB_CHECK_SUCCESS(hbDNNRelease(packed_dnn_handle), "hbDNNRelease failed");
  return 0;
}

int print_model_info(hbPackedDNNHandle_t packed_dnn_handle) {
  // 1. getModelNameList
  std::stringstream ss;
  const char **model_name_list;
  int model_count = 0;
  HB_CHECK_SUCCESS(
      hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle),
      "hbDNNGetModelNameList failed");
  ss << "model count:" << model_count;
  for (int i = 0; i < model_count; i++) {
    ss << ", model[" << i << "]: " << model_name_list[i];
  }
  VLOG(EXAMPLE_DEBUG) << ss.str();

  for (int j = 0; j < model_count; j++) {
    // 2. getModelHandle
    hbDNNHandle_t dnnHandle;
    HB_CHECK_SUCCESS(
        hbDNNGetModelHandle(&dnnHandle, packed_dnn_handle, model_name_list[j]),
        "hbDNNGetModelHandle failed");
    VLOG(EXAMPLE_DEBUG) << "hbDNNGetModelHandle [" << model_name_list[j]
                        << "] success!";

    std::stringstream ss;
    // 3.getInputCount
    int input_count = 0;
    HB_CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnnHandle),
                     "hbDNNGetInputCount failed");
    ss << "[" << model_name_list[j]
       << "] Model Info:  input num: " << input_count;

    // 4. getInputTensorProperties
    for (int i = 0; i < input_count; i++) {
      hbDNNTensorProperties properties;
      HB_CHECK_SUCCESS(hbDNNGetInputTensorProperties(&properties, dnnHandle, i),
                       "hbDNNGetInputTensorProperties failed");
      std::string valid_shape = "( ";
      for (int k = 0; k < properties.validShape.numDimensions; k++) {
        valid_shape += std::to_string(properties.validShape.dimensionSize[k]);
        if (k != properties.validShape.numDimensions - 1) {
          valid_shape += ", ";
        }
      }
      valid_shape += " )";
      ss << ", input[" << i << "] validShape: " << valid_shape;

      std::string aligne_shape = "( ";
      for (int k = 0; k < properties.alignedShape.numDimensions; k++) {
        aligne_shape +=
            std::to_string(properties.alignedShape.dimensionSize[k]);
        if (k != properties.alignedShape.numDimensions - 1) {
          aligne_shape += ", ";
        }
      }
      aligne_shape += " )";
      ss << ", alignedShape: " << aligne_shape;
      ss << ", tensorLayout: " << properties.tensorLayout;
      ss << ", tensorType: " << properties.tensorType;
    }

    // 5. getOutputCount
    int output_count = 0;
    HB_CHECK_SUCCESS(hbDNNGetOutputCount(&output_count, dnnHandle),
                     "hbDNNGetOutputCount failed");
    ss << ", output num: " << output_count;

    // 6. getOutputTensorProperties
    for (int i = 0; i < output_count; i++) {
      hbDNNTensorProperties properties;
      HB_CHECK_SUCCESS(
          hbDNNGetOutputTensorProperties(&properties, dnnHandle, i),
          "hbDNNGetOutputTensorProperties failed");

      std::string valid_shape = "( ";
      for (int k = 0; k < properties.validShape.numDimensions; k++) {
        valid_shape += std::to_string(properties.validShape.dimensionSize[k]);
        if (k != properties.validShape.numDimensions - 1) {
          valid_shape += ", ";
        }
      }
      valid_shape += " )";
      ss << ", output[" << i << "] validShape: " << valid_shape;

      std::string aligne_shape = "( ";
      for (int k = 0; k < properties.alignedShape.numDimensions; k++) {
        aligne_shape +=
            std::to_string(properties.alignedShape.dimensionSize[k]);
        if (k != properties.alignedShape.numDimensions - 1) {
          aligne_shape += ", ";
        }
      }
      aligne_shape += " )";
      ss << ", alignedShape: " << aligne_shape;
      ss << ", tensorLayout: " << properties.tensorLayout;
      ss << ", tensorType: " << properties.tensorType;
    }
    VLOG(EXAMPLE_DEBUG) << ss.str();
  }
  return 0;
}

void split(std::string &str,
           char sep,
           std::vector<std::string> &tokens,
           int limit) {
  int pos = -1;
  while (true) {
    int next_pos = str.find(sep, pos + 1);
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
