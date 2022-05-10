//
// Created by shiyu.fu on 2020-04-08.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <gtest/gtest.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "bpu_predict_extension.h"
#include "fasterrcnn_method/dump.h"
#include "fasterrcnn_method/fasterrcnn_method.h"
#include "fasterrcnn_method/yuv_utils.h"  // NOLINT
#include "hobotlog/hobotlog.hpp"
#include "opencv2/opencv.hpp"
#include "xstream/vision_type.h"
#include "xstream/xstream_sdk.h"

using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;
using xstream::InputParamPtr;

using xstream::FasterRCNNMethod;
using xstream::ImageFrame;
using xstream::RawDataImageFrame;

void print_model_info(BPU_MODEL_S* bpu_model) {
  // TODO(yingxiang.hong):  readable info for enum value
  auto shape_str_fn = [](BPU_DATA_SHAPE_S* shape) {
    std::stringstream ss;
    ss << "(";
    std::copy(shape->d, shape->d + shape->ndim,
              std::ostream_iterator<int>(ss, ","));
    ss << ")";
    ss << ", layout:" << shape->layout;
    return ss.str();
  };

  std::stringstream ss;
  ss << "Input num:" << bpu_model->input_num;
  for (int i = 0; i < bpu_model->input_num; i++) {
    auto& input_node = bpu_model->inputs[i];
    ss << ", input[" << i << "]: "
       << "name:" << input_node.name << ", data type:" << input_node.data_type
       << ", shape:" << shape_str_fn(&input_node.shape)
       << ", aligned shape:" << shape_str_fn(&input_node.aligned_shape);
  }

  ss << ", Output num:" << bpu_model->output_num;
  for (int i = 0; i < bpu_model->output_num; i++) {
    auto& output_node = bpu_model->outputs[i];
    ss << ", output[" << i << "]: "
       << "name:" << output_node.name << ", op:" << output_node.op_type
       << ", data type:" << output_node.data_type
       << ", shape:" << shape_str_fn(&output_node.shape)
       << ", aligned shape:" << shape_str_fn(&output_node.aligned_shape);
  }

  LOGI << "Model info:" << ss.str();
}

TEST(FasterRCNNTest, ModelInfo) {
  std::string model_file_path = "./models/multitask.hbm";
  // load model
  int ret = 0;
  BPU_MODEL_S* bpu_model;
  SetLogLevel(HOBOT_LOG_INFO);
  {
    std::ifstream ifs(model_file_path.c_str(), std::ios::in | std::ios::binary);
    if (!ifs) {
      HOBOT_CHECK(0) << "Open model file: " << model_file_path << " failed";
    }
    ifs.seekg(0, std::ios::end);
    int model_length = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    char* model_bin = new char[sizeof(char) * model_length];
    ifs.read(model_bin, model_length);
    ifs.close();
    bpu_model = new BPU_MODEL_S();
    ret = HB_BPU_loadModel(model_bin, model_length, bpu_model);
    HOBOT_CHECK(ret == 0) << "load model failed" << HB_BPU_getErrorName(ret);
    delete[] model_bin;
  }

  // get bpu version
  const char* version = HB_BPU_getVersion();
  ASSERT_TRUE(version != nullptr);
  LOGI << "Get BPU version: " << version;
  print_model_info(bpu_model);
  HB_BPU_releaseModel(bpu_model);
}

TEST(FasterRCNNTest, BasicAPIs) {
  FasterRCNNMethod faster_rcnn_method;
  std::string config_file = "./configs/multitask_config.json";
  auto ret = faster_rcnn_method.Init(config_file);
  EXPECT_EQ(ret, 0);
  auto faster_rcnn_param =
    std::make_shared<xstream::FasterRCNNParam>("FasterRCNNMethod");
  faster_rcnn_param->max_face_count = 1;
  ret = faster_rcnn_method.UpdateParameter(faster_rcnn_param);
  EXPECT_EQ(ret, 0);
  auto param = faster_rcnn_method.GetParameter();
  EXPECT_EQ(param->unique_name_, "FasterRCNNMethod");
  auto version = faster_rcnn_method.GetVersion();
  EXPECT_FALSE(version.empty());
  auto method_info = faster_rcnn_method.GetMethodInfo();
  EXPECT_FALSE(method_info.is_thread_safe_);
  EXPECT_TRUE(method_info.is_need_reorder_);
}
