/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include "bpu_predict_extension.h"
#include "vio/hb_vio_interface.h"
#include "./plat_cnn.h"

static void Usage() {
  std::cout << "./FasterRCNNMethod_example model_info model_name ipc/panel\n";
}

int TestModelInfo(int argc, char **argv) {
  if (argc < 3) {
    Usage();
    return -1;
  }
  std::cout << "core num: " << cnn_core_num() << std::endl;
  std::string model = argv[2];
  std::string model_file_path;
  if (model == "ipc") {
    model_file_path = "./models/IPCModel.hbm";
  } else if (model == "panel") {
    model_file_path = "./models/faceMultitask.hbm";
  } else {
    std::cout << "not support this model " << model << "\n";
  }
  BPU_MODEL_S bpu_handle;
  int ret = HB_BPU_loadModelFromFile(model_file_path.c_str(),
                          &bpu_handle);
  if (ret != 0) {
    std::cout << "here load bpu model failed: "
              << HB_BPU_getErrorName(ret) << std::endl;
    return 1;
  }
  std::cout << "here load bpu model OK" << std::endl;

  // get bpu version
  const char* version = HB_BPU_getVersion();
  if (version == nullptr) {
    std::cout << "here get bpu_predict version failed: " << std::endl;
    return 1;
  }
  std::cout << "here get bpu_predict version: " << version << std::endl;


  // print input info
  std::cout << "model has input: " << bpu_handle.input_num << std::endl;
  for (int i = 0; i < bpu_handle.input_num; ++i) {
    BPU_MODEL_NODE_S &input_node = bpu_handle.inputs[i];
    std::cout << "input " << i << ", name = " << input_node.name << std::endl;
    std::cout << "\t shape = ";
    for (int shape_i = 0; shape_i < input_node.shape.ndim; ++shape_i) {
      std::cout << input_node.shape.d[shape_i] << ",";
    }
    std::cout << std::endl;
  }

  // print output info
  std::cout << "model has output: " << bpu_handle.output_num << std::endl;
  for (int i = 0; i < bpu_handle.output_num; ++i) {
    BPU_MODEL_NODE_S &output_node = bpu_handle.outputs[i];
    std::cout << "output " << i << ", name = " << output_node.name << std::endl;
    std::cout << "\t shape = ";
    for (int shape_i = 0; shape_i < output_node.shape.ndim; ++shape_i) {
      std::cout << output_node.shape.d[shape_i] << ",";
    }
    std::cout << std::endl;
  }
  HB_BPU_releaseModel(&bpu_handle);
  return 0;
}

