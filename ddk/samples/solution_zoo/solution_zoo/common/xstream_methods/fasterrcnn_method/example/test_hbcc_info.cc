//
// Created by yaoyao.sun on 2019-05-18.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#include <iostream>
#include "bpu_predict_extension.h"

int TestHBCCInfo(int argc, char **argv) {
  const char *model_file_path = "./models/faceMultitask.hbm";

  BPU_MODEL_S bpu_handle;
  int ret = HB_BPU_loadModelFromFile(model_file_path, &bpu_handle);
  if (ret != 0) {
    std::cout << "here load bpu model failed: "
              << HB_BPU_getErrorName(ret) << std::endl;
    return 1;
  }
  HB_BPU_releaseModel(&bpu_handle);
  return 0;
}

