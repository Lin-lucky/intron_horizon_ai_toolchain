/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: DnnPostProcessMethod.h
 * @Brief: declaration of the DnnPostProcessMethod
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-11-24 13:35:19
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-11-24 14:04:23
 */

#ifndef INCLUDE_DNNPOSTPROCESSMETHOD_DNNPOSTPROCESSMETHOD_H_
#define INCLUDE_DNNPOSTPROCESSMETHOD_DNNPOSTPROCESSMETHOD_H_

#include <mutex>
#include <fstream>
#include <string>
#include <vector>
#include "json/json.h"
#include "config.h"
#include "xstream/simple_method.h"
#include "dnn_async_data.h"
#include "bpu_predict_extension.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/profiler.h"

namespace xstream {

class DnnPostProcessMethod : public SimpleMethod {
 public:
  DnnPostProcessMethod() {}
  virtual ~DnnPostProcessMethod() {}

  int Init(const std::string &cfg_path) override {
    LOGI << "DnnPostProcessMethod Init";
    // 0. parse config_
    Json::Value config;
    std::ifstream infile(cfg_path.c_str());
    if (!infile.good()) {
      LOGW << "DnnPostProcessMethod error config file path: " << cfg_path;
    } else {
      infile >> config;
    }
    config_ = Config(config);
    return 0;
  }
  void Finalize() override {}

  // 主逻辑，完全复用，派生类不需要再实现DoProcess
  std::vector<BaseDataPtr> DoProcess(
      const std::vector<BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override {
    LOGD << "DnnPostProcessMethod DoProcess";
    HOBOT_CHECK(input.size() == 1) << "only support DnnAsyncData";
    RUN_PROCESS_TIME_PROFILER("DnnPostProcessMethod_DoProcess");
    std::vector<BaseDataPtr> output;
    {
      // DnnPostProcessMethod的输入只有一个，输入数据就是DnnAsyncData
      auto dnn_async_data = std::static_pointer_cast<DnnAsyncData>(input[0]);

      // 调用HB_BPU_waitModelDone接口，等待bpu异步任务完成
      if (!dnn_async_data->dnn_is_sync) {
        for (size_t i = 0; i < dnn_async_data->task_handle.size(); ++i) {
          BPU_TASK_HANDLE &task_handle = dnn_async_data->task_handle[i];
          if (task_handle == nullptr) continue;
          if (!HB_BPU_waitModelDone(&task_handle)) {
            HB_BPU_releaseTask(&task_handle);
          }
        }
      }

      // 调用派生类的接口，完成模型后处理，将结果转换成Method的输出格式
      {
        RUN_PROCESS_TIME_PROFILER("ParseDnnResult");
        int ret = ParseDnnResult(*dnn_async_data, output);
        if (ret != 0) {
          LOGE << "ParseDnnResult failed";
        }
      }
      // 释放输入与输出Tensor
      for (size_t i = 0; i < dnn_async_data->input_tensors.size(); ++i) {
        if (!dnn_async_data->input_tensors[i].empty()) {
          FreeTensor(dnn_async_data->input_tensors[i]);
        }
      }
      for (size_t i = 0; i < dnn_async_data->output_tensors.size(); ++i) {
        if (!dnn_async_data->output_tensors[i].empty()) {
          FreeTensor(dnn_async_data->output_tensors[i]);
        }
      }
    }
    return output;
  }

 public:
  Config config_;

  // 释放InputTensor/OutputTensor
  void FreeTensor(std::vector<BPU_TENSOR_S> &tensors) {
    for (size_t i = 0; i < tensors.size(); i++) {
      BPU_TENSOR_S &tensor = tensors[i];
      switch (tensor.data_type) {
        case BPU_TYPE_IMG_Y:
        case BPU_TYPE_IMG_YUV_NV12:
        case BPU_TYPE_IMG_YUV444:
        case BPU_TYPE_IMG_RGB:
        case BPU_TYPE_IMG_BGR:
        case BPU_TYPE_IMG_BGRP:
        case BPU_TYPE_IMG_RGBP:
        case BPU_TYPE_TENSOR_U8:
        case BPU_TYPE_TENSOR_S8:
        case BPU_TYPE_TENSOR_F32:
        case BPU_TYPE_TENSOR_S32:
        case BPU_TYPE_TENSOR_U32:
          HB_SYS_bpuMemFree(&tensor.data);
          break;
        case BPU_TYPE_IMG_NV12_SEPARATE:
          HB_SYS_bpuMemFree(&tensor.data);
          HB_SYS_bpuMemFree(&tensor.data_ext);
          break;
        default:
          HOBOT_CHECK(0) << "not support data_type: " << tensor.data_type;
          break;
      }
    }
  }

  // 派生类需要实现
  // 完成模型的后处理，以及转换成Method输出格式;不需考虑tensor的释放
  // IN: dnn_result. OUT: frame_result
  virtual int ParseDnnResult(DnnAsyncData &dnn_result,
                             std::vector<BaseDataPtr> &frame_result) {
    return -1;
  }
};
}  // namespace xstream
#endif  // INCLUDE_DNNPOSTPROCESSMETHOD_DNNPOSTPROCESSMETHOD_H_
