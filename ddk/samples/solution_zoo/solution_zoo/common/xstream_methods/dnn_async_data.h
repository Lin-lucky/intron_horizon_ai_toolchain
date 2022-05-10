/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: DnnAsyncData.h
 * @Brief: declaration of the DnnAsyncData
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-11-23 11:09:18
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-11-23 16:01:33
 */

#ifndef DNNASYNCDATA_DNNASYNCDATA_H_
#define DNNASYNCDATA_DNNASYNCDATA_H_

#include <vector>
#include <memory>
#include "bpu_predict_extension.h"
#include "xstream/xstream_data.h"

namespace xstream {

// to release BPU_MODEL_S
struct BPUModelWrapper {
  explicit BPUModelWrapper(BPU_MODEL_S bpu_model_) : bpu_model(bpu_model_) {}
  BPUModelWrapper() = default;

  ~BPUModelWrapper() {
    if (bpu_model.handle != nullptr) {
      HB_BPU_releaseModel(&bpu_model);
    }
  }
  BPU_MODEL_S bpu_model;
};

struct DnnAsyncData : public BaseData {
  // 模型handle,解析结果时需要通过它获取模型的信息，比如shape、shift等
  std::shared_ptr<BPUModelWrapper> dnn_model;
  // 输入tensor, 需要由DnnPostProcess进行资源释放
  // 最外层的vector,表示bpu任务的数量,与task_handle的维度一致,对于全图检测框类,维度应该为1
  std::vector<std::vector<BPU_TENSOR_S>> input_tensors;
  // 输出tensor,异步方式需要由DnnPostProcess进行资源释放
  std::vector<std::vector<BPU_TENSOR_S>> output_tensors;
  // 任务handle, 调用HB_BPU_waitModelDone接口需要
  std::vector<BPU_TASK_HANDLE> task_handle;
  // 是否调用bpu-predict同步接口
  bool dnn_is_sync;

  // 对于ROI类输入需要，需要在后处理中
  std::vector<int> valid_box;
  // pyramid + roi方式，后处理可能依赖ROI
  std::vector<BPU_BBOX> dnn_input_box;

  // 原始图像大小,后处理坐标映射需要
  int src_image_width;
  int src_image_height;

  void *reserved;  // 保留字段,用于扩展
};

}  // namespace xstream
#endif  // DNNASYNCDATA_DNNASYNCDATA_H_
