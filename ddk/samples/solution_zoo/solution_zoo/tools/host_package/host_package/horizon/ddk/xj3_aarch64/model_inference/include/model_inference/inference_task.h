/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_task
 * @file   inference_task.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef INFERENCE_TASK_H_
#define INFERENCE_TASK_H_

#include <vector>
#include <future>
#include <memory>
#include "inference_data.h"
#include "xstream/xstream_world.h"

namespace inference {

// Task只包括single model handle
class InferenceEngineTask : public xstream::BaseData {
 public:
  int64_t sequence_id_;                  // 外部传入(Execute函数中赋值)
  int inference_result_;
  void *user_context_;
  std::promise<int> inference_promise_;  // 推理结束标志
  void *model_handle_;  // BPU: BPU_MODEL_S; DNN: hawDNNHandle_t
  InferCtrlParam run_ctrls_;
  void *task_handle_;  // BPU: BPU_TASK_HANDLE; DNN: hawDNNTaskHandle_t

  bool convert_to_float_ = true;
  // 模型结果浮点数据：定点模型需要转换、浮点模型直接复制模型结果
  std::vector<FloatTensor> float_tensors_;
  std::vector<Tensor> output_tensors_;

  std::function<void(
      const std::shared_ptr<InferenceEngineTask> inference_task)> callback_;

  virtual ~InferenceEngineTask() {}
};

class TensorInferenceEngineTask : public InferenceEngineTask {
 public:
  std::vector<Tensor> input_tensors_;
  uint64_t timestamp_;
  uint64_t track_id_;
};

class RoiInferenceEngineTask : public InferenceEngineTask {
 public:
  // for DNN: input_tensors_[i][j]: input_layer i, roi j对应层的图像数据
  // for BPU: input_tensors_[i][k]: input_layer i,
  // pym_image k(BPU_ADDR_INFO_S包括非基础层)
  std::vector<std::vector<Tensor>> input_tensors_;
  std::vector<InferBBox> roi_box_;  // roi_box_[j]: roi j
};

}  // namespace inference

#endif  // INFERENCE_TASK_H_
