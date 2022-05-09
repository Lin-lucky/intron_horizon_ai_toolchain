/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_engine_bpu
 * @file   inference_engine_bpu.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef INFERENCE_ENGINE_BPU_H_
#define INFERENCE_ENGINE_BPU_H_

#include <memory>
#include <vector>
#include <map>
#include "inference_engine.h"
#include "bpu_predict_extension.h"

namespace inference {

class BPUInferenceEngine : public InferenceEngine {
  friend class InferenceEngine;
 public:
  // 准备InferenceEngineTask辅助接口
  virtual void *LoadModelFileHandle(
      const char *model_file_name, bool is_packed_model);

  virtual void *LoadModelHandle(
      void *model_file_handle, const char *model_name);

  virtual int ReleaseModelHandle(void *model_handle);
  virtual int ReleaseModelFileHandle(void *model_file_handle);

  virtual int GetModelInputInfo(
      void *model_handle,
      std::vector<TensorProperties> &input_modelinfo);

  virtual int GetModelOutputInfo(
      void *model_handle,
      std::vector<TensorProperties> &output_modelinfo);

  virtual int AllocModelTensor(
      std::vector<Tensor> &tensors,
      bool need_alloc = true, int batch = 1);

  virtual int FreeTensor(std::vector<Tensor> &tensors);
  virtual int ResizeImage(const Tensor &nv12_tensor, Tensor &input_tensor,
                          const InferBBox &rois,
                          const ResizeCtrlParam resize_param);
  virtual int PrepareRoiTensor(
      const Inferencer *const infer,
      const std::vector<Tensor> &pyramid_tensors,  // 输入的多层金字塔数据
      std::vector<InferBBox> &rois, std::vector<Tensor> &tensors);

  virtual int InferenceTaskCore(std::shared_ptr<InferenceEngineTask> task);

  virtual int ResultTaskCore(std::shared_ptr<InferenceEngineTask> task);

  BPUInferenceEngine() {}
  virtual ~BPUInferenceEngine() {}

 private:
  // 需要映射的变量
  static std::map<BPU_LAYOUT_E, TensorLayout> layout_table_;
  static std::map<BPU_DATA_TYPE_E, DataType> datatype_table_;
  static std::map<TensorLayout, BPU_LAYOUT_E> layout2bpu_table_;
  static std::map<DataType, BPU_DATA_TYPE_E> datatype2bpu_table_;

  std::mutex mutex_;

  void ConvertModelInfo(const BPU_MODEL_NODE_S &node_info,
                        TensorProperties &model_info);

  int ConvertTensor2BPUTensor(const Tensor &tensor,
                              BPU_TENSOR_S &bpu_tensor);
};

}  // namespace inference

#endif  // INFERENCE_ENGINE_BPU_H_
