/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_engine_dnn
 * @file   inference_engine_dnn.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef INFERENCE_ENGINE_DNN_H_
#define INFERENCE_ENGINE_DNN_H_

#include <memory>
#include <vector>

#include "dnn/hb_dnn.h"
#include "inference_engine.h"

namespace inference {

class DNNInferenceEngine : public InferenceEngine {
  friend class InferenceEngine;

 public:
  DNNInferenceEngine() {}
  ~DNNInferenceEngine() {}
  // 加载模型文件并返回其句柄
  virtual void *LoadModelFileHandle(const char *model_file_name,
                                    bool is_packed_model);
  // 返回指定模型句柄
  virtual void *LoadModelHandle(void *model_file_handle,
                                const char *model_name);
  // 打印模型输入输出详细信息
  virtual int PrintModelInfo(void *dnn_handle);
  // 释放模型句柄
  virtual int ReleaseModelHandle(void *model_handle);
  // 释放模型文件句柄
  virtual int ReleaseModelFileHandle(void *model_file_handle);
  // 获取模型输入信息
  virtual int GetModelInputInfo(void *model_handle,
                                std::vector<TensorProperties> &input_modelinfo);
  // 获取模型输出信息
  virtual int GetModelOutputInfo(
      void *model_handle, std::vector<TensorProperties> &output_modelinfo);
  // 为Tensor分配内存
  virtual int AllocModelTensor(std::vector<Tensor> &tensors,
                               bool need_alloc = true, int batch = 1);
  // 释放Tensor内存
  virtual int FreeTensor(std::vector<Tensor> &tensors);
  // 执行预测任务
  virtual int InferenceTaskCore(std::shared_ptr<InferenceEngineTask> task);
  // 获取预测结果
  virtual int ResultTaskCore(std::shared_ptr<InferenceEngineTask> task);
  // 初始化Roi类型的Tensor
  virtual int PrepareRoiTensor(
      const Inferencer *const infer,
      const std::vector<Tensor> &pyramid_tensors,  // 输入的多层金字塔数据
      std::vector<InferBBox> &rois, std::vector<Tensor> &tensors);
  virtual int ResizeImage(const Tensor &nv12_tensor, Tensor &input_tensor,
                          const InferBBox &rois,
                          const ResizeCtrlParam resize_param);

 private:
  std::mutex mutex_;
  // 把dnn类型的model信息结构转换成TensorProperties
  inline void ConvertModelInfo(const hbDNNTensorProperties &node_info,
                        TensorProperties &model_info);
  // 把Tensor类型转换成Dnn的tensor类型
  inline int ConvertTensor2DNNTensor(const Tensor &tensor,
                                     hbDNNTensor &dnn_tensor);
  int find_pyramid_layer(const int &dst_h, const int &dst_w,
                         const InferBBox &box, int &pym_layer, int &left,
                         int &top, int &right, int &bottom);
  int check_roi_valid(int left, int top, int right, int bottom, int dst_h,
                      int dst_w);
  float get_max_scale(const int &dst_h, const int &dst_w, const int &height,
                      const int &width);
};

}  // namespace inference

#endif  // INFERENCE_ENGINE_DNN_H_
