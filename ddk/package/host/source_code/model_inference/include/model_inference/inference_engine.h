/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_engine
 * @file   inference_engine.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef INFERENCE_ENGINE_H_
#define INFERENCE_ENGINE_H_

#include <queue>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include "inference_data.h"
#include "inference_task.h"
#include "inferencer.h"

namespace inference {
class Inferencer;
class InferenceEngineTaskCompare {
 public:
  bool operator()(
      std::shared_ptr<InferenceEngineTask> task_a,
      std::shared_ptr<InferenceEngineTask> task_b) {
    return (task_a->sequence_id_ >= task_a->sequence_id_);
  }
};

enum PredictType {
  BPU_PREDICT,
  DNN_PREDICT,
};

class InferenceEngine {
 public:
  static InferenceEngine *GetInstance();
  static void SetPredictType(PredictType pre_type) {
    predict_type_ = pre_type;
  }

  // 设置infer以及result_work线程数量
  // 注意使用：需要在GetInstance之前设置，构造函数根据数量构建线程
  static void SetInferenceWorkThreadNum(int thread_num);
  static void SetResultWorkThreadNum(int thread_num);

  InferenceEngine() {
    stop_ = false;
    // 绑定InferenceTaskLoop和ResultTaskLoop
    for (int thr_num = 0; thr_num < inference_thread_per_core_; thr_num++) {
      for (int core_id = 0; core_id < queue_size_; core_id++) {
        inference_work_threads_.push_back(
            std::thread(&InferenceEngine::InferenceTaskLoop, this, core_id));
      }
    }
    for (int thr_num = 0; thr_num < result_work_thread_num_; thr_num++) {
      result_work_threads_.push_back(
        std::thread(&InferenceEngine::ResultTaskLoop, this));
    }
  }

  virtual ~InferenceEngine() {
    stop_ = true;
    for (int i = 0; i < queue_size_; i++) {
      inference_condition_[i].notify_all();
    }
    result_condition_.notify_all();
    for (auto &thr : inference_work_threads_) {
      if (thr.joinable()) {
        thr.join();
      }
    }
    for (auto &thr : result_work_threads_) {
      if (thr.joinable()) {
        thr.join();
      }
    }
    // 清空任务队列
  }

  // 用于加载模型文件，不管是单模型还是packed的模型
  virtual void *LoadModelFileHandle(
      const char *model_file_name, bool is_packed_model) {
    return nullptr;
  }

  // 内部维护一个状态， 用于辅助后续的接口<model_file_handle, is_packed_model>
  std::map<void *, bool> is_packed_model_file_handle_;

  // 用于获取模型句柄，不管是单模型还是packed的模型
  virtual void *LoadModelHandle(
      void *model_file_handle, const char *model_name) {
    return nullptr;
  }
  // 调试接口, 打印所有输入输出信息
  virtual int PrintModelInfo(void *dnn_handle) { return -1; }
  virtual int ReleaseModelHandle(void *model_handle) { return -1; }
  virtual int ReleaseModelFileHandle(void *model_file_handle) { return -1; }

  // 获取模型信息
  virtual int GetModelInputInfo(
      void *model_handle, std::vector<TensorProperties> &input_modelinfo) {
    return -1;
  }
  virtual int GetModelOutputInfo(
      void *model_handle, std::vector<TensorProperties> &output_modelinfo) {
    return -1;
  }
  virtual int ResizeImage(const Tensor &nv12_tensor, Tensor &input_tensor,
                          const InferBBox &rois,
                          const ResizeCtrlParam resize_param) {
    return -1;
  }

  // 获取h、w、c索引
  int GetHWCIndex(DataType data_type, TensorLayout layout,
                  int *h_idx, int *w_idx, int *c_idx);

  // 仅赋值TensorProperties，不AllocTensor空间
  int PrepareModelTensorProperties(
      const std::vector<TensorProperties> &model_info,
      std::vector<Tensor> &tensors) {
    int layer_num = model_info.size();
    tensors.resize(layer_num);
    for (int layer_idx = 0; layer_idx < layer_num; layer_idx++) {
      TensorProperties info = model_info[layer_idx];
      Tensor &tensor = tensors[layer_idx];
      // properties
      tensor.properties = info;
    }
    return 0;
  }

  // 管理输入输出tensor, tensor properties需要已赋值
  // DNN、BPU接口申请后，包装为对外暴露的数据结构Tensor
  virtual int AllocModelTensor(
      std::vector<Tensor> &tensors,
      bool need_alloc = true, int batch = 1) {
    return -1;
  }

  virtual int FreeTensor(std::vector<Tensor> &tensors) {
    return -1;
  }

  // 根据pyramid和roi生成需要的tensor：区分DNN和BPU
  virtual int PrepareRoiTensor(
      const Inferencer *const infer,
      const std::vector<Tensor> &pyramid_tensors,  // 输入的多层金字塔数据
      std::vector<InferBBox> &rois, std::vector<Tensor> &tensors) {
    return -1;
  }

  int AddInferenceTask(std::shared_ptr<InferenceEngineTask> task);

 private:
  // 根据predict_type_实现具体的接口
  static PredictType predict_type_;
  static InferenceEngine *infer_engine_;
  static std::mutex mutex_;

 protected:
  std::map<void *, std::map<std::string, void *>> packed_models_;

  // 0: core 0 ; 1: core 1; 2: for any core
  static const int queue_size_ = 3;
  static int inference_thread_per_core_;  // 每个队列的推理线程数, 默认2
  static int result_work_thread_num_;     // 后处理队列的线程数, 默认4

  std::priority_queue<std::shared_ptr<InferenceEngineTask>,
                      std::vector<std::shared_ptr<InferenceEngineTask>>,
                      InferenceEngineTaskCompare>
      inference_task_queue_[queue_size_];
  std::mutex inference_queue_mutexs_[queue_size_];
  std::condition_variable inference_condition_[queue_size_];
  std::vector<std::thread> inference_work_threads_;

  std::priority_queue<std::shared_ptr<InferenceEngineTask>,
                      std::vector<std::shared_ptr<InferenceEngineTask>>,
                      InferenceEngineTaskCompare>
      result_task_queue_;
  std::mutex result_queue_mutex_;
  std::condition_variable result_condition_;
  std::vector<std::thread> result_work_threads_;

  std::atomic_bool stop_;

  int InferenceTaskLoop(int core_id);

  // 区分BPU、DNN
  virtual int InferenceTaskCore(std::shared_ptr<InferenceEngineTask> task) {
    return -1;
  }

  int ResultTaskLoop();

  // 区分BPU、DNN
  virtual int ResultTaskCore(std::shared_ptr<InferenceEngineTask> task) {
    return -1;
  }

  // output_tensors to float_tensors, batch for resizer model
  // copy or convert output_tensors to float_tensors
  void OutputTensors2FloatTensors(const Tensor &output_tensor,
                                  FloatTensor &float_tensor, int batch = 1);
};

}  // namespace inference

#endif  // INFERENCE_ENGINE_H_
