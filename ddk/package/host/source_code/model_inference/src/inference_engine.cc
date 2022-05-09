/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_engine
 * @file   inference_engine.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/inference_engine.h"
#include <cstring>
#include "model_inference/inference_engine_bpu.h"
#undef HB_SYS_MEM_CACHE_INVALIDATE
#undef HB_SYS_MEM_CACHE_CLEAN
#include "model_inference/inference_engine_dnn.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

std::mutex InferenceEngine::mutex_;
PredictType InferenceEngine::predict_type_ = DNN_PREDICT;  // 默认dnn方式
InferenceEngine *InferenceEngine::infer_engine_ = nullptr;
// 每个队列的推理线程数, 默认2
int InferenceEngine::inference_thread_per_core_ = 2;
// 后处理队列的线程数, 默认4
int InferenceEngine::result_work_thread_num_ = 4;

InferenceEngine *InferenceEngine::GetInstance() {
  if (infer_engine_ == nullptr) {
    std::lock_guard<std::mutex> guard(InferenceEngine::mutex_);
    if (infer_engine_ == nullptr) {
      if (predict_type_ == BPU_PREDICT) {
        infer_engine_ = new (std::nothrow) BPUInferenceEngine();
      } else if (predict_type_ == DNN_PREDICT) {
        infer_engine_ = new (std::nothrow) DNNInferenceEngine();
      } else {
        infer_engine_ = new (std::nothrow) InferenceEngine();
      }
    }
  }
  return (infer_engine_);
}

void InferenceEngine::SetInferenceWorkThreadNum(int thread_num) {
  HOBOT_CHECK(thread_num > 0) << "invalid thread_num: " << thread_num;
  inference_thread_per_core_ = thread_num;
}

void InferenceEngine::SetResultWorkThreadNum(int thread_num) {
  HOBOT_CHECK(thread_num > 0) << "invalid thread_num: " << thread_num;
  result_work_thread_num_ = thread_num;
}

int InferenceEngine::GetHWCIndex(
    DataType data_type, TensorLayout layout,
    int *h_idx, int *w_idx, int *c_idx) {
  switch (data_type) {
    case IMG_TYPE_BGR:
    case IMG_TYPE_RGB:
    case IMG_TYPE_YUV444:
      *h_idx = 1;
      *w_idx = 2;
      *c_idx = 3;
      break;
    case IMG_TYPE_NV12:
    case IMG_TYPE_Y:
    case IMG_TYPE_NV12_SEPARATE:
      *c_idx = 1;
      *h_idx = 2;
      *w_idx = 3;
      break;
    case TENSOR_TYPE_S4:
    case TENSOR_TYPE_U4:
    case TENSOR_TYPE_U8:
    case TENSOR_TYPE_S8:
    case TENSOR_TYPE_F16:
    case TENSOR_TYPE_S16:
    case TENSOR_TYPE_U16:
    case TENSOR_TYPE_F32:
    case TENSOR_TYPE_S32:
    case TENSOR_TYPE_U32:
    case TENSOR_TYPE_F64:
    case TENSOR_TYPE_S64:
    case TENSOR_TYPE_U64: {
      // Feature's height and width is determined by layout.
      switch (layout) {
        case LAYOUT_NONE:
        return -1;
      case LAYOUT_NHWC:
        *h_idx = 1;
        *w_idx = 2;
        *c_idx = 3;
        break;
      case LAYOUT_NCHW:
        *c_idx = 1;
        *h_idx = 2;
        *w_idx = 3;
        break;
      case LAYOUT_NHWC_4W8C:
        *c_idx = 1;
        *h_idx = 2;
        *w_idx = 3;
        break;
      default:
        return -1;
      }
      break;
    }
    default:
      return -1;
  }
  return 0;
}

int InferenceEngine::AddInferenceTask(
    std::shared_ptr<InferenceEngineTask> task) {
  // 解析task内的run_ctrls_参数，确认core_id后送入对应的inference_task_queue_
  // 【这里需要注意BPU和DNN的参数区别（对外统一，内部运行时做区分）】
  int core_id = task->run_ctrls_.bpu_core_id;
  HOBOT_CHECK(core_id == 0 || core_id == 1 || core_id == 2);
  {
    std::lock_guard<std::mutex> lck(inference_queue_mutexs_[core_id]);
    inference_task_queue_[core_id].push(task);
    inference_condition_[core_id].notify_one();
  }
  return 0;
}

int InferenceEngine::InferenceTaskLoop(int core_id) {
  LOGD << "InferenceTaskLoop: " << core_id;
  // 从对应核上的inference_task_queue_[core_id]上获取 sequence id最小的任务；
  // 刷新cache，确保input tensor的数据写入bpu内存空间
  // 调用同步接口进行预测；
  // 预测完成，将对应的task的result设置正确，将处理好的任务放入返回队列
  const int core_num = 2;
  if (core_id > core_num || core_id < 0) {
    return -1;
  }
  do {
    std::shared_ptr<InferenceEngineTask> task;
    {
      std::unique_lock<std::mutex> lck(inference_queue_mutexs_[core_id]);
      inference_condition_[core_id].wait(
          lck, [&]() { return inference_task_queue_[core_id].size() > 0 ||
                              stop_; });
      if (stop_) {
        return 0;
      }
      task = inference_task_queue_[core_id].top();
      inference_task_queue_[core_id].pop();
    }

    // 同步预测
    int ret = InferenceTaskCore(task);

    // 预测完成，将对应的task的result设置正确，将处理好的任务放入返回队列
    {
      task->inference_result_ = ret;  // 设置推理结果
      std::lock_guard<std::mutex>
          lck(result_queue_mutex_);
      result_task_queue_.push(task);
      result_condition_.notify_one();
    }
  } while (!stop_);
  return 0;
}

int InferenceEngine::ResultTaskLoop() {
  // 从result_task_queue_获取结果;
  // 根据预测结果，刷新output的cache;
  // 调用task中的callback_;
  do {
    std::shared_ptr<InferenceEngineTask> task;
    {
      std::unique_lock<std::mutex> lck(result_queue_mutex_);
      result_condition_.wait(
          lck, [&]() { return result_task_queue_.size() > 0 ||
                              stop_; });
      if (stop_) {
        return 0;
      }
      task = result_task_queue_.top();
      result_task_queue_.pop();
    }

    // 判断推理是否成功, ResultTaskCore刷新flush
    int ret = task->inference_result_ || ResultTaskCore(task);
    if (ret == 0) {
      int batch = 0;
      // 释放input_tensor, tensor输入需要释放input
      std::shared_ptr<TensorInferenceEngineTask> tensor_task =
          std::dynamic_pointer_cast<TensorInferenceEngineTask>(task);
      if (tensor_task != nullptr) {
        batch = 1;
        FreeTensor(tensor_task->input_tensors_);
      } else {
        std::shared_ptr<RoiInferenceEngineTask> roi_task =
            std::dynamic_pointer_cast<RoiInferenceEngineTask>(task);
        for (auto &infer_box : roi_task->roi_box_) {
          if (infer_box.resizable) {
            batch++;
          }
        }
      }

      int out_layers = task->output_tensors_.size();
      if (task->convert_to_float_) {
      // 定点转浮点
        task->float_tensors_.resize(out_layers);
        for (int i = 0; i < out_layers; i++) {
          Tensor &tensor = task->output_tensors_[i];
          FloatTensor &float_tensor = task->float_tensors_[i];
          OutputTensors2FloatTensors(
              tensor, float_tensor, batch);
        }
        // 释放output_tensors
        FreeTensor(task->output_tensors_);
      }
      // task->callback_(task);
    } else {
      // 推理失败，释放output_tensors
      std::shared_ptr<TensorInferenceEngineTask> tensor_task =
          std::dynamic_pointer_cast<TensorInferenceEngineTask>(task);
      if (tensor_task != nullptr) {
        FreeTensor(tensor_task->input_tensors_);
      }
      FreeTensor(task->output_tensors_);
    }
    LOGD << "task ret: " << ret;
    task->inference_promise_.set_value(ret);  // 设置推理结束标志
  } while (!stop_);
}

void InferenceEngine::OutputTensors2FloatTensors(
    const Tensor &tensor,
    FloatTensor &float_tensor, int batch) {
  switch (tensor.properties.tensor_type) {
    // 模型输出直接是float，直接复制
    case TENSOR_TYPE_F32: {
      float_tensor.layout = tensor.properties.tensor_layout;
      int elem_size = 4;  // float32
      int batch_valid_size = 1;
      int batch_aligned_size = 1;
      for (int i = 0; i < 4; i++) {
        float_tensor.dim[i] = tensor.properties.valid_shape.dimension_size[i];
        batch_valid_size *= float_tensor.dim[i];
        batch_aligned_size *=
            tensor.properties.aligned_shape.dimension_size[i];
      }
      float_tensor.value.resize(batch_valid_size * batch);

      for (int batch_idx = 0; batch_idx < batch; batch_idx++) {
        // void *dst = float_tensor.value.data();
        void *dst = &float_tensor.value[batch_idx * batch_valid_size];
        void *src = tensor.sys_mem[0].vir_addr +
                    batch_idx * batch_aligned_size * elem_size;

        uint32_t dst_n_stride = float_tensor.dim[1] * float_tensor.dim[2] *
                                float_tensor.dim[3] * elem_size;
        uint32_t dst_h_stride = float_tensor.dim[2] * float_tensor.dim[3] *
                                elem_size;
        uint32_t dst_w_stride = float_tensor.dim[3] * elem_size;
        uint32_t src_n_stride =
            tensor.properties.valid_shape.dimension_size[1] *
            tensor.properties.valid_shape.dimension_size[2] *
            tensor.properties.valid_shape.dimension_size[3] * elem_size;
        uint32_t src_h_stride =
            tensor.properties.valid_shape.dimension_size[2] *
            tensor.properties.valid_shape.dimension_size[3] * elem_size;
        uint32_t src_w_stride =
            tensor.properties.valid_shape.dimension_size[3] * elem_size;
        for (int nn = 0; nn < float_tensor.dim[0]; nn++) {
          void *cur_n_dst = reinterpret_cast<int8_t *>(dst) +
              nn * dst_n_stride;
          void *cur_n_src = reinterpret_cast<int8_t *>(src) +
              nn * src_n_stride;
          for (int hh = 0; hh < float_tensor.dim[1]; hh++) {
            void *cur_h_dst =
                reinterpret_cast<int8_t *>(cur_n_dst) + hh * dst_h_stride;
            void *cur_h_src =
                reinterpret_cast<int8_t *>(cur_n_src) + hh * src_h_stride;
            for (int ww = 0; ww < float_tensor.dim[2]; ww++) {
              void *cur_w_dst = reinterpret_cast<int8_t *>(cur_h_dst) +
                  ww * dst_w_stride;
              void *cur_w_src = reinterpret_cast<int8_t *>(cur_h_src) +
                  ww * src_w_stride;
              memcpy(cur_w_dst, cur_w_src,
                     float_tensor.dim[3] * elem_size);
            }
          }
        }
      }
      break;
    }
    case TENSOR_TYPE_S8:
    case TENSOR_TYPE_U8:
    case TENSOR_TYPE_S32:
    case TENSOR_TYPE_U32:
    case TENSOR_TYPE_S64: {
      int src_elem_size = 1;
      int dst_elem_size = 4;  // float
      if (tensor.properties.tensor_type == TENSOR_TYPE_S32 ||
          tensor.properties.tensor_type == TENSOR_TYPE_U32) {
        src_elem_size = 4;
      } else if (tensor.properties.tensor_type == TENSOR_TYPE_S64) {
        src_elem_size = 8;
      }
      // convert to float
      float_tensor.layout = tensor.properties.tensor_layout;
      int batch_valid_size = 1;
      int batch_aligned_size = 1;
      for (int i = 0; i < 4; i++) {
        float_tensor.dim[i] =
            tensor.properties.valid_shape.dimension_size[i];
        batch_valid_size *= float_tensor.dim[i];
        batch_aligned_size *= tensor.properties.aligned_shape.dimension_size[i];
      }
      float_tensor.value.resize(batch_valid_size * batch);

      for (int batch_idx = 0; batch_idx < batch; batch_idx++) {
        void *dst = &float_tensor.value[batch_idx * batch_valid_size];
        void *src = tensor.sys_mem[0].vir_addr +
                    batch_idx * batch_aligned_size * src_elem_size;
        uint32_t dst_n_stride = float_tensor.dim[1] * float_tensor.dim[2] *
                                float_tensor.dim[3] * dst_elem_size;
        uint32_t dst_h_stride = float_tensor.dim[2] * float_tensor.dim[3] *
                                dst_elem_size;
        uint32_t dst_w_stride = float_tensor.dim[3] * dst_elem_size;
        uint32_t src_n_stride =
            tensor.properties.aligned_shape.dimension_size[1] *
            tensor.properties.aligned_shape.dimension_size[2] *
            tensor.properties.aligned_shape.dimension_size[3] *
            src_elem_size;
        uint32_t src_h_stride =
            tensor.properties.aligned_shape.dimension_size[2] *
            tensor.properties.aligned_shape.dimension_size[3] *
            src_elem_size;
        uint32_t src_w_stride =
            tensor.properties.aligned_shape.dimension_size[3] * src_elem_size;
        for (int nn = 0; nn < float_tensor.dim[0]; nn++) {
          void *cur_n_dst = reinterpret_cast<int8_t *>(dst) +
              nn * dst_n_stride;
          void *cur_n_src = reinterpret_cast<int8_t *>(src) +
              nn * src_n_stride;
          for (int hh = 0; hh < float_tensor.dim[1]; hh++) {
            void *cur_h_dst =
                reinterpret_cast<int8_t *>(cur_n_dst) + hh * dst_h_stride;
            void *cur_h_src =
                reinterpret_cast<int8_t *>(cur_n_src) + hh * src_h_stride;
            for (int ww = 0; ww < float_tensor.dim[2]; ww++) {
              void *cur_w_dst = reinterpret_cast<int8_t *>(cur_h_dst) +
                  ww * dst_w_stride;
              void *cur_w_src = reinterpret_cast<int8_t *>(cur_h_src) +
                  ww * src_w_stride;
              for (int cc = 0; cc < float_tensor.dim[3]; cc++) {
                void *cur_c_dst = reinterpret_cast<int8_t *>(cur_w_dst) +
                    cc * dst_elem_size;
                void *cur_c_src = reinterpret_cast<int8_t *>(cur_w_src) +
                    cc * src_elem_size;
                if (src_elem_size == 4) {
                  int32_t tmp_int32_value =
                      *(reinterpret_cast<int32_t *>(cur_c_src));
                  // 转浮点
                  float tmp_float_value = tmp_int32_value;
                  // 确定对应的shift
                  uint8_t shift = 0;
                  if (float_tensor.layout == LAYOUT_NHWC) {
                    shift = tensor.properties.shift.shift_data[cc];
                  } else if (float_tensor.layout == LAYOUT_NCHW) {
                    shift = tensor.properties.shift.shift_data[hh];
                  }
                  if (tmp_int32_value != 0) {
                    int *ix = reinterpret_cast<int *>(&tmp_float_value);
                    (*ix) -= shift * 0x00800000;
                  }
                  *(reinterpret_cast<float *>(cur_c_dst)) =
                      tmp_float_value;
                } else if (src_elem_size == 1) {
                  int8_t tmp_int8_value =
                      *(reinterpret_cast<int8_t *>(cur_c_src));
                  // 转浮点
                  float tmp_float_value;
                  // 确定对应的shift
                  uint8_t shift = 0;
                  if (float_tensor.layout == LAYOUT_NHWC) {
                    shift = tensor.properties.shift.shift_data[cc];
                  } else if (float_tensor.layout == LAYOUT_NCHW) {
                    shift = tensor.properties.shift.shift_data[hh];
                  }
                  tmp_float_value = (static_cast<float>(tmp_int8_value)) /
                      (static_cast<float>(1 << shift));
                  *(reinterpret_cast<float *>(cur_c_dst)) =
                      tmp_float_value;
                } else {  // src_elem_size == 8
                  int64_t tmp_int64_value =
                      *(reinterpret_cast<int64_t *>(cur_c_src));
                  // 转浮点
                  float tmp_float_value;
                  // 确定对应的shift
                  uint8_t shift = 0;
                  if (float_tensor.layout == LAYOUT_NHWC) {
                    shift = tensor.properties.shift.shift_data[cc];
                  } else if (float_tensor.layout == LAYOUT_NCHW) {
                    shift = tensor.properties.shift.shift_data[hh];
                  }
                  tmp_float_value = (static_cast<float>(tmp_int64_value)) /
                      (static_cast<float>(1 << shift));
                  *(reinterpret_cast<float *>(cur_c_dst)) = tmp_float_value;
                }
              }
            }
          }
        }
      }

      break;
    }
    default:
      HOBOT_CHECK(0) << "not support tensor_type: "
          << tensor.properties.tensor_type;
  }
}

}  // namespace inference
