/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_engine
 * @file   inference_engine_dnn.h
 * @author xudong.du
 * @email  xudong.du@horizon.ai
 * @version   0.0.0.1
 * @date      2021.07.19
 */

#include "model_inference/inference_engine_dnn.h"

#include "hobotlog/hobotlog.hpp"

namespace inference {

#define BPU_DOWN_SCALE_LEVEL_MAX (5)
#define BPU_DOWN_SCALE(level) (1.0f / (1 << level))

int DNNInferenceEngine::PrintModelInfo(void *dnn_handle) {
  hbPackedDNNHandle_t packed_dnn_handle =
      static_cast<hbPackedDNNHandle_t>(dnn_handle);
  if (nullptr == packed_dnn_handle) {
    return -1;
  }
  // 1. getModelNameList
  std::stringstream ss;
  const char **model_name_list;
  int model_count = 0;
  if (0 != hbDNNGetModelNameList(&model_name_list, &model_count,
                                 packed_dnn_handle)) {
    LOGE << "hawDNN_getModelNameList failed";
    return -1;
  }
  ss << "model count:" << model_count;
  for (int i = 0; i < model_count; i++) {
    ss << ", model[" << i << "]: " << model_name_list[i];
  }
  LOGD << ss.str();

  for (int j = 0; j < model_count; j++) {
    // 2. getModelHandle
    hbDNNHandle_t dnnHandle;
    if (0 != hbDNNGetModelHandle(&dnnHandle, packed_dnn_handle,
                                 model_name_list[j])) {
      LOGE << "hbDNNGetModelHandle failed";
      return -1;
    }
    LOGD << "hbDNNGetModelHandle [" << model_name_list[j] << "] success!";
    std::cout << "handle: " << dnnHandle << ", " << packed_dnn_handle
              << std::endl;
    std::stringstream ss;
    // 3.getInputCount
    int input_count = 0;
    if (0 != hbDNNGetInputCount(&input_count, dnnHandle)) {
      LOGE << "hbDNNGetInputCount failed";
      return -1;
    }
    ss << "[" << model_name_list[j]
       << "] Model Info:  input num: " << input_count;

    // 4. getInputTensorProperties
    for (int i = 0; i < input_count; i++) {
      hbDNNTensorProperties properties;
      if (0 != hbDNNGetInputTensorProperties(&properties, dnnHandle, i)) {
        LOGE << "hawDNN_getInputTensorProperties failed";
        return -1;
      }
      std::string valid_shape = "( ";
      for (int k = 0; k < properties.validShape.numDimensions; k++) {
        valid_shape += std::to_string(properties.validShape.dimensionSize[k]);
        if (k != properties.validShape.numDimensions - 1) {
          valid_shape += ", ";
        }
      }
      valid_shape += " )";
      ss << ", input[" << i << "] validShape: " << valid_shape;

      std::string aligne_shape = "( ";
      for (int k = 0; k < properties.alignedShape.numDimensions; k++) {
        aligne_shape +=
            std::to_string(properties.alignedShape.dimensionSize[k]);
        if (k != properties.alignedShape.numDimensions - 1) {
          aligne_shape += ", ";
        }
      }
      aligne_shape += " )";
      ss << ", alignedShape: " << aligne_shape;
      ss << ", tensorLayout: " << properties.tensorLayout;
      ss << ", tensorType: " << properties.tensorType;
    }

    // 5. getOutputCount
    int output_count = 0;
    if (0 != hbDNNGetOutputCount(&output_count, dnnHandle)) {
      LOGE << "hawDNN_getOutputCount failed";
      return -1;
    }
    ss << ", output num: " << output_count;

    // 6. getOutputTensorProperties
    for (int i = 0; i < output_count; i++) {
      hbDNNTensorProperties properties;
      if (0 != hbDNNGetOutputTensorProperties(&properties, dnnHandle, i)) {
        LOGE << "hbDNNGetOutputTensorProperties failed";
      }

      std::string valid_shape = "( ";
      for (int k = 0; k < properties.validShape.numDimensions; k++) {
        valid_shape += std::to_string(properties.validShape.dimensionSize[k]);
        if (k != properties.validShape.numDimensions - 1) {
          valid_shape += ", ";
        }
      }
      valid_shape += " )";
      ss << ", output[" << i << "] validShape: " << valid_shape;

      std::string aligne_shape = "( ";
      for (int k = 0; k < properties.alignedShape.numDimensions; k++) {
        aligne_shape +=
            std::to_string(properties.alignedShape.dimensionSize[k]);
        if (k != properties.alignedShape.numDimensions - 1) {
          aligne_shape += ", ";
        }
      }
      aligne_shape += " )";
      ss << ", alignedShape: " << aligne_shape;
      ss << ", tensorLayout: " << properties.tensorLayout;
      ss << ", tensorType: " << properties.tensorType;
    }
    LOGD << ss.str();
  }
  return 0;
}

// 根据bool参数调用BPU加载模型接口，若成功，则将此handle维护到map
void *DNNInferenceEngine::LoadModelFileHandle(const char *model_file_name,
                                              bool is_packed_model) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_packed_model) {
    hbPackedDNNHandle_t dnn_handle;
    int ret = hbDNNInitializeFromFiles(&dnn_handle, &model_file_name, 1);
    LOGD << "hbDNNInitializeFromFiles, " << model_file_name
         << ", ret = " << ret;
    if (ret == 0) {
      is_packed_model_file_handle_[dnn_handle] = is_packed_model;
      const char **model_name_list;
      int model_count = 0;
      hbDNNGetModelNameList(&model_name_list, &model_count, dnn_handle);
      for (int i = 0; i < model_count; i++) {
        LOGD << "modelName = " << model_name_list[i];
      }
      return dnn_handle;
    } else {
      LOGF << "Load Model Failed, model_path: " << model_file_name;
      return nullptr;
    }
  } else {
    hbPackedDNNHandle_t packed_dnn_handle;
    std::ifstream ifs(model_file_name, std::ifstream::binary);
    if (!ifs.is_open()) {
      LOGF << "Open file failed, model_path: " << model_file_name;
      return nullptr;
    }
    ifs.seekg(0, ifs.end);
    int32_t length = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    char *buffer = new char[length];
    ifs.read(buffer, length);
    const void *file_data = static_cast<void *>(buffer);
    ifs.close();

    int ret =
        hbDNNInitializeFromDDR(&packed_dnn_handle, &file_data, &length, 1);
    LOGD << "hbDNNInitializeFromDDR, " << model_file_name << ", ret = " << ret;
    if (ret == 0) {
      is_packed_model_file_handle_[packed_dnn_handle] = is_packed_model;
      return packed_dnn_handle;
    } else {
      LOGF << "Load Model Failed, model_path: " << model_file_name;
      return nullptr;
    }
  }
}

// 判断model_file_handle是packed，则根据model_name返回对应的model_handle
// 非packed,直接返回；packed, 根据model_name取model_handle
void *DNNInferenceEngine::LoadModelHandle(void *model_file_handle,
                                          const char *model_name) {
  if (is_packed_model_file_handle_.end() ==
      is_packed_model_file_handle_.find(model_file_handle)) {
    LOGE << "not found model_file_handle";
    return nullptr;
  }
  hbPackedDNNHandle_t packed_dnn_handle =
      static_cast<hbPackedDNNHandle_t>(model_file_handle);
  hbDNNHandle_t dnnHandle;
  int ret = hbDNNGetModelHandle(&dnnHandle, packed_dnn_handle, model_name);
  if (0 == ret) {
    LOGE << "found model: " << model_name;
    return dnnHandle;
  } else {
    LOGE << "not found model: " << model_name;
    return nullptr;
  }
}

// 直接返回
int DNNInferenceEngine::ReleaseModelHandle(void *model_handle) { return 0; }

// 判断是否packed，调用对应的BPU接口
int DNNInferenceEngine::ReleaseModelFileHandle(void *model_file_handle) {
  if (is_packed_model_file_handle_.end() ==
      is_packed_model_file_handle_.find(model_file_handle)) {
    LOGE << "not found model_file_handle";
    return -1;
  }
  int ret = hbDNNRelease(static_cast<hbPackedDNNHandle_t>(model_file_handle));
  if (0 == ret) {
    is_packed_model_file_handle_.erase(model_file_handle);
  } else {
    LOGE << "release packed model file handle failed.";
    return -1;
  }
  return 0;
}

int DNNInferenceEngine::GetModelInputInfo(
    void *model_handle, std::vector<TensorProperties> &input_modelinfo) {
  if (model_handle == nullptr) {
    return -1;
  }
  // 转换DNN
  hbDNNHandle_t dnn_model_handle = static_cast<hbDNNHandle_t>(model_handle);
  int input_num = 0;
  int ret = hbDNNGetInputCount(&input_num, dnn_model_handle);
  if (ret != 0) {
    return -1;
  }
  input_modelinfo.resize(input_num);
  for (int input_idx = 0; input_idx < input_num; input_idx++) {
    hbDNNTensorProperties input;
    ret = hbDNNGetInputTensorProperties(&input, dnn_model_handle, input_idx);
    TensorProperties &model_info = input_modelinfo[input_idx];
    ConvertModelInfo(input, model_info);
  }
  return 0;
}

int DNNInferenceEngine::GetModelOutputInfo(
    void *model_handle, std::vector<TensorProperties> &output_modelinfo) {
  if (model_handle == nullptr) {
    return -1;
  }
  // 转换DNN
  hbDNNHandle_t dnn_model_handle = static_cast<hbDNNHandle_t>(model_handle);
  int output_num = 0;
  int ret = hbDNNGetOutputCount(&output_num, dnn_model_handle);
  if (0 != ret) {
    LOGE << "GetOutputCount failed.";
    return -1;
  }
  output_modelinfo.resize(output_num);
  for (int output_idx = 0; output_idx < output_num; output_idx++) {
    hbDNNTensorProperties output;
    hbDNNGetOutputTensorProperties(&output, dnn_model_handle, output_idx);
    TensorProperties &model_info = output_modelinfo[output_idx];
    ConvertModelInfo(output, model_info);
  }
  return 0;
}

int DNNInferenceEngine::AllocModelTensor(std::vector<Tensor> &tensors,
                                         bool need_alloc, int batch) {
  int layer_num = tensors.size();
  for (int layer_idx = 0; layer_idx < layer_num; layer_idx++) {
    Tensor &tensor = tensors[layer_idx];
    TensorProperties info = tensor.properties;
    // sys_mem
    int h_idx, w_idx, c_idx;
    if (0 != GetHWCIndex(info.tensor_type, info.tensor_layout, &h_idx, &w_idx,
                         &c_idx)) {
      LOGE << "GetHWCIndex Failed. tensor_type = " << info.tensor_type
           << ", tensor_layout = " << info.tensor_layout;
      return -1;
    }
    int height = info.valid_shape.dimension_size[h_idx];
    int stride = info.aligned_shape.dimension_size[w_idx];
    switch (info.tensor_type) {
      case IMG_TYPE_NV12: {
        // shape.d[h_idx] aligned_shape.d[w_idx]
        int y_length = height * stride;
        int uv_length = height / 2 * stride;
        if (need_alloc) {
          hbSysMem mem;
          hbSysAllocCachedMem(&mem, y_length + uv_length);
          tensor.sys_mem[0].phy_addr = mem.phyAddr;
          tensor.sys_mem[0].vir_addr = mem.virAddr;
          tensor.sys_mem[0].mem_size = mem.memSize;
        }
        break;
      }
      case IMG_TYPE_NV12_SEPARATE: {
        int y_length = height * stride;
        int uv_length = height / 2 * stride;
        if (need_alloc) {
          hbSysMem mem_0, mem_1;
          hbSysAllocCachedMem(&mem_0, y_length);
          hbSysAllocCachedMem(&mem_1, uv_length);
          tensor.sys_mem[0].phy_addr = mem_0.phyAddr;
          tensor.sys_mem[0].vir_addr = mem_0.virAddr;
          tensor.sys_mem[0].mem_size = mem_0.memSize;
          tensor.sys_mem[1].phy_addr = mem_1.phyAddr;
          tensor.sys_mem[1].vir_addr = mem_1.virAddr;
          tensor.sys_mem[1].mem_size = mem_1.memSize;
        }
        break;
      }

      case TENSOR_TYPE_S8:
      case TENSOR_TYPE_U8:
      case TENSOR_TYPE_F16:
      case TENSOR_TYPE_S16:
      case TENSOR_TYPE_U16:
      case TENSOR_TYPE_F32:
      case TENSOR_TYPE_S32:
      case TENSOR_TYPE_U32:
      case TENSOR_TYPE_F64:
      case TENSOR_TYPE_S64:
      case TENSOR_TYPE_U64: {
        int elem_size = 1;
        if (info.tensor_type >= TENSOR_TYPE_F16 &&
            info.tensor_type <= TENSOR_TYPE_U16) {
          elem_size = 2;
        } else if (info.tensor_type >= TENSOR_TYPE_F32 &&
                   info.tensor_type <= TENSOR_TYPE_U32) {
          elem_size = 4;
        } else if (info.tensor_type >= TENSOR_TYPE_F64 &&
                   info.tensor_type <= TENSOR_TYPE_U64) {
          elem_size = 8;
        }
        int channel_stride = info.aligned_shape.dimension_size[c_idx];
        // batch针对roi的输出使用
        int length = elem_size * batch;
        for (int j = 0; j < info.aligned_shape.num_dimensions; j++) {
          length *= info.aligned_shape.dimension_size[j];
        }

        // 16对齐
        length = ((length + (16 - 1)) / 16 * 16);
        LOGD << "output tensor len = " << length;
        if (need_alloc) {
          hbSysMem mem;
          hbSysAllocCachedMem(&mem, length);
          tensor.sys_mem[0].phy_addr = mem.phyAddr;
          tensor.sys_mem[0].vir_addr = mem.virAddr;
          tensor.sys_mem[0].mem_size = mem.memSize;
          LOGD << "mem tensor len = " << length;
        }
        break;
      }
      default:
        LOGE << "not support tensor_type: " << info.tensor_type;
        return -1;
    }
  }
  return 0;
}

int DNNInferenceEngine::FreeTensor(std::vector<Tensor> &tensors) {
  for (size_t i = 0; i < tensors.size(); i++) {
    Tensor &tensor = tensors[i];
    if (tensor.sys_mem_type != MEM_TYPE_ALLOCATED) {
      continue;
    }
    switch (tensor.properties.tensor_type) {
      case IMG_TYPE_NV12:
      case TENSOR_TYPE_S4:
      case TENSOR_TYPE_U4:
      case TENSOR_TYPE_S8:
      case TENSOR_TYPE_U8:
      case TENSOR_TYPE_F16:
      case TENSOR_TYPE_S16:
      case TENSOR_TYPE_U16:
      case TENSOR_TYPE_F32:
      case TENSOR_TYPE_S32:
      case TENSOR_TYPE_U32:
      case TENSOR_TYPE_F64:
      case TENSOR_TYPE_S64:
      case TENSOR_TYPE_U64: {
        hbSysMem mem = {tensor.sys_mem[0].phy_addr, tensor.sys_mem[0].vir_addr,
                        tensor.sys_mem[0].mem_size};
        hbSysFreeMem(&mem);
        break;
      }
      case IMG_TYPE_NV12_SEPARATE: {
        hbSysMem mem_0 = {tensor.sys_mem[0].phy_addr,
                          tensor.sys_mem[0].vir_addr,
                          tensor.sys_mem[0].mem_size};
        hbSysMem mem_1 = {tensor.sys_mem[1].phy_addr,
                          tensor.sys_mem[1].vir_addr,
                          tensor.sys_mem[1].mem_size};
        hbSysFreeMem(&mem_0);
        hbSysFreeMem(&mem_1);
        break;
      }
      default:
        LOGE << "not support tensor_type: " << tensor.properties.tensor_type;
        break;
    }
  }
  tensors.clear();
  return 0;
}

void DNNInferenceEngine::ConvertModelInfo(
    const hbDNNTensorProperties &node_info, TensorProperties &model_info) {
  // validShape && alignedShape
  auto &valid_shape = model_info.valid_shape;
  auto &aligned_shape = model_info.aligned_shape;
  valid_shape.num_dimensions = node_info.validShape.numDimensions;
  aligned_shape.num_dimensions = node_info.alignedShape.numDimensions;
  for (int dim = 0; dim < valid_shape.num_dimensions; dim++) {
    valid_shape.dimension_size[dim] = node_info.validShape.dimensionSize[dim];
    aligned_shape.dimension_size[dim] =
        node_info.alignedShape.dimensionSize[dim];
  }
  // tensor_layout
  model_info.tensor_layout = static_cast<TensorLayout>(node_info.tensorLayout);

  // tensor_type, 需要映射
  model_info.tensor_type = static_cast<DataType>(node_info.tensorType);
  // shift && scale
  model_info.shift.shift_len = node_info.shift.shiftLen;
  model_info.shift.shift_data = node_info.shift.shiftData;
  model_info.scale.scale_len = node_info.scale.scaleLen;
  model_info.scale.scale_data = node_info.scale.scaleData;
  // quanti_type
  model_info.quanti_type = static_cast<QuantiType>(node_info.quantiType);
  return;
}

int DNNInferenceEngine::ConvertTensor2DNNTensor(const Tensor &tensor,
                                                hbDNNTensor &dnn_tensor) {
  // validShape && alignedShape
  dnn_tensor.properties.tensorType =
      static_cast<int32_t>(tensor.properties.tensor_type);
  dnn_tensor.properties.tensorLayout =
      static_cast<int32_t>(tensor.properties.tensor_layout);
  dnn_tensor.properties.validShape.numDimensions =
      tensor.properties.valid_shape.num_dimensions;
  for (int dim = 0; dim < dnn_tensor.properties.validShape.numDimensions;
       dim++) {
    dnn_tensor.properties.validShape.dimensionSize[dim] =
        tensor.properties.valid_shape.dimension_size[dim];
  }

  dnn_tensor.properties.alignedShape.numDimensions =
      tensor.properties.aligned_shape.num_dimensions;
  for (int dim = 0; dim < dnn_tensor.properties.alignedShape.numDimensions;
       dim++) {
    dnn_tensor.properties.alignedShape.dimensionSize[dim] =
        tensor.properties.aligned_shape.dimension_size[dim];
  }

  switch (tensor.properties.tensor_type) {
    case IMG_TYPE_NV12:
    case TENSOR_TYPE_S8:
    case TENSOR_TYPE_U8:
    case TENSOR_TYPE_S16:
    case TENSOR_TYPE_U16:
    case TENSOR_TYPE_F32:
    case TENSOR_TYPE_S32:
    case TENSOR_TYPE_U32:
    case TENSOR_TYPE_F64:
    case TENSOR_TYPE_S64:
    case TENSOR_TYPE_U64:
      dnn_tensor.sysMem[0].phyAddr = tensor.sys_mem[0].phy_addr;
      dnn_tensor.sysMem[0].virAddr = tensor.sys_mem[0].vir_addr;
      dnn_tensor.sysMem[0].memSize = tensor.sys_mem[0].mem_size;
      break;
    case IMG_TYPE_NV12_SEPARATE:
      dnn_tensor.sysMem[0].phyAddr = tensor.sys_mem[0].phy_addr;
      dnn_tensor.sysMem[0].virAddr = tensor.sys_mem[0].vir_addr;
      dnn_tensor.sysMem[0].memSize = tensor.sys_mem[0].mem_size;
      dnn_tensor.sysMem[1].phyAddr = tensor.sys_mem[1].phy_addr;
      dnn_tensor.sysMem[1].virAddr = tensor.sys_mem[1].vir_addr;
      dnn_tensor.sysMem[1].memSize = tensor.sys_mem[1].mem_size;
      break;
    default:
      LOGE << "not support tensor_type: " << tensor.properties.tensor_type;
      break;
  }
  return 0;
}

int DNNInferenceEngine::InferenceTaskCore(
    std::shared_ptr<InferenceEngineTask> task) {
  // 刷新cache，确保input tensor的数据写入bpu内存空间
  // 从task数据中封装BPU_RUN_CTRL_S, 调用同步接口进行预测；
  int ret = 0;
  // 判断tensor input or roi input
  std::shared_ptr<TensorInferenceEngineTask> tensor_task =
      std::dynamic_pointer_cast<TensorInferenceEngineTask>(task);
  if (tensor_task != nullptr) {
    // tensor task
    // Tensor转换BPU_TENSOR_S
    int input_layer = tensor_task->input_tensors_.size();
    int output_layer = tensor_task->output_tensors_.size();
    LOGD << "input_layer: " << input_layer;
    LOGD << "output_layer: " << output_layer;
    if (input_layer <= 0 || output_layer <= 0) {
      LOGD << "prepare task failed, no need run model";
      return -1;
    }
    std::vector<hbDNNTensor> dnn_input_tensors(input_layer);
    for (int i = 0; i < input_layer; i++) {
      Tensor &tensor = tensor_task->input_tensors_[i];
      hbDNNTensor &dnn_tensor = dnn_input_tensors[i];
      ConvertTensor2DNNTensor(tensor, dnn_tensor);
      // flush tensor
      switch (tensor.properties.tensor_type) {
        case IMG_TYPE_NV12:
        case TENSOR_TYPE_S8:
          hbSysFlushMem(&dnn_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
          break;
        case IMG_TYPE_NV12_SEPARATE:
          hbSysFlushMem(&dnn_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
          hbSysFlushMem(&dnn_tensor.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
          break;
        default:
          LOGE << "not support tensor_type: " << tensor.properties.tensor_type;
          break;
      }
    }

    hbDNNTensor *dnn_output_tensors = new hbDNNTensor[output_layer];
    for (int i = 0; i < output_layer; i++) {
      Tensor &tensor = tensor_task->output_tensors_[i];
      hbDNNTensor &dnn_tensor = dnn_output_tensors[i];
      ConvertTensor2DNNTensor(tensor, dnn_tensor);
    }
    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    infer_ctrl_param.bpuCoreId = tensor_task->run_ctrls_.bpu_core_id;
    infer_ctrl_param.dspCoreId = tensor_task->run_ctrls_.dsp_core_id;
    infer_ctrl_param.priority = tensor_task->run_ctrls_.priority;
    infer_ctrl_param.more = tensor_task->run_ctrls_.more;
    ret =
        hbDNNInfer(&task_handle, &dnn_output_tensors, dnn_input_tensors.data(),
                   static_cast<hbDNNHandle_t>(tensor_task->model_handle_),
                   &infer_ctrl_param);
    delete dnn_output_tensors;
    if (0 != ret) {
      LOGE << "hbDNNInfer failed.";
      return -1;
    }
    // wait task done
    ret = hbDNNWaitTaskDone(task_handle, 0);  // wait until task end
    if (0 != ret) {
      LOGE << "hbDNNWaitTaskDone failed, ret_code = " << ret;
    }
    ret = hbDNNReleaseTask(task_handle);
    if (0 != ret) {
      LOGE << "hbDNNReleaseTask failed, ret_code = " << ret;
    }
    task_handle = nullptr;
  } else {
    // roi tensor
    std::shared_ptr<RoiInferenceEngineTask> roi_task =
        std::dynamic_pointer_cast<RoiInferenceEngineTask>(task);
    if (roi_task->input_tensors_.size() <= 0 ||
        roi_task->output_tensors_.size() <= 0) {
      LOGD << "no need run model";
      return -1;
    }
    HOBOT_CHECK(roi_task->input_tensors_.size() == 1)
        << "only support input one layer";
    // input_tensors_转换down_scales[]
    int input_layer = roi_task->input_tensors_[0].size();
    LOGD << "input_layer = " << input_layer;
    std::vector<hbDNNTensor> dnn_input_tensors(input_layer);
    for (int layer = 0; layer < input_layer; layer++) {
      Tensor &input_tensor = roi_task->input_tensors_[0][layer];
      hbDNNTensor &dnn_tensor = dnn_input_tensors[layer];
      ConvertTensor2DNNTensor(input_tensor, dnn_tensor);
      // flush tensor
      switch (input_tensor.properties.tensor_type) {
        case IMG_TYPE_NV12:
        case TENSOR_TYPE_S8:
          hbSysFlushMem(&dnn_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
          break;
        case IMG_TYPE_NV12_SEPARATE:
          hbSysFlushMem(&dnn_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
          hbSysFlushMem(&dnn_tensor.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
          break;
        default:
          LOGE << "not support tensor_type: "
               << input_tensor.properties.tensor_type;
          break;
      }
    }
    // output_tensors_
    int output_layer = roi_task->output_tensors_.size();
    hbDNNTensor *dnn_output_tensors = new hbDNNTensor[output_layer];
    for (int out_layer = 0; out_layer < output_layer; out_layer++) {
      ConvertTensor2DNNTensor(roi_task->output_tensors_[out_layer],
                              dnn_output_tensors[out_layer]);
    }

    // InferBox封装有效的BPU_BBOX送入模型预测
    std::vector<hbDNNRoi> dnn_boxes;
    for (auto &infer_box : roi_task->roi_box_) {
      if (infer_box.resizable) {  // valid box
        LOGD << infer_box.x1 << ", " << infer_box.y1 << ", " << infer_box.x2
             << ", " << infer_box.y2 << ", w = " << infer_box.x2 - infer_box.x1
             << ", h = " << infer_box.y2 - infer_box.y1;
        dnn_boxes.push_back(hbDNNRoi{static_cast<int32_t>(infer_box.x1),
                                     static_cast<int32_t>(infer_box.y1),
                                     static_cast<int32_t>(infer_box.x2),
                                     static_cast<int32_t>(infer_box.y2)});
        if (infer_box.scaling != 0) {
          infer_box.x1 *= infer_box.scaling;
          infer_box.y1 *= infer_box.scaling;
          infer_box.x2 *= infer_box.scaling;
          infer_box.y2 *= infer_box.scaling;
        }
      }
    }

    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    infer_ctrl_param.bpuCoreId = roi_task->run_ctrls_.bpu_core_id;
    infer_ctrl_param.dspCoreId = roi_task->run_ctrls_.dsp_core_id;
    infer_ctrl_param.priority = roi_task->run_ctrls_.priority;
    infer_ctrl_param.more = roi_task->run_ctrls_.more;
    hbDNNTaskHandle_t task_handle = nullptr;
    LOGD << "start dnn_roiinfer";
    ret = hbDNNRoiInfer(
        &task_handle, &dnn_output_tensors, dnn_input_tensors.data(),
        dnn_boxes.data(), dnn_boxes.size(),
        static_cast<hbDNNHandle_t>(roi_task->model_handle_), &infer_ctrl_param);
    if (ret != 0) {
      LOGE << "hbDNNRoiInfer failed, ret_code = " << ret;
      return -1;
    } else {
      LOGD << "hbDNNRoiInfer Suc.";
    }
    delete dnn_output_tensors;
    // wait task done
    ret = hbDNNWaitTaskDone(task_handle, 0);  // wait until task end
    if (0 != ret) {
      LOGE << "hbDNNWaitTaskDone failed, ret_code = " << ret;
    }
    ret = hbDNNReleaseTask(task_handle);
    if (0 != ret) {
      LOGE << "hbDNNReleaseTask failed, ret_code = " << ret;
    }
    task_handle = nullptr;
  }
  LOGD << "run model end, ret: " << ret;
  return ret;
}

int DNNInferenceEngine::ResultTaskCore(
    std::shared_ptr<InferenceEngineTask> task) {
  // 刷新output flush等后处理之前的操作
  int output_layer = task->output_tensors_.size();
  hbDNNTensor *dnn_output_tensors = new hbDNNTensor[output_layer];
  for (int i = 0; i < output_layer; i++) {
    Tensor &tensor = task->output_tensors_[i];
    hbDNNTensor &dnn_tensor = dnn_output_tensors[i];
    ConvertTensor2DNNTensor(tensor, dnn_tensor);
  }

  int ret = 0;
  for (int i = 0; i < output_layer; i++) {
    auto &dnn_tensor = dnn_output_tensors[i];
    ret = hbSysFlushMem(&dnn_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    if (ret != 0) break;
  }
  delete[] dnn_output_tensors;
  return ret;
}

int DNNInferenceEngine::PrepareRoiTensor(
    const Inferencer *const infer, const std::vector<Tensor> &pyramid_tensors,
    std::vector<InferBBox> &rois, std::vector<Tensor> &tensors) {
  int h_idx, w_idx, c_idx;
  int ret = GetHWCIndex(infer->input_model_info_[0].tensor_type,
                        infer->input_model_info_[0].tensor_layout, &h_idx,
                        &w_idx, &c_idx);
  if (0 != ret) {
    LOGE << "PrepareRoiTesor: GetHWCIndex Failed, tensor_type = "
         << infer->input_model_info_[0].tensor_type
         << ", tensor_layout = " << infer->input_model_info_[0].tensor_layout;
    return -1;
  }

  int32_t ds_pym_layer = pyramid_tensors.size();
  if (ds_pym_layer <= 0) {
    LOGE << "Invalid ds_pym_layer.";
    return -1;
  }
  int32_t roi_size = rois.size();
  int32_t dst_w = infer->input_model_info_[0].valid_shape.dimension_size[w_idx];
  int32_t dst_h = infer->input_model_info_[0].valid_shape.dimension_size[h_idx];
  LOGD << "PrepareRoiTensor, dst_h = " << dst_h << ", dst_w = " << dst_w;

  for (int i = 0; i < roi_size; i++) {
    auto &box = rois[i];
    if (box.x1 >= box.x2 || box.y1 >= box.y2 || (box.x1 <= 0 && box.x2 <= 0) ||
        (box.y1 <= 0 && box.y2 <= 0)) {
      LOGE << "input box: " << box.x1 << ", " << box.y1 << ", " << box.x2
           << ", " << box.y2 << " is invalid";
      box.resizable = false;
      continue;
    }
    int pyr_level = 0;
    int y_left, y_top, y_right, y_bottom;
    int ret = find_pyramid_layer(dst_h, dst_w, box, pyr_level, y_left, y_top,
                                 y_right, y_bottom);
    if (0 != ret) {
      box.resizable = false;
      LOGD << "[WARNING] can not find suitable pym-level";
    } else {
      if ((pyr_level * 4) >= ds_pym_layer) {
        box.resizable = false;
        LOGE << "suitable pym-level: " << pyr_level * 4
             << "exceeds input downscale layer's number: " << ds_pym_layer;
      } else {
        LOGD << "pyr_level = " << pyr_level;
        tensors.push_back(pyramid_tensors[pyr_level * 4]);
        box.x1 = y_left;
        box.y1 = y_top;
        box.x2 = y_right;
        box.y2 = y_bottom;
        box.scaling = 1 << pyr_level;
      }
    }
  }
  return 0;
}

typedef struct section_s {
  float start;
  float end;
} section_t;

float DNNInferenceEngine::get_max_scale(const int &dst_h, const int &dst_w,
                                        const int &height, const int &width) {
  section_t sec_1, sec_2, sec_3;
  sec_1.start = dst_w * 1.0 / (8.0 * width);
  sec_1.end = 2.0 * dst_w / width;
  sec_2.start = dst_h * 1.0 / (8.0 * height);
  sec_2.end = 2.0 * dst_h / height;

  //  calculate intersection of 1) and 2)
  section_t sec_12_intersec;
  sec_12_intersec.start = std::max(sec_1.start, sec_2.start);
  sec_12_intersec.end = std::min(sec_1.end, sec_2.end);
  if (width >= height) {
    sec_3.start = 18.0 / height;
    sec_3.end = 256.0 / width;
  } else {
    sec_3.start = 18.0 / width;
    sec_3.end = 256.0 / height;
  }

  // calculate intersection of 1) 2) and 3)
  section_t sec_123_intersec;
  sec_123_intersec.start = std::max(sec_3.start, sec_12_intersec.start);
  sec_123_intersec.end = std::min(sec_3.end, sec_12_intersec.end);
  return (sec_123_intersec.end - 0.0001);
}

int DNNInferenceEngine::check_roi_valid(int left, int top, int right,
                                        int bottom, int dst_h, int dst_w) {
  int box_width = right - left + 1;
  int box_height = bottom - top + 1;
  if (box_width < 16 || box_width > 256 || box_height < 16 ||
      box_height > 256) {
    LOGD << "[WARNING] roi not match 16 <= x <= 256";

    return -1;
  }
  if ((box_width / dst_w) >= 2 || (box_height / dst_h) >= 2 ||
      (dst_w / box_width) >= 8 || (dst_h / box_height) >= 8) {
    LOGD << "[WARNING] roi not match 2 < scale < 8";
    return -1;
  }
  return 0;
}

int DNNInferenceEngine::find_pyramid_layer(const int &dst_h, const int &dst_w,
                                           const InferBBox &box, int &pym_layer,
                                           int &left, int &top, int &right,
                                           int &bottom) {
  int width = box.x2 - box.x1 + 1;
  int height = box.y2 - box.y1 + 1;
  int64_t y_left, y_top, y_right, y_bottom;

  uint8_t recal = 1;
  float scale = get_max_scale(dst_h, dst_w, height, width);
  for (pym_layer = 0; pym_layer <= BPU_DOWN_SCALE_LEVEL_MAX; ++pym_layer) {
    if (scale >= BPU_DOWN_SCALE(pym_layer)) {
      recal = 0;

      y_left = static_cast<int64_t>(/*must be even*/
                                    BPU_DOWN_SCALE(pym_layer) * box.x1 + 1) &
               (~0x1);
      y_top = static_cast<int64_t>(/*must be even*/
                                   BPU_DOWN_SCALE(pym_layer) * box.y1 + 1) &
              (~0x1);
      y_right = static_cast<int64_t>(/*must be odd*/
                                     BPU_DOWN_SCALE(pym_layer) * box.x2 - 1) |
                (0x1);
      y_bottom = static_cast<int64_t>(/*must be odd*/
                                      BPU_DOWN_SCALE(pym_layer) * box.y2 - 1) |
                 (0x1);
      break;
    }
  }

  if (recal) {
    LOGD << "[WARNING] No suitable pyramid for this bounding box";
    return -1;
  }

  bool no_pyramid_left = false;
  int is_invalid_roi = 0;
  do {
    is_invalid_roi =
        check_roi_valid(y_left, y_top, y_right, y_bottom, dst_h, dst_w);
    if (is_invalid_roi != 0) {
      LOGD << "check roi invalid : " << y_left << ", " << y_top << ", "
           << y_right << ", " << y_bottom;
      if (++pym_layer > BPU_DOWN_SCALE_LEVEL_MAX) no_pyramid_left = true;
      if (no_pyramid_left) {
        LOGD << "[WARNING] No pyramid left, can not search a suitable "
                "pyramid for this bounding box";
        return -1;
      }
      y_left = static_cast<int64_t>(/*must be even*/
                                    BPU_DOWN_SCALE(pym_layer) * box.x1 + 1) &
               (~0x1);
      y_top = static_cast<int64_t>(/*must be even*/
                                   BPU_DOWN_SCALE(pym_layer) * box.y1 + 1) &
              (~0x1);
      y_right = static_cast<int64_t>(/*must be odd*/
                                     BPU_DOWN_SCALE(pym_layer) * box.x2 - 1) |
                (0x1);
      y_bottom = static_cast<int64_t>(/*must be odd*/
                                      BPU_DOWN_SCALE(pym_layer) * box.y2 - 1) |
                 (0x1);
    }
  } while (is_invalid_roi != 0);

  left = y_left;
  top = y_top;
  right = y_right;
  bottom = y_bottom;
  return 0;
}

int DNNInferenceEngine::ResizeImage(const Tensor &nv12_tensor,
                                    Tensor &input_tensor, const InferBBox &rois,
                                    const ResizeCtrlParam resize_param) {
  bool is_modify_shape = false;
  int h_idx, w_idx, c_idx;
  int ret = GetHWCIndex(input_tensor.properties.tensor_type,
                        input_tensor.properties.tensor_layout, &h_idx, &w_idx,
                        &c_idx);
  if (0 != ret) {
    LOGE << "ResizeImage: GetHWCIndex Failed, tensor_type = "
         << input_tensor.properties.tensor_type
         << ", tensor_layout = " << input_tensor.properties.tensor_layout;
    return -1;
  }
  if (input_tensor.properties.aligned_shape.dimension_size[c_idx] == 4) {
    input_tensor.properties.aligned_shape.dimension_size[c_idx] = 3;
    is_modify_shape = true;
  }
  hbDNNTaskHandle_t task_handle;
  hbDNNTensor dnn_nv12_tensor;
  ConvertTensor2DNNTensor(nv12_tensor, dnn_nv12_tensor);
  hbDNNTensor dnn_inout_tensor;
  ConvertTensor2DNNTensor(input_tensor, dnn_inout_tensor);
  hbDNNRoi input_roi = {
      static_cast<int32_t>(rois.x1), static_cast<int32_t>(rois.y1),
      static_cast<int32_t>(rois.x2), static_cast<int32_t>(rois.y2)};
  hbDNNResizeCtrlParam ctrl_param;
  HB_DNN_INITIALIZE_RESIZE_CTRL_PARAM(&ctrl_param);
  ctrl_param.resizeType =
      static_cast<hbDNNResizeType>(resize_param.resize_type);
  ctrl_param.bpuCoreId = resize_param.bpu_core_id;
  ret = hbDNNResize(&task_handle, &dnn_inout_tensor, &dnn_nv12_tensor,
                    &input_roi, &ctrl_param);
  if (ret != 0) {
    LOGE << "DNN hbDNNResize failed";
    return -1;
  }
  // wait task done
  ret = hbDNNWaitTaskDone(task_handle, 0);  // wait until task end
  if (0 != ret) {
    LOGE << "hbDNNWaitTaskDone failed, ret_code = " << ret;
  }
  ret = hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    LOGE << "hbDNNReleaseTask failed, ret_code = " << ret;
  }
  if (is_modify_shape) {
    input_tensor.properties.aligned_shape.dimension_size[c_idx] = 4;
    is_modify_shape = false;
  }
  task_handle = nullptr;
  return ret;
}

}  // namespace inference
