/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_engine
 * @file   inference_engine.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/inference_engine_bpu.h"
#include "hobotlog/hobotlog.hpp"
#include "bpu_predict_extension.h"

namespace inference {

std::map<BPU_LAYOUT_E, TensorLayout> BPUInferenceEngine::layout_table_ = {
      {BPU_LAYOUT_NONE, LAYOUT_NONE},
      {BPU_LAYOUT_NHWC, LAYOUT_NHWC},
      {BPU_LAYOUT_NCHW, LAYOUT_NCHW}};
std::map<BPU_DATA_TYPE_E, DataType> BPUInferenceEngine::datatype_table_ = {
      {BPU_TYPE_IMG_Y, IMG_TYPE_Y},
      {BPU_TYPE_IMG_YUV_NV12, IMG_TYPE_NV12},
      {BPU_TYPE_IMG_YUV444, IMG_TYPE_YUV444},
      {BPU_TYPE_IMG_BGR, IMG_TYPE_BGR},
      {BPU_TYPE_IMG_BGRP, TENSOR_TYPE_MAX},  // 暂不支持
      {BPU_TYPE_IMG_RGB, IMG_TYPE_RGB},
      {BPU_TYPE_IMG_RGBP, TENSOR_TYPE_MAX},  // 暂不支持
      {BPU_TYPE_IMG_NV12_SEPARATE, IMG_TYPE_NV12_SEPARATE},
      {BPU_TYPE_TENSOR_U8, TENSOR_TYPE_U8},
      {BPU_TYPE_TENSOR_S8, TENSOR_TYPE_S8},
      {BPU_TYPE_TENSOR_F32, TENSOR_TYPE_F32},
      {BPU_TYPE_TENSOR_S32, TENSOR_TYPE_S32},
      {BPU_TYPE_TENSOR_U32, TENSOR_TYPE_U32},
      {BPU_TYPE_TENSOR_S64, TENSOR_TYPE_S64},
      {BPU_TYPE_MAX, TENSOR_TYPE_MAX}};
std::map<TensorLayout, BPU_LAYOUT_E> BPUInferenceEngine::layout2bpu_table_ = {
      {LAYOUT_NONE, BPU_LAYOUT_NONE},
      {LAYOUT_NHWC, BPU_LAYOUT_NHWC},
      {LAYOUT_NCHW, BPU_LAYOUT_NCHW}};
std::map<DataType, BPU_DATA_TYPE_E> BPUInferenceEngine::datatype2bpu_table_ = {
      {IMG_TYPE_Y, BPU_TYPE_IMG_Y},
      {IMG_TYPE_NV12, BPU_TYPE_IMG_YUV_NV12},
      {IMG_TYPE_YUV444, BPU_TYPE_IMG_YUV444},
      {IMG_TYPE_BGR, BPU_TYPE_IMG_BGR},
      {IMG_TYPE_RGB, BPU_TYPE_IMG_RGB},
      {IMG_TYPE_NV12_SEPARATE, BPU_TYPE_IMG_NV12_SEPARATE},
      {TENSOR_TYPE_U8, BPU_TYPE_TENSOR_U8},
      {TENSOR_TYPE_S8, BPU_TYPE_TENSOR_S8},
      {TENSOR_TYPE_F32, BPU_TYPE_TENSOR_F32},
      {TENSOR_TYPE_S32, BPU_TYPE_TENSOR_S32},
      {TENSOR_TYPE_U32, BPU_TYPE_TENSOR_U32},
      {TENSOR_TYPE_S64, BPU_TYPE_TENSOR_S64},
      {TENSOR_TYPE_MAX, BPU_TYPE_MAX}};

void BPUInferenceEngine::ConvertModelInfo(const BPU_MODEL_NODE_S &node_info,
                                          TensorProperties &model_info) {
  // validShape && alignedShape
  auto &valid_shape = model_info.valid_shape;
  auto &aligned_shape = model_info.aligned_shape;
  valid_shape.num_dimensions = node_info.shape.ndim;
  aligned_shape.num_dimensions = node_info.aligned_shape.ndim;
  for (int dim = 0; dim < valid_shape.num_dimensions; dim++)
  {
    valid_shape.dimension_size[dim] = node_info.shape.d[dim];
    aligned_shape.dimension_size[dim] = node_info.aligned_shape.d[dim];
  }
  // tensor_layout, 需要映射
  model_info.tensor_layout = layout_table_[node_info.shape.layout];

  // tensor_type, 需要映射
  model_info.tensor_type = datatype_table_[node_info.data_type];
  // shift
  model_info.shift.shift_len = node_info.shift_len;
  model_info.shift.shift_data = node_info.shifts;
  return;
}

// 根据bool参数调用BPU加载模型接口，若成功，则将此handle维护到map
void *BPUInferenceEngine::LoadModelFileHandle(
    const char *model_file_name, bool is_packed_model) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_packed_model) {
    BPU_MODEL_S *model_handle = new BPU_MODEL_S();
    int ret = HB_BPU_loadModelFromFile(model_file_name, model_handle);
    if (ret == 0) {
      is_packed_model_file_handle_[model_handle] = is_packed_model;
      return model_handle;
    } else {
      LOGF << "Load Model Failed, model_path: " << model_file_name;
      return nullptr;
    }
  } else {
    BPU_MODEL_PACKAGE_S *packed_model_handle = new BPU_MODEL_PACKAGE_S();
    int ret = HB_BPU_loadModelPackageFromFile(
        model_file_name, packed_model_handle);
    if (ret == 0) {
      is_packed_model_file_handle_[packed_model_handle] = is_packed_model;
      return packed_model_handle;
    } else {
      LOGF << "Load Model Failed";
      return nullptr;
    }
  }
}

// 判断model_file_handle是packed，则根据model_name返回对应的model_handle
// 非packed,直接返回；packed, 根据model_name取model_handle
void *BPUInferenceEngine::LoadModelHandle(
    void *model_file_handle, const char *model_name) {
  if (is_packed_model_file_handle_.end() ==
      is_packed_model_file_handle_.find(model_file_handle)) {
    LOGE << "not found model_file_handle";
    return nullptr;
  }
  if (is_packed_model_file_handle_[model_file_handle]) {
    BPU_MODEL_PACKAGE_S *packed_model_handle =
        static_cast<BPU_MODEL_PACKAGE_S*>(model_file_handle);
    for (int i = 0; i < packed_model_handle->model_num; i++) {
      if (packed_model_handle->model_name_list[i] == model_name) {
        return &packed_model_handle->model_list[i];
      }
    }
    LOGE << "not found model: " << model_name;
    return nullptr;
  } else {
    return model_file_handle;
  }
}

// 直接返回
int BPUInferenceEngine::ReleaseModelHandle(void *model_handle) {
  return 0;
}

// 判断是否packed，调用对应的BPU接口
int BPUInferenceEngine::ReleaseModelFileHandle(void *model_file_handle) {
  if (is_packed_model_file_handle_.end() ==
      is_packed_model_file_handle_.find(model_file_handle)) {
    LOGE << "not found model_file_handle";
    return -1;
  }
  if (is_packed_model_file_handle_[model_file_handle]) {
    HB_BPU_releaseModelPackage(
        static_cast<BPU_MODEL_PACKAGE_S *>(model_file_handle));
    delete model_file_handle;
  } else {
    HB_BPU_releaseModel(static_cast<BPU_MODEL_S *>(model_file_handle));
    delete model_file_handle;
  }
  is_packed_model_file_handle_.erase(model_file_handle);
  return 0;
}

int BPUInferenceEngine::GetModelInputInfo(
    void *model_handle, std::vector<TensorProperties> &input_modelinfo) {
  if (model_handle == nullptr) {
    return -1;
  }
  // 转换BPU
  BPU_MODEL_S *bpu_model_handle = static_cast<BPU_MODEL_S *>(model_handle);
  int input_num = bpu_model_handle->input_num;
  input_modelinfo.resize(input_num);
  for (int layer_idx = 0; layer_idx < input_num; layer_idx++) {
    BPU_MODEL_NODE_S input = bpu_model_handle->inputs[layer_idx];
    TensorProperties &model_info = input_modelinfo[layer_idx];
    ConvertModelInfo(input, model_info);
  }
  return 0;
}

int BPUInferenceEngine::GetModelOutputInfo(
    void *model_handle, std::vector<TensorProperties> &output_modelinfo) {
  if (model_handle == nullptr) {
    return -1;
  }
  // 转换BPU
  BPU_MODEL_S *bpu_model_handle = static_cast<BPU_MODEL_S *>(model_handle);
  int output_num = bpu_model_handle->output_num;
  output_modelinfo.resize(output_num);
  for (int layer_idx = 0; layer_idx < output_num; layer_idx++) {
    BPU_MODEL_NODE_S output = bpu_model_handle->outputs[layer_idx];
    TensorProperties &model_info = output_modelinfo[layer_idx];
    ConvertModelInfo(output, model_info);
  }
  return 0;
}

int BPUInferenceEngine::AllocModelTensor(
    std::vector<Tensor> &tensors, bool need_alloc, int batch) {
  int layer_num = tensors.size();
  for (int layer_idx = 0; layer_idx < layer_num; layer_idx++) {
    Tensor &tensor = tensors[layer_idx];
    TensorProperties info = tensor.properties;
    // sys_mem
    int h_idx, w_idx, c_idx;
    GetHWCIndex(info.tensor_type, info.tensor_layout,
                &h_idx, &w_idx, &c_idx);
    int height = info.valid_shape.dimension_size[h_idx];
    int stride = info.aligned_shape.dimension_size[w_idx];

    switch (info.tensor_type) {
    case IMG_TYPE_NV12: {
      // shape.d[h_idx] aligned_shape.d[w_idx]
      int y_length = height * stride;
      int uv_length = height / 2 * stride;
      if (need_alloc) {
        BPU_MEMORY_S mem;
        HB_SYS_bpuMemAlloc("in_data0", y_length + uv_length, true,
                           &mem);
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
        BPU_MEMORY_S mem_0, mem_1;
        HB_SYS_bpuMemAlloc("in_data0", y_length, true, &mem_0);
        HB_SYS_bpuMemAlloc("in_data1", uv_length, true, &mem_1);
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
      if (need_alloc) {
        BPU_MEMORY_S mem;
        HB_SYS_bpuMemAlloc("in_data0", length, true, &mem);
        tensor.sys_mem[0].phy_addr = mem.phyAddr;
        tensor.sys_mem[0].vir_addr = mem.virAddr;
        tensor.sys_mem[0].mem_size = mem.memSize;
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

int BPUInferenceEngine::FreeTensor(std::vector<Tensor> &tensors) {
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
      BPU_MEMORY_S mem = {tensor.sys_mem[0].phy_addr,
                          tensor.sys_mem[0].vir_addr,
                          tensor.sys_mem[0].mem_size};
      HB_SYS_bpuMemFree(&mem);
      break;
    }
    case IMG_TYPE_NV12_SEPARATE: {
      BPU_MEMORY_S mem_0 = {tensor.sys_mem[0].phy_addr,
                            tensor.sys_mem[0].vir_addr,
                            tensor.sys_mem[0].mem_size};
      BPU_MEMORY_S mem_1 = {tensor.sys_mem[1].phy_addr,
                            tensor.sys_mem[1].vir_addr,
                            tensor.sys_mem[1].mem_size};
      HB_SYS_bpuMemFree(&mem_0);
      HB_SYS_bpuMemFree(&mem_1);
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

int BPUInferenceEngine::ConvertTensor2BPUTensor(
    const Tensor &tensor, BPU_TENSOR_S &bpu_tensor) {
  bpu_tensor.data_type = datatype2bpu_table_[tensor.properties.tensor_type];
  bpu_tensor.data_shape.layout =
      layout2bpu_table_[tensor.properties.tensor_layout];
  bpu_tensor.data_shape.ndim = tensor.properties.valid_shape.num_dimensions;
  for (int dim = 0; dim < bpu_tensor.data_shape.ndim; dim++) {
    bpu_tensor.data_shape.d[dim] =
        tensor.properties.valid_shape.dimension_size[dim];
  }

  bpu_tensor.aligned_shape.layout =
      layout2bpu_table_[tensor.properties.tensor_layout];
  bpu_tensor.aligned_shape.ndim =
      tensor.properties.aligned_shape.num_dimensions;
  for (int dim = 0; dim < bpu_tensor.aligned_shape.ndim; dim++) {
    bpu_tensor.aligned_shape.d[dim] =
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
    bpu_tensor.data.phyAddr = tensor.sys_mem[0].phy_addr;
    bpu_tensor.data.virAddr = tensor.sys_mem[0].vir_addr;
    bpu_tensor.data.memSize = tensor.sys_mem[0].mem_size;
    break;
  case IMG_TYPE_NV12_SEPARATE:
    bpu_tensor.data.phyAddr = tensor.sys_mem[0].phy_addr;
    bpu_tensor.data.virAddr = tensor.sys_mem[0].vir_addr;
    bpu_tensor.data.memSize = tensor.sys_mem[0].mem_size;
    bpu_tensor.data_ext.phyAddr = tensor.sys_mem[1].phy_addr;
    bpu_tensor.data_ext.virAddr = tensor.sys_mem[1].vir_addr;
    bpu_tensor.data_ext.memSize = tensor.sys_mem[1].mem_size;
    break;
  default:
    LOGE << "not support tensor_type: " << tensor.properties.tensor_type;
    break;
  }
  return 0;
}

int BPUInferenceEngine::InferenceTaskCore(
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
    std::vector<BPU_TENSOR_S> bpu_input_tensors(input_layer);
    for (int i = 0; i < input_layer; i++) {
      Tensor &tensor = tensor_task->input_tensors_[i];
      BPU_TENSOR_S &bpu_tensor = bpu_input_tensors[i];
      ConvertTensor2BPUTensor(tensor, bpu_tensor);
      // flush tensor
      switch (tensor.properties.tensor_type) {
      case IMG_TYPE_NV12:
      case TENSOR_TYPE_S8:
        HB_SYS_flushMemCache(&bpu_tensor.data, HB_SYS_MEM_CACHE_CLEAN);
        break;
      case IMG_TYPE_NV12_SEPARATE:
        HB_SYS_flushMemCache(&bpu_tensor.data, HB_SYS_MEM_CACHE_CLEAN);
        HB_SYS_flushMemCache(&bpu_tensor.data_ext, HB_SYS_MEM_CACHE_CLEAN);
        break;
      default:
        LOGE << "not support tensor_type: " << tensor.properties.tensor_type;
        break;
      }
    }

    std::vector<BPU_TENSOR_S> bpu_output_tensors(output_layer);
    for (int i = 0; i < output_layer; i++) {
      Tensor &tensor = tensor_task->output_tensors_[i];
      BPU_TENSOR_S &bpu_tensor = bpu_output_tensors[i];
      ConvertTensor2BPUTensor(tensor, bpu_tensor);
    }
    BPU_TASK_HANDLE task_handle;
    BPU_RUN_CTRL_S run_ctrl{tensor_task->run_ctrls_.bpu_core_id, 1};
    ret = HB_BPU_runModel(
        static_cast<BPU_MODEL_S *>(tensor_task->model_handle_),
        bpu_input_tensors.data(),
        input_layer,
        bpu_output_tensors.data(),
        output_layer,
        &run_ctrl,
        true,
        &task_handle);
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
    int pyramid_layers = roi_task->input_tensors_[0].size();
    std::vector<BPU_ADDR_INFO_S> down_scales(pyramid_layers);
    for (int layer = 0; layer < pyramid_layers; layer++) {
      Tensor &input_tensor = roi_task->input_tensors_[0][layer];
      BPU_ADDR_INFO_S &down_scale = down_scales[layer];
      // 金字塔nv12格式
      down_scale.width =
          input_tensor.properties.valid_shape.dimension_size[3];
      down_scale.height =
          input_tensor.properties.valid_shape.dimension_size[2];
      down_scale.step =
          input_tensor.properties.aligned_shape.dimension_size[3];
      down_scale.y_paddr = input_tensor.sys_mem[0].phy_addr;
      down_scale.c_paddr = input_tensor.sys_mem[1].phy_addr;
      down_scale.y_vaddr =
          reinterpret_cast<uint64_t>(input_tensor.sys_mem[0].vir_addr);
      down_scale.c_vaddr =
          reinterpret_cast<uint64_t>(input_tensor.sys_mem[1].vir_addr);
    }
    // output_tensors_: Tensor转换BPU_TENSOR_S
    std::vector<BPU_TENSOR_S> bpu_output_tensors(
        roi_task->output_tensors_.size());
    for (int out_layer = 0;
         out_layer < roi_task->output_tensors_.size();
         out_layer++) {
      ConvertTensor2BPUTensor(roi_task->output_tensors_[out_layer],
                              bpu_output_tensors[out_layer]);
    }

    // InferBox封装有效的BPU_BBOX送入模型预测
    std::vector<BPU_BBOX> bpu_boxes;
    for (auto &infer_box : roi_task->roi_box_) {
      if (infer_box.resizable) {  // valid box
        bpu_boxes.push_back(BPU_BBOX{infer_box.x1, infer_box.y1,
                                     infer_box.x2, infer_box.y2,
                                     infer_box.score, 0, true});
      }
    }

    BPU_RUN_CTRL_S run_ctrl = {roi_task->run_ctrls_.bpu_core_id, 1};
    int resizable_cnt = 0;
    BPU_TASK_HANDLE task_handle;
    ret = HB_BPU_runModelWithBbox(
        static_cast<BPU_MODEL_S *>(roi_task->model_handle_),
        down_scales.data(),
        pyramid_layers,
        bpu_boxes.data(),
        bpu_boxes.size(),
        bpu_output_tensors.data(),
        bpu_output_tensors.size(),
        &run_ctrl,
        true,
        &resizable_cnt,
        &task_handle);
    LOGD << "resizable_cnt: " << resizable_cnt;
    if (resizable_cnt == 0 && ret != 0) {
      LOGE << "no box pass resizer, " << HB_BPU_getErrorName(ret);
    } else if (ret != 0) {
    LOGE << "RunModelWithBBox failed, " << HB_BPU_getErrorName(ret);
    } else {
      // ret == 0
      // 根据BPUBox更新InferBox
      int resizable_idx = 0;
      for (auto &infer_box : roi_task->roi_box_) {
        if (infer_box.resizable) {  // valid box
          infer_box.resizable = bpu_boxes[resizable_idx++].resizable;
        }
      }
      HOBOT_CHECK(resizable_idx == bpu_boxes.size());
    }
  }
  LOGD << "run model end, ret: " << ret;
  return ret;
}

int BPUInferenceEngine::ResultTaskCore(
    std::shared_ptr<InferenceEngineTask> task) {
  // 刷新output flush等后处理之前的操作
  int output_layer = task->output_tensors_.size();
  std::vector<BPU_TENSOR_S> bpu_output_tensors(output_layer);
  for (int i = 0; i < output_layer; i++) {
    Tensor &tensor = task->output_tensors_[i];
    BPU_TENSOR_S &bpu_tensor = bpu_output_tensors[i];
    ConvertTensor2BPUTensor(tensor, bpu_tensor);
  }

  int ret = 0;
  for (auto &bpu_tensor : bpu_output_tensors) {
    ret = HB_SYS_flushMemCache(&bpu_tensor.data, HB_SYS_MEM_CACHE_INVALIDATE);
    if (ret != 0)
      break;
  }
  return ret;
}

int BPUInferenceEngine::PrepareRoiTensor(
    const Inferencer *const infer, const std::vector<Tensor> &pyramid_tensors,
    std::vector<InferBBox> &rois, std::vector<Tensor> &tensors) {
  tensors = pyramid_tensors;
  return 0;
}

int BPUInferenceEngine::ResizeImage(const Tensor &nv12_tensor,
                                    Tensor &input_tensor, const InferBBox &rois,
                                    const ResizeCtrlParam resize_param) {
  BPU_TENSOR_S bpu_nv12_tensor;
  ConvertTensor2BPUTensor(nv12_tensor, bpu_nv12_tensor);
  BPU_TENSOR_S bpu_input_output_tensor;
  ConvertTensor2BPUTensor(input_tensor, bpu_input_output_tensor);
  BPU_ROI_S input_roi = {static_cast<int>(rois.x1), static_cast<int>(rois.y1),
                         static_cast<int>(rois.x2), static_cast<int>(rois.y2)};
  BPU_RESIZE_CTRL_S ctrl_param = {
      static_cast<BPU_RESIZE_TYPE_E>(resize_param.resize_type),
      static_cast<BPU_DATA_TYPE_E>(resize_param.output_type),
      resize_param.bpu_core_id};
  int ret = HB_BPU_cropAndResize(&bpu_nv12_tensor, &input_roi,
                                 &bpu_input_output_tensor, &ctrl_param);
  if (ret != 0) {
    LOGE << "BPU cropAndResize failed";
    return -1;
  }
  return 0;
}

}  // namespace inference
