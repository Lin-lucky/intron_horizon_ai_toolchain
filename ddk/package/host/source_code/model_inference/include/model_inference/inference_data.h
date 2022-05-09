/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of inference_data
 * @file   inference_data.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef INFERENCE_DATA_H_
#define INFERENCE_DATA_H_

#include <memory>
#include <vector>

namespace inference {

typedef struct {
  uint64_t phy_addr;
  void *vir_addr;
  uint32_t mem_size;
} SysMem;

typedef struct {
  int32_t dimension_size[8];
  int32_t num_dimensions;
} TensorShape;

typedef enum {
  LAYOUT_NHWC = 0,
  LAYOUT_NCHW = 2,
  LAYOUT_NHWC_4W8C = 134,  // 适配老模型中的layout特殊处理
  LAYOUT_NONE = 255,
} TensorLayout;

typedef enum {
  IMG_TYPE_Y,
  IMG_TYPE_NV12,
  IMG_TYPE_NV12_SEPARATE,
  IMG_TYPE_YUV444,
  IMG_TYPE_RGB,
  IMG_TYPE_BGR,
  TENSOR_TYPE_S4,
  TENSOR_TYPE_U4,
  TENSOR_TYPE_S8,
  TENSOR_TYPE_U8,
  TENSOR_TYPE_F16,
  TENSOR_TYPE_S16,
  TENSOR_TYPE_U16,
  TENSOR_TYPE_F32,
  TENSOR_TYPE_S32,
  TENSOR_TYPE_U32,
  TENSOR_TYPE_F64,
  TENSOR_TYPE_S64,
  TENSOR_TYPE_U64,
  TENSOR_TYPE_MAX
} DataType;

typedef struct {
  int32_t shift_len;
  uint8_t *shift_data;
} QuantiShift;

typedef struct {
  int32_t scale_len;
  float *scale_data;
} QuantiScale;

typedef enum {
  NONE,  // no quantization
  SHIFT,
  SCALE,
} QuantiType;

typedef struct {
  TensorShape valid_shape;
  TensorShape aligned_shape;
  TensorLayout tensor_layout;
  DataType tensor_type;
  QuantiShift shift;
  QuantiScale scale;
  QuantiType quanti_type;
} TensorProperties;

typedef enum {
  MEM_TYPE_ALLOCATED,
  MEM_TYPE_ASSIGNMENT,
  MEM_TYPE_POOL
} SysMemType;

typedef struct {
  SysMem sys_mem[4];
  TensorProperties properties;
  SysMemType sys_mem_type = MEM_TYPE_ALLOCATED;
} Tensor;

typedef struct {
  int32_t bpu_core_id;  // BPU param
  int32_t dsp_core_id;
  int32_t priority;
  int32_t more;
  void *reserved;
} InferCtrlParam;  // 封装run_ctrl_param

// resize param
typedef enum {
  RESIZE_TYPE_BILINEAR = 0,
} ResizeType;

typedef enum {
  // IMG type is uint8
  RESIZE_TYPE_IMG_Y,
  RESIZE_TYPE_IMG_YUV_NV12,
  RESIZE_TYPE_IMG_YUV444,
  RESIZE_TYPE_IMG_BGR,
  RESIZE_TYPE_IMG_BGRP,
  RESIZE_TYPE_IMG_RGB,
  RESIZE_TYPE_IMG_RGBP,
  RESIZE_TYPE_IMG_NV12_SEPARATE,  // for separate yuv nv12
  RESIZE_TYPE_TENSOR_U8,          // for uint8 tensor
  RESIZE_TYPE_TENSOR_S8,          // for signed int8
  RESIZE_TYPE_TENSOR_F32,         // for float32
  RESIZE_TYPE_TENSOR_S32,         // for int32
  RESIZE_TYPE_TENSOR_U32,         // for uint32
  RESIZE_TYPE_TENSOR_S64,         // for int64
  RESIZE_TYPE_MAX,
} ResizeDataType;

typedef struct {
  ResizeType resize_type;
  ResizeDataType output_type;
  int32_t bpu_core_id;
  int32_t priority;
  int32_t *reserved1;
}ResizeCtrlParam;

// 浮点转换结果
typedef struct {
  TensorLayout layout;
  int dim[4];
  std::vector<float> value;  // batch * (nhwc), batch for resizer model
} FloatTensor;

typedef struct {
  float x1;
  float y1;
  float x2;
  float y2;
  float score;
  int type;
  bool resizable;
  float scaling;  // only valid for dnn
} InferBBox;

}  // namespace inference

#endif  // INFERENCE_DATA_H_
