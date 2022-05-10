/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @File: dnn_util.h
 * @Brief: declaration of the dnn_util
 * @Author: zhe.sun
 * @Email: zhe.sun@horizon.ai
 * @Date: 2020-12-21 18:09:18
 * @Last Modified by: zhe.sun
 * @Last Modified time: 2020-12-21 21:01:33
 */

#ifndef DNNUTIL_INCLUDE_DNNUTIL_H_
#define DNNUTIL_INCLUDE_DNNUTIL_H_

#include <map>
#include <string>
#include "bpu_predict_extension.h"
#include "hobotlog/hobotlog.hpp"
#include "xstream/vision_type.h"

namespace xstream {

enum class NormalizeMode {
  BPU_MODEL_NORM_BY_WIDTH_LENGTH,
  BPU_MODEL_NORM_BY_WIDTH_RATIO,
  BPU_MODEL_NORM_BY_HEIGHT_RATIO,
  BPU_MODEL_NORM_BY_LSIDE_RATIO,
  BPU_MODEL_NORM_BY_HEIGHT_LENGTH,
  BPU_MODEL_NORM_BY_LSIDE_LENGTH,
  BPU_MODEL_NORM_BY_LSIDE_SQUARE,
  BPU_MODEL_NORM_BY_DIAGONAL_SQUARE,
  BPU_MODEL_NORM_BY_WIDTH_SQUARE,
  BPU_MODEL_NORM_BY_HEIGHT_SQUARE,
  BPU_MODEL_NORM_BY_NOTHING
};

enum class FilterMode { OUT_OF_RANGE, NO_FILTER };

struct NormParams {
  NormalizeMode norm_method;
  float expand_scale = 1.0f;
  float aspect_ratio = 1.0f;
};

const std::map<std::string, NormalizeMode> g_norm_method_map = {
    {"norm_by_width_length", NormalizeMode::BPU_MODEL_NORM_BY_WIDTH_LENGTH},
    {"norm_by_width_ratio", NormalizeMode::BPU_MODEL_NORM_BY_WIDTH_RATIO},
    {"norm_by_height_rario", NormalizeMode::BPU_MODEL_NORM_BY_HEIGHT_RATIO},
    {"norm_by_lside_ratio", NormalizeMode::BPU_MODEL_NORM_BY_LSIDE_RATIO},
    {"norm_by_height_length", NormalizeMode::BPU_MODEL_NORM_BY_HEIGHT_LENGTH},
    {"norm_by_lside_length", NormalizeMode::BPU_MODEL_NORM_BY_LSIDE_LENGTH},
    {"norm_by_lside_square", NormalizeMode::BPU_MODEL_NORM_BY_LSIDE_SQUARE},
    {"norm_by_diagonal_square",
     NormalizeMode::BPU_MODEL_NORM_BY_DIAGONAL_SQUARE},
    {"norm_by_width_square", NormalizeMode::BPU_MODEL_NORM_BY_WIDTH_SQUARE},
    {"norm_by_height_square", NormalizeMode::BPU_MODEL_NORM_BY_HEIGHT_SQUARE},
    {"norm_by_nothing", NormalizeMode::BPU_MODEL_NORM_BY_NOTHING}};

const std::map<std::string, FilterMode> g_filter_method_map = {
    {"out_of_range", FilterMode::OUT_OF_RANGE},
    {"no_filter", FilterMode::NO_FILTER}};

const std::map<std::string, BPU_DATA_TYPE_E> g_data_type_map = {
    {"img_y", BPU_DATA_TYPE_E::BPU_TYPE_IMG_Y},
    {"img_yuv_nv12", BPU_DATA_TYPE_E::BPU_TYPE_IMG_YUV_NV12},
    {"img_yuv444", BPU_DATA_TYPE_E::BPU_TYPE_IMG_YUV444},
    {"img_bgr", BPU_DATA_TYPE_E::BPU_TYPE_IMG_BGR},
    {"img_bgrp", BPU_DATA_TYPE_E::BPU_TYPE_IMG_BGRP},
    {"img_rgb", BPU_DATA_TYPE_E::BPU_TYPE_IMG_RGB},
    {"img_rgbp", BPU_DATA_TYPE_E::BPU_TYPE_IMG_RGBP},
    {"img_nv12_separate", BPU_DATA_TYPE_E::BPU_TYPE_IMG_NV12_SEPARATE},
    {"tensor_u8", BPU_DATA_TYPE_E::BPU_TYPE_TENSOR_U8},
    {"tensor_s8", BPU_DATA_TYPE_E::BPU_TYPE_TENSOR_S8},
    {"tensor_f32", BPU_DATA_TYPE_E::BPU_TYPE_TENSOR_F32},
    {"tensor_s32", BPU_DATA_TYPE_E::BPU_TYPE_TENSOR_S32},
    {"tensor_u32", BPU_DATA_TYPE_E::BPU_TYPE_TENSOR_U32},
    {"max", BPU_DATA_TYPE_E::BPU_TYPE_MAX}};

inline int FilterRoi(BBox *src, BBox *dst, int src_w, int src_h,
                     FilterMode filter_method) {
  switch (filter_method) {
    case FilterMode::OUT_OF_RANGE: {
      if (dst->x1_ < 0 || dst->y1_ < 0 || dst->x2_ > src_w || dst->y2_ > src_h)
        return -1;
    }
    case FilterMode::NO_FILTER: {
      return 0;
    }
  }
  return 0;
}

inline int NormalizeRoi(BBox *src, BBox *dst, NormParams norm_params,
                        uint32_t total_w, uint32_t total_h,
                        FilterMode filter_method) {
  *dst = *src;
  float box_w = dst->x2_ - dst->x1_;
  float box_h = dst->y2_ - dst->y1_;
  float center_x = (dst->x1_ + dst->x2_) / 2.0f;
  float center_y = (dst->y1_ + dst->y2_) / 2.0f;
  float w_new = box_w;
  float h_new = box_h;
  NormalizeMode norm_method = norm_params.norm_method;
  float norm_ratio = norm_params.expand_scale;
  float aspect_ratio = norm_params.aspect_ratio;

  switch (norm_method) {
    case NormalizeMode::BPU_MODEL_NORM_BY_WIDTH_LENGTH: {
      w_new = box_w * norm_ratio;
      h_new = box_h + w_new - box_w;
      if (h_new <= 0) return -1;
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_WIDTH_RATIO:
    case NormalizeMode::BPU_MODEL_NORM_BY_HEIGHT_RATIO:
    case NormalizeMode::BPU_MODEL_NORM_BY_LSIDE_RATIO: {
      h_new = box_h * norm_ratio;
      w_new = box_w * norm_ratio;
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_HEIGHT_LENGTH: {
      h_new = box_h * norm_ratio;
      w_new = box_w + h_new - box_h;
      if (w_new <= 0) return -1;
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_LSIDE_LENGTH: {
      if (box_w > box_h) {
        w_new = box_w * norm_ratio;
        h_new = box_h + w_new - box_w;
        if (h_new <= 0) return -1;
      } else {
        h_new = box_h * norm_ratio;
        w_new = box_w + h_new - box_h;
        if (w_new <= 0) return -1;
      }
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_LSIDE_SQUARE: {
      if (box_w > box_h / aspect_ratio) {
        w_new = box_w * norm_ratio;
        h_new = w_new / aspect_ratio;
      } else {
        h_new = box_h * norm_ratio;
        w_new = h_new * aspect_ratio;
      }
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_DIAGONAL_SQUARE: {
      float diagonal = sqrt(pow(box_w, 2.0) + pow(box_h, 2.0));
      w_new = h_new = diagonal * norm_ratio;
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_WIDTH_SQUARE: {
      w_new = box_w * norm_ratio;
      h_new = w_new;
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_HEIGHT_SQUARE: {
      h_new = box_h * norm_ratio;
      w_new = h_new;
    } break;
    case NormalizeMode::BPU_MODEL_NORM_BY_NOTHING:
      break;
    default:
      return 0;
  }
  dst->x1_ = center_x - w_new / 2;
  dst->x2_ = center_x + w_new / 2;
  dst->y1_ = center_y - h_new / 2;
  dst->y2_ = center_y + h_new / 2;

  if (FilterRoi(src, dst, total_w, total_h, filter_method)) {
    *dst = *src;
    return -1;
  }

  dst->x1_ = dst->x1_ < 0 ? 0.0f : dst->x1_;
  dst->y1_ = dst->y1_ < 0 ? 0.0f : dst->y1_;
  dst->x2_ = dst->x2_ > total_w ? total_w : dst->x2_;
  dst->y2_ = dst->y2_ > total_h ? total_h : dst->y2_;
  LOGD << "norm roi[x1, y1, x2, y2]: [" << dst->x1_ << ", " << dst->y1_ << ", "
       << dst->x2_ << ", " << dst->y2_ << "]";
  return 0;
}

inline float GetFloatByInt(int32_t value, uint32_t shift) {
  float ret_x = value;
  if (value != 0) {
    int *ix = reinterpret_cast<int *>(&ret_x);
    (*ix) -= shift * 0x00800000;
  }
  return ret_x;
}

// 定点转浮点
// IN: src_ptr, bpu_model, out_index; OUT: dest_ptr
// 返回0：转换成功；返回-1，BPU输出是浮点数据，不需要转换
// 输出结果dest_ptr, 需要在函数外部由用户申请空间
inline int ConvertOutputToFloat(void *src_ptr, void *dest_ptr,
                                const BPU_MODEL_S &bpu_model, int out_index) {
  if (bpu_model.outputs[out_index].data_type == BPU_TYPE_TENSOR_F32) {
    LOGW << "BPU Output data_type is float32, no need convert to float";
    return -1;
  }
  auto &aligned_shape = bpu_model.outputs[out_index].aligned_shape;
  auto &real_shape = bpu_model.outputs[out_index].shape;
  auto out_elem_size = 1, float_elem_size = 4;
  if (bpu_model.outputs[out_index].data_type == BPU_TYPE_TENSOR_S32 ||
      bpu_model.outputs[out_index].data_type == BPU_TYPE_TENSOR_U32) {
    out_elem_size = 4;
  }
  auto shift = bpu_model.outputs[out_index].shifts;

  uint32_t dst_n_stride =
      real_shape.d[1] * real_shape.d[2] * real_shape.d[3] * float_elem_size;
  uint32_t dst_h_stride = real_shape.d[2] * real_shape.d[3] * float_elem_size;
  uint32_t dst_w_stride = real_shape.d[3] * float_elem_size;
  uint32_t src_n_stride = aligned_shape.d[1] * aligned_shape.d[2] *
                          aligned_shape.d[3] * out_elem_size;
  uint32_t src_h_stride =
      aligned_shape.d[2] * aligned_shape.d[3] * out_elem_size;
  uint32_t src_w_stride = aligned_shape.d[3] * out_elem_size;

  float tmp_float_value;
  int32_t tmp_int32_value;

  for (int nn = 0; nn < real_shape.d[0]; nn++) {
    void *cur_n_dst = reinterpret_cast<int8_t *>(dest_ptr) + nn * dst_n_stride;
    void *cur_n_src = reinterpret_cast<int8_t *>(src_ptr) + nn * src_n_stride;
    for (int hh = 0; hh < real_shape.d[1]; hh++) {
      void *cur_h_dst =
          reinterpret_cast<int8_t *>(cur_n_dst) + hh * dst_h_stride;
      void *cur_h_src =
          reinterpret_cast<int8_t *>(cur_n_src) + hh * src_h_stride;
      for (int ww = 0; ww < real_shape.d[2]; ww++) {
        void *cur_w_dst =
            reinterpret_cast<int8_t *>(cur_h_dst) + ww * dst_w_stride;
        void *cur_w_src =
            reinterpret_cast<int8_t *>(cur_h_src) + ww * src_w_stride;
        for (int cc = 0; cc < real_shape.d[3]; cc++) {
          void *cur_c_dst =
              reinterpret_cast<int8_t *>(cur_w_dst) + cc * float_elem_size;
          void *cur_c_src =
              reinterpret_cast<int8_t *>(cur_w_src) + cc * out_elem_size;
          if (out_elem_size == 4) {
            tmp_int32_value = *(reinterpret_cast<int32_t *>(cur_c_src));
            tmp_float_value = GetFloatByInt(tmp_int32_value, shift[cc]);
            *(reinterpret_cast<float *>(cur_c_dst)) = tmp_float_value;
          } else {
            tmp_int32_value = *(reinterpret_cast<int8_t *>(cur_c_src));
            tmp_float_value = GetFloatByInt(tmp_int32_value, shift[cc]);
            *(reinterpret_cast<float *>(cur_c_dst)) = tmp_float_value;
          }
        }
      }
    }
  }
  return 0;
}

}  // namespace xstream
#endif  // DNNUTIL_INCLUDE_DNNUTIL_H_
