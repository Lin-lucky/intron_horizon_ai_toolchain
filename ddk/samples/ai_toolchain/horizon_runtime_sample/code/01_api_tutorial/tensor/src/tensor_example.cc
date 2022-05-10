// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string.h>

#include <cstdlib>
#include <ctime>
#include <iostream>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

/**
 * Align by 16
 */
#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

/**
 * Prepare tensor and fill with image data
 * @param[in] image_height
 * @param[in] image_width
 * @param[in] image_data_type
 * @param[in] layout
 * @param[out] tensor
 */
void prepare_tensor(int image_height,
                    int image_width,
                    hbDNNDataType image_data_type,
                    hbDNNTensorLayout layout,
                    hbDNNTensor *tensor);

/**
 * Prepare tensor and fill with image data
 * @param[in] image_data
 * @param[out] tensor
 */
void prepare_image_data(void *image_data, hbDNNTensor *tensor);

/**
 * nhwc to nchw
 * @param[out] out_data0
 * @param[out] out_data1
 * @param[out] out_data2
 * @param[in] in_data
 * @param[in] height
 * @param[in] width
 */
void nhwc_to_nchw(uint8_t *out_data0,
                  uint8_t *out_data1,
                  uint8_t *out_data2,
                  uint8_t *in_data,
                  int height,
                  int width);

int get_hwc_index(const int32_t layout, int *h_idx, int *w_idx, int *c_idx);

/**
 * Free image tensor
 * @param tensor: Tensor to be released
 */
void release_tensor(hbDNNTensor *tensor);

int main(int argc, char **argv) {
  // Set random seed
  std::srand(std::time(nullptr));

  // Random function
  auto randint_fn = [](int min, int max) {
    while (true) {
      auto random_value = std::rand();
      if (random_value < min) {
        continue;
      }
      return random_value % (max + 1);
    }
  };

  uint8_t image_data[1920 * 1080 * 3];

  // Test prepare tensor & free tensor function
  auto test_fn = [&randint_fn, &image_data](hbDNNDataType image_data_type,
                                            hbDNNTensorLayout layout) {
    int image_height = randint_fn(16, 1080);
    int image_width = randint_fn(16, 1920);
    hbDNNTensor tensor;
    prepare_tensor(image_height, image_width, image_data_type, layout, &tensor);
    prepare_image_data(image_data, &tensor);
    std::cout << "Tensor data type:" << tensor.properties.tensorType
              << ", Tensor layout: " << tensor.properties.tensorLayout
              << ", shape:" << tensor.properties.validShape.dimensionSize[0]
              << "x" << tensor.properties.validShape.dimensionSize[1] << "x"
              << tensor.properties.validShape.dimensionSize[2] << "x"
              << tensor.properties.validShape.dimensionSize[3]
              << ", aligned shape:"
              << tensor.properties.alignedShape.dimensionSize[0] << "x"
              << tensor.properties.alignedShape.dimensionSize[1] << "x"
              << tensor.properties.alignedShape.dimensionSize[2] << "x"
              << tensor.properties.alignedShape.dimensionSize[3] << std::endl;
    release_tensor(&tensor);
  };

  // Y
  test_fn(HB_DNN_IMG_TYPE_Y, HB_DNN_LAYOUT_NCHW);
  // YUV_NV12
  test_fn(HB_DNN_IMG_TYPE_NV12, HB_DNN_LAYOUT_NCHW);
  // YUV_NV12_SEPARATE
  test_fn(HB_DNN_IMG_TYPE_NV12_SEPARATE, HB_DNN_LAYOUT_NCHW);
  // BGR
  test_fn(HB_DNN_IMG_TYPE_BGR, HB_DNN_LAYOUT_NCHW);
  test_fn(HB_DNN_IMG_TYPE_BGR, HB_DNN_LAYOUT_NHWC);
  // RGB
  test_fn(HB_DNN_IMG_TYPE_RGB, HB_DNN_LAYOUT_NCHW);
  test_fn(HB_DNN_IMG_TYPE_RGB, HB_DNN_LAYOUT_NHWC);
  // YUV444
  test_fn(HB_DNN_IMG_TYPE_YUV444, HB_DNN_LAYOUT_NCHW);
  test_fn(HB_DNN_IMG_TYPE_YUV444, HB_DNN_LAYOUT_NHWC);
}

void prepare_tensor(int height,
                    int width,
                    hbDNNDataType image_data_type,
                    hbDNNTensorLayout layout,
                    hbDNNTensor *tensor) {
  auto &tensor_property = tensor->properties;
  tensor_property.tensorType = image_data_type;
  tensor_property.tensorLayout = layout;
  int h_idx, w_idx, c_idx;
  get_hwc_index(layout, &h_idx, &w_idx, &c_idx);
  tensor_property.validShape.numDimensions = 4;
  tensor_property.validShape.dimensionSize[0] = 1;
  tensor_property.validShape.dimensionSize[h_idx] = height;
  tensor_property.validShape.dimensionSize[w_idx] = width;
  tensor_property.validShape.dimensionSize[c_idx] = 3;
  tensor_property.alignedShape = tensor_property.validShape;
  if (image_data_type == HB_DNN_IMG_TYPE_Y) {
    tensor_property.validShape.dimensionSize[c_idx] = 1;
    // Align by 16 bytes
    int stride = ALIGN_16(width);
    tensor_property.alignedShape.dimensionSize[w_idx] = stride;
    tensor_property.alignedShape.dimensionSize[c_idx] = 1;
    hbSysAllocCachedMem(&tensor->sysMem[0], height * stride);
  } else if (image_data_type == HB_DNN_IMG_TYPE_YUV444 ||
             image_data_type == HB_DNN_IMG_TYPE_BGR ||
             image_data_type == HB_DNN_IMG_TYPE_RGB) {
    hbSysAllocCachedMem(&tensor->sysMem[0], height * width * 3);
  } else if (image_data_type == HB_DNN_IMG_TYPE_NV12) {
    // Align by 16 bytes
    int stride = ALIGN_16(width);
    int y_length = height * stride;
    int uv_length = height / 2 * stride;
    tensor_property.alignedShape.dimensionSize[w_idx] = stride;
    hbSysAllocCachedMem(&tensor->sysMem[0], y_length + uv_length);
  } else if (image_data_type == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
    // Align by 16 bytes
    int stride = ALIGN_16(width);
    int y_length = height * stride;
    int uv_length = height / 2 * stride;
    tensor_property.alignedShape.dimensionSize[w_idx] = stride;
    hbSysAllocCachedMem(&tensor->sysMem[0], y_length);
    hbSysAllocCachedMem(&tensor->sysMem[1], uv_length);
  } else if (image_data_type == HB_DNN_TENSOR_TYPE_F32 ||
             image_data_type == HB_DNN_TENSOR_TYPE_S32 ||
             image_data_type == HB_DNN_TENSOR_TYPE_U32) {
    hbSysAllocCachedMem(&tensor->sysMem[0], height * width * 4);
  } else if (image_data_type == HB_DNN_TENSOR_TYPE_U8 ||
             image_data_type == HB_DNN_TENSOR_TYPE_S8) {
    hbSysAllocCachedMem(&tensor->sysMem[0], height * width);
  }
}

void prepare_image_data(void *image_data, hbDNNTensor *tensor) {
  auto &tensor_property = tensor->properties;
  int32_t image_data_type = tensor_property.tensorType;
  int32_t layout = tensor_property.tensorLayout;
  int h_idx, w_idx, c_idx;
  get_hwc_index(layout, &h_idx, &w_idx, &c_idx);
  int image_height = tensor_property.validShape.dimensionSize[h_idx];
  int image_width = tensor_property.validShape.dimensionSize[w_idx];
  int stride = tensor_property.alignedShape.dimensionSize[w_idx];
  if (image_data_type == HB_DNN_IMG_TYPE_Y) {
    uint8_t *data = reinterpret_cast<uint8_t *>(image_data);
    uint8_t *data0 = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
    for (int h = 0; h < image_height; ++h) {
      auto *raw = data0 + h * stride;
      for (int w = 0; w < image_width; ++w) {
        *raw++ = *data++;
      }
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  } else if (image_data_type == HB_DNN_IMG_TYPE_YUV444 ||
             image_data_type == HB_DNN_IMG_TYPE_BGR ||
             image_data_type == HB_DNN_IMG_TYPE_RGB) {
    if (layout == HB_DNN_LAYOUT_NHWC) {
      uint8_t *data = reinterpret_cast<uint8_t *>(image_data);
      int image_length = image_height * stride * 3;
      void *data0 = tensor->sysMem[0].virAddr;
      memcpy(data0, data, image_length);
    } else {
      int channel_size = image_height * image_width;
      uint8_t *mem = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
      nhwc_to_nchw(mem,
                   mem + channel_size,
                   mem + channel_size * 2,
                   reinterpret_cast<uint8_t *>(image_data),
                   image_height,
                   image_width);
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  } else if (image_data_type == HB_DNN_IMG_TYPE_NV12) {
    uint8_t *data = reinterpret_cast<uint8_t *>(image_data);
    // Align by 16 bytes
    int y_length = image_height * stride;
    int uv_length = image_height / 2 * stride;

    // Copy y data to data
    uint8_t *y = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
    for (int h = 0; h < image_height; ++h) {
      auto *raw = y + h * stride;
      for (int w = 0; w < image_width; ++w) {
        *raw++ = *data++;
      }
    }
    // Copy uv data
    uint8_t *uv =
        reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr) + y_length;
    int uv_height = image_height / 2;
    for (int i = 0; i < uv_height; ++i) {
      auto *raw = uv + i * stride;
      for (int j = 0; j < image_width; ++j) {
        *raw++ = *data++;
      }
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
  } else if (image_data_type == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
    uint8_t *data = reinterpret_cast<uint8_t *>(image_data);
    int y_length = image_height * stride;
    int uv_length = image_height / 2 * stride;

    // Copy y data to data0
    uint8_t *y = reinterpret_cast<uint8_t *>(tensor->sysMem[0].virAddr);
    for (int h = 0; h < image_height; ++h) {
      auto *raw = y + h * stride;
      for (int w = 0; w < image_width; ++w) {
        *raw++ = *data++;
      }
    }
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
    // Copy uv data to data_ext
    uint8_t *uv = reinterpret_cast<uint8_t *>(tensor->sysMem[1].virAddr);
    int uv_height = image_height / 2;
    for (int i = 0; i < uv_height; ++i) {
      auto *raw = uv + i * stride;
      for (int j = 0; j < image_width; ++j) {
        *raw++ = *data++;
      }
    }
    hbSysFlushMem(&(tensor->sysMem[1]), HB_SYS_MEM_CACHE_CLEAN);
  }
  return;
}

void nhwc_to_nchw(uint8_t *out_data0,
                  uint8_t *out_data1,
                  uint8_t *out_data2,
                  uint8_t *in_data,
                  int height,
                  int width) {
  for (int hh = 0; hh < height; ++hh) {
    for (int ww = 0; ww < width; ++ww) {
      *out_data0++ = *(in_data++);
      *out_data1++ = *(in_data++);
      *out_data2++ = *(in_data++);
    }
  }
}

void release_tensor(hbDNNTensor *tensor) {
  switch (tensor->properties.tensorType) {
    case HB_DNN_IMG_TYPE_Y:
    case HB_DNN_IMG_TYPE_RGB:
    case HB_DNN_IMG_TYPE_BGR:
    case HB_DNN_IMG_TYPE_YUV444:
    case HB_DNN_IMG_TYPE_NV12:
      hbSysFreeMem(&(tensor->sysMem[0]));
      break;
    case HB_DNN_IMG_TYPE_NV12_SEPARATE:
      hbSysFreeMem(&(tensor->sysMem[0]));
      hbSysFreeMem(&(tensor->sysMem[1]));
      break;
    default:
      break;
  }
}

int get_hwc_index(int32_t layout, int *h_idx, int *w_idx, int *c_idx) {
  if (layout == HB_DNN_LAYOUT_NHWC) {
    *h_idx = 1;
    *w_idx = 2;
    *c_idx = 3;
  } else if (layout == HB_DNN_LAYOUT_NCHW) {
    *c_idx = 1;
    *h_idx = 2;
    *w_idx = 3;
  }
  return 0;
}
