/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @File: CNNMethodData.h
 * @Brief: declaration of the CNNMethodData
 * @Author: zhengzheng.ge
 * @Email: zhengzheng.ge@horizon.ai
 * @Date: 2019-04-15 14:18:28
 * @Last Modified by: zhengzheng.ge
 * @Last Modified time: 2019-04-15 16:16:58
 */

#ifndef INCLUDE_CNNMETHOD_UTIL_CNNMETHODDATA_H_
#define INCLUDE_CNNMETHOD_UTIL_CNNMETHODDATA_H_

#include <vector>
#include "xstream/xstream_data.h"
#include "xstream/vision_type.h"

namespace xstream {

struct CNNMethodRunData {
  const std::vector<BaseDataPtr> *input;
  const xstream::InputParamPtr *param;

  std::vector<std::vector<uint32_t>> real_nhwc;

  std::vector<BaseDataPtr> norm_rois;
  std::vector<std::vector<uint32_t>> all_shift;

  int input_dim_size;
  std::vector<uint32_t> elem_size;

  // mxnet_output_[i] : object j
  // mxnet_output_[i][j] : object j, layer k
  std::vector<std::vector<std::vector<int8_t>>> mxnet_output;

  std::vector<BaseDataPtr> output;

  // other user data
  void *context;
};
}  // namespace xstream
#endif  // INCLUDE_CNNMETHOD_UTIL_CNNMETHODDATA_H_
