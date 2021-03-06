/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     order vector base data type
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.19
 */

#ifndef TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_ARRAY_H_
#define TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_ARRAY_H_

#include <cstdint>
#include <cstdlib>
#include <vector>

#include "hobotxstream/xstream_data.h"

namespace xstream {

/**
 * @brief 一位数组
 *
 */
struct Array : public xstream::BaseData {
  /**
   * @brief Construct a new Array object
   *
   * @param values_size 数据长度
   * @param values 如果不为null则拷贝该内存
   */
  explicit Array(size_t values_size = 0, float* values = nullptr);
  ~Array();

  uint32_t class_ = 0;         //< 分类信息
  float score_ = 0.0;          //< 置信度
  std::vector<float> values_;  //< 数据指针
};

}  // namespace xstream

#endif  // TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_ARRAY_H_
