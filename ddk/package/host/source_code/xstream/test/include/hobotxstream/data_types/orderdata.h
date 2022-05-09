/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     order vector base data type
 * @author    shuhuan.sun
 * @email     shuhuan.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.19
 */
#ifndef TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_ORDERDATA_H_
#define TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_ORDERDATA_H_

#include <cstdint>
#include <cstdlib>
#include <vector>

#include "xstream/xstream_data.h"

namespace xstream {
struct OrderData : public xstream::BaseData {
  explicit OrderData(size_t sequece_id);
  ~OrderData();

  size_t sequence_id = 0;
};
}  // namespace xstream

#endif  // TEST_INCLUDE_HOBOTXSTREAM_DATA_TYPES_ORDERDATA_H_
