// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_HB_DNN_STATUS_H_
#define DNN_HB_DNN_STATUS_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

typedef enum {
  HB_DNN_SUCCESS = 0,
  HB_DNN_INVALID_ARGUMENT,
  HB_DNN_INVALID_MODEL,
  HB_DNN_MODEL_NUMBER_EXCEED_LIMIT,
  HB_DNN_INVALID_PACKED_DNN_HANDLE,
  HB_DNN_INVALID_DNN_HANDLE,
  HB_DNN_CAN_NOT_OPEN_FILE,
  HB_DNN_OUT_OF_MEMORY,
  HB_DNN_TIMEOUT,
  HB_DNN_TASK_NUM_EXCEED_LIMIT,
  HB_DNN_TASK_BATCH_SIZE_EXCEED_LIMIT,
  HB_DNN_INVALID_TASK_HANDLE,
  HB_DNN_RUN_TASK_FAILED,
  HB_DNN_MODEL_IS_RUNNING,
  HB_DNN_INCOMPATIBLE_MODEL,
  HB_DNN_API_USE_ERROR
} hbDNNStatus;

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // DNN_HB_DNN_STATUS_H_
