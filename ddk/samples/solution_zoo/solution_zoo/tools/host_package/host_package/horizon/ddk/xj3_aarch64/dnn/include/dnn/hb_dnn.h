// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef DNN_HB_DNN_H_
#define DNN_HB_DNN_H_

#include "hb_sys.h"

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#define HB_DNN_VERSION_MAJOR 1U
#define HB_DNN_VERSION_MINOR 3U
#define HB_DNN_VERSION_PATCH 2U

#define HB_DNN_TENSOR_MAX_DIMENSIONS 8

#define HB_DNN_INITIALIZE_INFER_CTRL_PARAM(param) \
  {                                               \
    (param)->bpuCoreId = HB_BPU_CORE_ANY;         \
    (param)->dspCoreId = HB_DSP_CORE_ANY;         \
    (param)->priority = HB_DNN_PRIORITY_LOWEST;   \
    (param)->reserved1 = 0;                       \
    (param)->reserved2 = 0;                       \
    (param)->reserved3 = 0;                       \
    (param)->reserved4 = 0;                       \
    (param)->more = false;                        \
  }

#define HB_DNN_INITIALIZE_RESIZE_CTRL_PARAM(param)     \
  {                                                    \
    (param)->bpuCoreId = HB_BPU_CORE_ANY;              \
    (param)->resizeType = HB_DNN_RESIZE_TYPE_BILINEAR; \
    (param)->priority = HB_DNN_PRIORITY_LOWEST;        \
    (param)->reserved1 = 0;                            \
    (param)->reserved2 = 0;                            \
    (param)->reserved3 = 0;                            \
    (param)->reserved4 = 0;                            \
  }

typedef void *hbPackedDNNHandle_t;
typedef void *hbDNNHandle_t;
typedef void *hbDNNTaskHandle_t;

typedef enum {
  HB_DNN_LAYOUT_NHWC = 0,
  HB_DNN_LAYOUT_NCHW = 2,
  HB_DNN_LAYOUT_NONE = 255,
} hbDNNTensorLayout;

typedef enum {
  HB_DNN_IMG_TYPE_Y,
  HB_DNN_IMG_TYPE_NV12,
  HB_DNN_IMG_TYPE_NV12_SEPARATE,
  HB_DNN_IMG_TYPE_YUV444,
  HB_DNN_IMG_TYPE_RGB,
  HB_DNN_IMG_TYPE_BGR,
  HB_DNN_TENSOR_TYPE_S4,
  HB_DNN_TENSOR_TYPE_U4,
  HB_DNN_TENSOR_TYPE_S8,
  HB_DNN_TENSOR_TYPE_U8,
  HB_DNN_TENSOR_TYPE_F16,
  HB_DNN_TENSOR_TYPE_S16,
  HB_DNN_TENSOR_TYPE_U16,
  HB_DNN_TENSOR_TYPE_F32,
  HB_DNN_TENSOR_TYPE_S32,
  HB_DNN_TENSOR_TYPE_U32,
  HB_DNN_TENSOR_TYPE_F64,
  HB_DNN_TENSOR_TYPE_S64,
  HB_DNN_TENSOR_TYPE_U64,
  HB_DNN_TENSOR_TYPE_MAX
} hbDNNDataType;

typedef struct {
  int32_t dimensionSize[HB_DNN_TENSOR_MAX_DIMENSIONS];
  int32_t numDimensions;
} hbDNNTensorShape;

typedef struct {
  int32_t shiftLen;
  uint8_t *shiftData;
} hbDNNQuantiShift;

typedef struct {
  int32_t scaleLen;
  float *scaleData;
} hbDNNQuantiScale;

typedef enum {
  NONE,  // no quantization
  SHIFT,
  SCALE,
} hbDNNQuantiType;

typedef struct {
  hbDNNTensorShape validShape;
  hbDNNTensorShape alignedShape;
  int32_t tensorLayout;
  int32_t tensorType;
  hbDNNQuantiShift shift;
  hbDNNQuantiScale scale;
  hbDNNQuantiType quantiType;
} hbDNNTensorProperties;

typedef struct {
  hbSysMem sysMem[4];
  hbDNNTensorProperties properties;
} hbDNNTensor;

typedef struct {
  int32_t left;
  int32_t top;
  int32_t right;
  int32_t bottom;
} hbDNNRoi;

typedef enum {
  HB_DNN_PRIORITY_LOWEST = 0,
  HB_DNN_PRIORITY_HIGHEST = 255,
  HB_DNN_PRIORITY_PREEMP = HB_DNN_PRIORITY_HIGHEST,
} hbDNNTaskPriority;

typedef struct {
  int32_t bpuCoreId;
  int32_t dspCoreId;
  int32_t priority;
  int32_t more;
  int32_t reserved1;
  int32_t reserved2;
  int32_t reserved3;
  int32_t reserved4;
} hbDNNInferCtrlParam;

typedef enum {
  HB_DNN_RESIZE_TYPE_BILINEAR = 0,
} hbDNNResizeType;

typedef struct {
  int32_t bpuCoreId;
  int32_t priority;
  hbDNNResizeType resizeType;
  int32_t reserved1;
  int32_t reserved2;
  int32_t reserved3;
  int32_t reserved4;
} hbDNNResizeCtrlParam;

/**
 * Get DNN version
 * @return DNN version info
 */
const char *hbDNNGetVersion();

/**
 * Creates and initializes Horizon DNN Networks from file list
 * @param[out] packedDNNHandle
 * @param[in] modelFileNames
 * @param[in] modelFileCount
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNInitializeFromFiles(hbPackedDNNHandle_t *packedDNNHandle,
                                 const char **modelFileNames,
                                 int32_t modelFileCount);

/**
 * Creates and initializes Horizon DNN Networks from memory
 * @param[out] packedDNNHandle
 * @param[in] modelData
 * @param[in] modelDataLengths
 * @param[in] modelDataCount
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNInitializeFromDDR(hbPackedDNNHandle_t *packedDNNHandle,
                               const void **modelData,
                               int32_t *modelDataLengths,
                               int32_t modelDataCount);

/**
 * Release DNN Networks in a given packed handle
 * @param[in] packedDNNHandle
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNRelease(hbPackedDNNHandle_t packedDNNHandle);

/**
 * Get model names from given packed handle
 * @param[out] modelNameList
 * @param[out] modelNameCount
 * @param[in] packedDNNHandle
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetModelNameList(const char ***modelNameList,
                              int32_t *modelNameCount,
                              hbPackedDNNHandle_t packedDNNHandle);

/**
 * Get DNN Network handle from packed Handle with given model name
 * @param[out] dnnHandle
 * @param[in] packedDNNHandle
 * @param[in] modelName
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetModelHandle(hbDNNHandle_t *dnnHandle,
                            hbPackedDNNHandle_t packedDNNHandle,
                            const char *modelName);

/**
 * Get input count
 * @param[out] inputCount
 * @param[in] dnnHandle
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetInputCount(int32_t *inputCount, hbDNNHandle_t dnnHandle);

/**
 * Get input tensor properties
 * @param[out] properties
 * @param[in] dnnHandle
 * @param[in] inputIndex
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetInputTensorProperties(hbDNNTensorProperties *properties,
                                      hbDNNHandle_t dnnHandle,
                                      int32_t inputIndex);

/**
 * Get output count
 * @param[out] outputCount
 * @param[in] dnnHandle
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetOutputCount(int32_t *outputCount, hbDNNHandle_t dnnHandle);

/**
 * Get output tensor properties
 * @param[out] properties
 * @param[in] dnnHandle
 * @param[in] outputIndex
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetOutputTensorProperties(hbDNNTensorProperties *properties,
                                       hbDNNHandle_t dnnHandle,
                                       int32_t outputIndex);

/**
 * Get the count of input stages required to execute the model.
 * By splitting the input data into multiple stages, the model can be
 * pre-executed when part of the input data is loaded.
 * So should call hbDNNInfer once for the 1st stage and hbDNNContinueTask (stageCount - 1) times
 * for the other stages.
 * @param[out] stageCount
 * @param[in] dnnHandle
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetInputStageCount(int32_t *stageCount, hbDNNHandle_t dnnHandle);

/**
 * Get required roi of input tensor to pre-execute the model. 
 * Input data in the roi should be prepared before call hbDNNInfer or hbDNNContinueTask.
 * The requires input roi of later stage must not be less than input roi of earlier stage,
 * and the last stage always requires all data of input tensor. 
 * @param[out] roi
 * @param[in] dnnHandle
 * @param[in] stageIndex [0, stageCount)
 * @param[in] inputIndex [0, inputCount)
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNGetInputStageRequiredRoi(hbDNNRoi *roi, hbDNNHandle_t dnnHandle,
                                      int32_t inputIndex, int32_t stageIndex);

/**
 * DNN inference
 * @param[out] taskHandle: return a pointer represent the task if success,  otherwise nullptr
 * @param[out] output: pointer to the output tensor array, the size of array should be equal to $(`hbDNNGetOutputCount`)
 * @param[in] dnnHandle: pointer to the dnn handle
 * @param[in] input: input tensor array, the size of array should be equal to  $(`hbDNNGetInputCount`)
 * @param[in] inferCtrlParam: infer control parameters
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNInfer(hbDNNTaskHandle_t *taskHandle, hbDNNTensor **output,
                   const hbDNNTensor *input, hbDNNHandle_t dnnHandle,
                   hbDNNInferCtrlParam *inferCtrlParam);

/**
 * DNN inference with rois
 * @param[out] taskHandle: return a pointer represent the task if success,  otherwise nullptr
 * @param[in] dnnHandle: pointer to the dnn handle
 * @param[in] input: input tensor array, the size of array should be equal to  $(`hbDNNGetInputCount`) * roiCount
 *      range of [idx*$(`hbDNNGetInputCount`), (idx+1)*$(`hbDNNGetInputCount`)) represents input tensors
 *      for roi[idx].
 * @param[in] rois: Rois
 * @param[in] roiCount: roi count
 * @param[in] output: pointer to the output tensor array
 * @param[in] inferCtrlParam: infer control parameters
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNRoiInfer(hbDNNTaskHandle_t *taskHandle, hbDNNTensor **output,
                      const hbDNNTensor *input, hbDNNRoi *rois,
                      int32_t roiCount, hbDNNHandle_t dnnHandle,
                      hbDNNInferCtrlParam *inferCtrlParam);

/**
 * Resize image
 * @param[out] taskHandle
 * @param output
 * @param[in] input
 * @param[in] roi
 * @param[in] resizeCtrlParam
 * @return  0 if success, return defined error code otherwise
 */
int32_t hbDNNResize(hbDNNTaskHandle_t *taskHandle, hbDNNTensor *output,
                    const hbDNNTensor *input, const hbDNNRoi *roi,
                    hbDNNResizeCtrlParam *resizeCtrlParam);

/**
 * Continue execute the task, number of calls should be equal to (stageCount - 1)
 * note that required roi of input tensor obtained from hbDNNGetModelStageRequiredInputRoi should be ready
 * @param[in] taskHandle: pointer to the task
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNContinueTask(hbDNNTaskHandle_t taskHandle);

/**
 * Wait util task completed or timeout.
 * @param[in] taskHandle: pointer to the task
 * @param[in] timeout: timeout of milliseconds
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNWaitTaskDone(hbDNNTaskHandle_t taskHandle, int32_t timeout);

/**
 * Release a task and its related resources. If the task has not been executed then it will be canceled,
 * and if the task has not been finished then it will be stopped.
 * This interface will return immediately, and all operations will run in the background
 * @param[in] taskHandle: pointer to the task
 * @return 0 if success, return defined error code otherwise
 */
int32_t hbDNNReleaseTask(hbDNNTaskHandle_t taskHandle);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // DNN_HB_DNN_H_
