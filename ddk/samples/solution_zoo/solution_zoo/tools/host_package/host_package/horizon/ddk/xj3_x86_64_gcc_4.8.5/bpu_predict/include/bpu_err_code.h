/*
 *  Copyright (c) 2019 by Horizon
 * \file bpu_error.h
 * \brief BPU error code
 */

#ifndef BPU_ERROR_CODE_H_
#define BPU_ERROR_CODE_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// Note: The return value of function is negative when error happened,
//       0 when success

// sucess
#define BPU_OK 0

#define BPU_ERR_BASE (-1000)
// bpu common error
#define BPU_ERR_INVALID_HANDLE (BPU_ERR_BASE - 1)
#define BPU_ERR_NOT_INITED (BPU_ERR_BASE - 2)
#define BPU_ERR_RELEASE_FAIL (BPU_ERR_BASE - 3)
#define BPU_ERR_KEY_NAME_NOT_FOUND (BPU_ERR_BASE - 4)
#define BPU_ERR_MEM_NOT_CACHABLE (BPU_ERR_BASE - 5)
#define BPU_ERR_MEM_ALLOC_FAILED (BPU_ERR_BASE - 6)
#define BPU_DMA_MEM_CPY_FAILED (BPU_ERR_BASE - 7)

#define BPU_ERR_NOT_IMPLEMENT (BPU_ERR_BASE - 14)
#define BPU_ERR_INVALID_DATA_TYPE (BPU_ERR_BASE - 15)
#define BPU_ERR_ADD_PADDING_FAILED (BPU_ERR_BASE - 16)
#define BPU_ERR_REMOVE_PADDING_FAILED (BPU_ERR_BASE - 17)
#define BPU_ERR_LOG_REGISTER_FAILED (BPU_ERR_BASE - 18)

// bpu predict error when init instance
#define BPU_ERR_CONFIG_ERR (BPU_ERR_BASE - 100)
#define BPU_ERR_LOAD_HBM_FAILED (BPU_ERR_BASE - 101)
#define BPU_ERR_SET_LOG_LEVEL_FAILED (BPU_ERR_BASE - 102)
#define BPU_ERR_SET_GLOBAL_CONFIG_FAILED (BPU_ERR_BASE - 103)
#define BPU_ERR_GET_MODEL_NUM_FAILED (BPU_ERR_BASE - 104)
#define BPU_ERR_GET_MODEL_NAMES_FAILED (BPU_ERR_BASE - 105)
#define BPU_ERR_GET_MODEL_HANDLE_FAILED (BPU_ERR_BASE - 106)
#define BPU_ERR_GET_MODEL_CORE_NUM_FAILED (BPU_ERR_BASE - 107)
#define BPU_ERR_CREATE_RUNTASK_MEMPOOL_FAILED (BPU_ERR_BASE - 108)
#define BPU_ERR_CREATE_BUFFER_MANAGER_FAILED (BPU_ERR_BASE - 109)
#define BPU_ERR_ENGINE_INIT_FAILED (BPU_ERR_BASE - 110)

// bpu predict error when run model
#define BPU_ERR_INVALID_MODEL_NAME (BPU_ERR_BASE - 111)
#define BPU_ERR_INVALID_CORE_ID (BPU_ERR_BASE - 112)
#define BPU_ERR_INVALID_PARAMETER (BPU_ERR_BASE - 113)
#define BPU_ERR_LOAD_MODEL_PACKAGE_FAILED (BPU_ERR_BASE - 114)
#define BPU_ERR_RUNTASK_ALLOC_FAILED (BPU_ERR_BASE - 200)
#define BPU_ERR_RUNTASK_INIT_FAILED (BPU_ERR_BASE - 201)
#define BPU_ERR_RUNTASK_ADD_FAILED (BPU_ERR_BASE - 202)
#define BPU_ERR_RUNTASK_NO_BBOX_FOR_RESIZER (BPU_ERR_BASE - 203)

#define BPU_ERR_RUNTASK_SET_INPUT_PYM_LAYER_ERR (BPU_ERR_BASE - 204)
#define BPU_ERR_RUNTASK_SET_INPUT_PYM_SHAPE_MISMATCH (BPU_ERR_BASE - 205)
#define BPU_ERR_RUNTASK_SET_INPUT_PYM_STEP_MISMATCH (BPU_ERR_BASE - 206)
#define BPU_ERR_RUNTASK_SET_INPUT_EXTRA_SIZE_ERR (BPU_ERR_BASE - 207)
#define BPU_ERR_RUNTASK_SET_INPUT_ALLOC_CNNMEM_ERR (BPU_ERR_BASE - 208)
#define BPU_ERR_RUNTASK_SET_INPUT_INVALID_STARTXY (BPU_ERR_BASE - 209)
#define BPU_ERR_RUNTASK_SET_INPUT_ROI_EXCEED (BPU_ERR_BASE - 210)
#define BPU_ERR_RUNTASK_SET_INPUT_ROI_NOT_ALIGNED (BPU_ERR_BASE - 211)
#define BPU_ERR_RUNTASK_SET_INPUT_PYMBUFF_NULLPTR (BPU_ERR_BASE - 212)
#define BPU_ERR_RUNTASK_SET_INPUT_GET_NORM_PARAM_FAILED (BPU_ERR_BASE - 213)
#define BPU_ERR_RUNTASK_SET_INPUT_FAKEIMG_NULLPTR (BPU_ERR_BASE - 214)
#define BPU_ERR_RUNTASK_SET_OUTPUT_NUM_MISMATCH (BPU_ERR_BASE - 215)
#define BPU_ERR_RUNTASK_SET_OUTPUT_ALLOC_CNNMEM_ERR (BPU_ERR_BASE - 216)

#define BPU_ERR_RUNTASK_INVALID_TASKID (BPU_ERR_BASE - 217)
#define BPU_ERR_RUNTASK_INVALID_INPUT_SOURCE (BPU_ERR_BASE - 218)
#define BPU_ERR_RUNTASK_MODEL_INPUT_ISNOT_RESIZER (BPU_ERR_BASE - 219)
#define BPU_ERR_RUNTASK_RESIZER_HBRT_RISTART_FAILED (BPU_ERR_BASE - 220)
#define BPU_ERR_RUNTASK_HBRT_RI_START_FAILED (BPU_ERR_BASE - 221)
#define BPU_ERR_RUNTASK_HBRT_RI_GET_OUTPUT_FAILED (BPU_ERR_BASE - 222)
#define BPU_ERR_RUNTASK_SET_FC_FAILED (BPU_ERR_BASE - 223)
#define BPU_ERR_RUNTASK_PRE_READ_OUTPUT_FAILED (BPU_ERR_BASE - 224)
#define BPU_ERR_RUNTASK_NOT_DONE (BPU_ERR_BASE - 225)
#define BPU_ERR_RUNTASK_RELEASE_FAILED (BPU_ERR_BASE - 226)

#define BPU_ERR_RUNTASK_SET_INPUT_NUM_MISMATCH (BPU_ERR_BASE - 227)
#define BPU_ERR_HBDK_VERSION_ERR (BPU_ERR_BASE - 228)
#define BPU_ERR_GET_MODEL_VERSION_FAILED (BPU_ERR_BASE - 229)
#define BPU_ERR_GET_HBM_VERSION_FAILED (BPU_ERR_BASE - 230)
#define BPU_ERR_RUNTASK_GET_OUTPUT_FAILED (BPU_ERR_BASE - 231)
#define BPU_ERR_RUNTASK_SET_INPUT_SIZE_ERR (BPU_ERR_BASE - 232)
#define BPU_ERR_RUNTASK_SET_INPUT_TYPE_ERR (BPU_ERR_BASE - 233)
#define BPU_ERR_RESIZE_OUTPUT_MEM_INVALID (BPU_ERR_BASE - 234)
#define BPU_ERR_RESIZE_OUTPUT_SHAPE_INVALID (BPU_ERR_BASE - 235)
#define BPU_ERR_RESIZE_INPUT_SHAPE_INVALID (BPU_ERR_BASE - 236)
#define BPU_ERR_RESIZE_INPUT_MEM_INVALID (BPU_ERR_BASE - 237)
#define BPU_ERR_ENABLE_PREEMPTION_FAILED (BPU_ERR_BASE - 238)
#define BPU_ERR_RUNTASK_INPUT_SHAPE_LAYOUT_INVALID (BPU_ERR_BASE - 239)
#define BPU_ERR_INVALID_LAYOUT (BPU_ERR_BASE - 240)

#define BPU_ERR_PYM_RESIZE_INPUT_LAYOUT_SET_ERROR (BPU_ERR_BASE - 241)
#define BPU_ERR_PYM_RESIZE_INPUT_NDIM_SET_ERROR (BPU_ERR_BASE - 242)
#define BPU_ERR_PYM_RESIZE_OUTPUT_LAYOUT_SET_ERROR (BPU_ERR_BASE - 243)
#define BPU_ERR_PYM_RESIZE_OUTPUT_NDIM_SET_ERROR (BPU_ERR_BASE - 244)
#define BPU_ERR_PYM_RESIZE_INPUT_SHAPE_SET_ERROR (BPU_ERR_BASE - 245)
#define BPU_ERR_PYM_RESIZE_OUTPUT_SHAPE_SET_ERROR (BPU_ERR_BASE - 246)
#define BPU_ERR_PYM_RESIZE_INPUT_MEM_SIZE_SET_ERROR (BPU_ERR_BASE - 247)
#define BPU_ERR_PYM_RESIZE_OUTPUT_MEM_SIZE_SET_ERROR (BPU_ERR_BASE - 248)

#define BPU_ERR_OUTPUT_NUM_LESS_THAN_BBOX_NUMBER (BPU_ERR_BASE - 249)

#define BPU_ERR_PYM_NO_SUITABLE_LEVEL (BPU_ERR_BASE - 250)

#define BPU_ERR_RUNTASK_SET_INPUT_CAMERA_BUFF_NULLPTR (BPU_ERR_BASE - 251)
#define BPU_ERR_RUNTASK_SET_INPUT_NORMALIZE_ROI_ERROR (BPU_ERR_BASE - 252)
#define BPU_ERR_RUNTASK_SET_INPUT_UNSUPPORTED_FEATURE (BPU_ERR_BASE - 253)
#define BPU_ERR_RUNTASK_SET_INPUT_RESIZE_RIO_FAILED (BPU_ERR_BASE - 254)
#define BPU_ERR_CHECK_MODEL_VALIDITY_FAILED (BPU_ERR_BASE - 255)
#define BPU_ERR_MODEL_OUTPUT_NUM_ERROR (BPU_ERR_BASE - 256)

// vio related error in bpu io
#define BPU_ERR_VIO_BASE (-2000)

#define BPU_ERR_VIO_PAESER_FAIL (BPU_ERR_VIO_BASE - 1)
#define BPU_ERR_VIO_OPEN_CFG_FAIL (BPU_ERR_VIO_BASE - 2)
#define BPU_ERR_VIO_INIT_FAIL (BPU_ERR_VIO_BASE - 3)
#define BPU_ERR_VIO_START_FAIL (BPU_ERR_VIO_BASE - 4)
#define BPU_ERR_VIO_STOP_FAIL (BPU_ERR_VIO_BASE - 5)
#define BPU_ERR_VIO_PYM_IS_BUSY (BPU_ERR_VIO_BASE - 6)

#define BPU_ERR_VIO_SIF_OPEN_DEV_FAIL (BPU_ERR_VIO_BASE - 7)
#define BPU_ERR_VIO_SIF_INIT_FAIL (BPU_ERR_VIO_BASE - 8)
#define BPU_ERR_VIO_SIF_UPDATE_FAIL (BPU_ERR_VIO_BASE - 9)
#define BPU_ERR_VIO_SIF_STOP_FAIL (BPU_ERR_VIO_BASE - 10)
#define BPU_ERR_VIO_SIF_START_FAIL (BPU_ERR_VIO_BASE - 11)
#define BPU_ERR_VIO_SIF_PARSER_FAIL (BPU_ERR_VIO_BASE - 12)
#define BPU_ERR_VIO_SIF_EPOLL_CREATE_FAIL (BPU_ERR_VIO_BASE - 13)
#define BPU_ERR_VIO_SIF_EPOLL_CTL_FAIL (BPU_ERR_VIO_BASE - 14)
#define BPU_ERR_VIO_SIF_EPOLL_WAIT_FAIL (BPU_ERR_VIO_BASE - 15)
#define BPU_ERR_VIO_SIF_STOP_WORKING (BPU_ERR_VIO_BASE - 16)

#define BPU_ERR_VIO_IPU_INIT_FAIL (BPU_ERR_VIO_BASE - 17)
#define BPU_ERR_VIO_IPU_DEINIT_FAIL (BPU_ERR_VIO_BASE - 18)
#define BPU_ERR_VIO_IPU_START_FAIL (BPU_ERR_VIO_BASE - 19)
#define BPU_ERR_VIO_IPU_STOP_FAIL (BPU_ERR_VIO_BASE - 20)
#define BPU_ERR_VIO_IPU_PARSER_FAIL (BPU_ERR_VIO_BASE - 21)
#define BPU_ERR_VIO_IPU_EPOLL_CREATE_FAIL (BPU_ERR_VIO_BASE - 22)
#define BPU_ERR_VIO_IPU_EPOLL_CTL_FAIL (BPU_ERR_VIO_BASE - 23)
#define BPU_ERR_VIO_IPU_EPOLL_WAIT_FAIL (BPU_ERR_VIO_BASE - 24)
#define BPU_ERR_VIO_IPU_STOP_WORKING (BPU_ERR_VIO_BASE - 25)

// cam related error in bpu io
#define BPU_ERR_CAM_BASE (-3000)
#define BPU_ERR_CAM_PARSE_BOARD_CFG_FAIL (BPU_ERR_CAM_BASE - 1)
#define BPU_ERR_CAM_PARSE_MIPI_CFG_FAIL (BPU_ERR_CAM_BASE - 2)
#define BPU_ERR_CAM_DLOPEN_LIBRARY_FAIL (BPU_ERR_CAM_BASE - 3)
#define BPU_ERR_CAM_INIT_FAIL (BPU_ERR_CAM_BASE - 4)
#define BPU_ERR_CAM_DEINIT_FAILL (BPU_ERR_CAM_BASE - 5)
#define BPU_ERR_CAM_START_FAIL (BPU_ERR_CAM_BASE - 6)
#define BPU_ERR_CAM_STOP_FAIL (BPU_ERR_CAM_BASE - 7)
#define BPU_ERR_CAM_INVALID_PARAM (BPU_ERR_CAM_BASE - 8)

// pym related error
#define BPU_ERR_PYM_BASE (-4000)
#define BPU_ERR_RELEASE_PYM_ERROR (BPU_ERR_PYM_BASE - 1)
#define BPU_ERR_SET_PYM_CHN_ATTR_ERROR (BPU_ERR_PYM_BASE - 2)
#define BPU_ERR_SET_GRP_ATTR_ERROR (BPU_ERR_PYM_BASE - 3)
#define BPU_ERR_STOP_GRP_ERROR (BPU_ERR_PYM_BASE - 4)
#define BPU_ERR_DESTROY_GRP_ERROR (BPU_ERR_PYM_BASE - 5)
#define BPU_ERR_RELEASE_CHN_FRAME_ERROR (BPU_ERR_PYM_BASE - 6)
#define BPU_ERR_GET_CHN_FRAME_ERROR (BPU_ERR_PYM_BASE - 7)
#define BPU_ERR_SEND_FRAME_ERROR (BPU_ERR_PYM_BASE - 8)
#define BPU_ERR_START_GRP_ERROR (BPU_ERR_PYM_BASE - 9)
#define BPU_ERR_ENABLE_CHN_ERROR (BPU_ERR_PYM_BASE - 10)

static inline const char* BPU_getErrorName(int error) {
  switch (error) {
    case BPU_OK:
      return "bpu success";
    case BPU_ERR_INVALID_HANDLE:
      return "bpu input handle is invalid";
    case BPU_ERR_RELEASE_FAIL:
      return "bpu release fail";
    case BPU_ERR_KEY_NAME_NOT_FOUND:
      return "bpu key name is not found";
    case BPU_ERR_MEM_NOT_CACHABLE:
      return "bpu mem is not cachable";
    case BPU_ERR_NOT_INITED:
      return "bpu instance is not inited";
    case BPU_ERR_NOT_IMPLEMENT:
      return "this funtion not implement";
    case BPU_ERR_CONFIG_ERR:
      return "bpu config error";
    case BPU_ERR_LOAD_HBM_FAILED:
      return "bpu load hbm file failed";
    case BPU_ERR_SET_LOG_LEVEL_FAILED:
      return "bpu set hbrt log level failed";
    case BPU_ERR_SET_GLOBAL_CONFIG_FAILED:
      return "bpu set hbrt global config failed";
    case BPU_ERR_GET_MODEL_NUM_FAILED:
      return "bpu hbrt get model num failed";
    case BPU_ERR_GET_MODEL_NAMES_FAILED:
      return "bpu hbrt get model names failed";
    case BPU_ERR_GET_MODEL_HANDLE_FAILED:
      return "bpu hbrt get model handle failed";
    case BPU_ERR_GET_MODEL_CORE_NUM_FAILED:
      return "bpu hbrt get model core num failed";
    case BPU_ERR_CREATE_RUNTASK_MEMPOOL_FAILED:
      return "bpu create model runtask mem pool failed";
    case BPU_ERR_CREATE_BUFFER_MANAGER_FAILED:
      return "bpu create buffer manager failed";
    case BPU_ERR_ENGINE_INIT_FAILED:
      return "bpu engine init failed";
    case BPU_ERR_MEM_ALLOC_FAILED:
      return "bpu mem alloc failed";
    case BPU_DMA_MEM_CPY_FAILED:
      return "bpu dma mem copy failed";
    case BPU_ERR_INVALID_MODEL_NAME:
      return "bpu input model name is invalid";
    case BPU_ERR_INVALID_CORE_ID:
      return "bpu input core id is invalid";
    case BPU_ERR_INVALID_PARAMETER:
      return "bpu input parameter is invalid";
    case BPU_ERR_LOAD_MODEL_PACKAGE_FAILED:
      return "bpu load model pack failed";
    case BPU_ERR_RUNTASK_ALLOC_FAILED:
      return "bpu model runtask alloc failed";
    case BPU_ERR_RUNTASK_INIT_FAILED:
      return "bpu model runtask init failed";
    case BPU_ERR_RUNTASK_ADD_FAILED:
      return "bpu model runtask add failed";
    case BPU_ERR_RUNTASK_NO_BBOX_FOR_RESIZER:
      return "bpu model runtask has no bbox for resizer";
    case BPU_ERR_RUNTASK_SET_INPUT_PYM_LAYER_ERR:
      return "bpu model runtask set input pym layer error";
    case BPU_ERR_RUNTASK_SET_INPUT_PYM_SHAPE_MISMATCH:
      return "bpu model runtask set input pym shape mismatch";
    case BPU_ERR_RUNTASK_SET_INPUT_PYM_STEP_MISMATCH:
      return "bpu model runtask set input pym step mismatch";
    case BPU_ERR_RUNTASK_SET_INPUT_EXTRA_SIZE_ERR:
      return "bpu model runtask set input extra data size error";
    case BPU_ERR_RUNTASK_SET_INPUT_ALLOC_CNNMEM_ERR:
      return "bpu model runtask set input alloc cnn mem error";
    case BPU_ERR_RUNTASK_SET_INPUT_INVALID_STARTXY:
      return "bpu model runtask set input invalid start X or Y";
    case BPU_ERR_RUNTASK_SET_INPUT_ROI_EXCEED:
      return "bpu model runtask set input roi exceed origin image";
    case BPU_ERR_RUNTASK_SET_INPUT_ROI_NOT_ALIGNED:
      return "bpu model runtask set input roi address is nog aligned";
    case BPU_ERR_RUNTASK_SET_INPUT_PYMBUFF_NULLPTR:
      return "bpu model runtask parameter input pym buffer is null";
    case BPU_ERR_RUNTASK_SET_INPUT_GET_NORM_PARAM_FAILED:
      return "bpu model runtask set input get norm param failed";
    case BPU_ERR_RUNTASK_SET_INPUT_FAKEIMG_NULLPTR:
      return "bpu model runtask set input fake image buffer is nullptr";
    case BPU_ERR_RUNTASK_SET_INPUT_NUM_MISMATCH:
      return "bpu model runtask set input num mismatch";
    case BPU_ERR_RUNTASK_SET_OUTPUT_NUM_MISMATCH:
      return "bpu model runtask set output num mismatch";
    case BPU_ERR_RUNTASK_SET_OUTPUT_ALLOC_CNNMEM_ERR:
      return "bpu model runtask set output alloc cnn mem error";
    case BPU_ERR_RUNTASK_INVALID_TASKID:
      return "bpu model runtask task id is invalid";
    case BPU_ERR_RUNTASK_INVALID_INPUT_SOURCE:
      return "bpu model runtask input source is invalid";
    case BPU_ERR_RUNTASK_MODEL_INPUT_ISNOT_RESIZER:
      return "bpu model runtask input is not for resizer";
    case BPU_ERR_RUNTASK_RESIZER_HBRT_RISTART_FAILED:
      return "bpu model runtask hbrt resizer ri start failed";
    case BPU_ERR_RUNTASK_HBRT_RI_START_FAILED:
      return "bpu model runtask hbrt ri start failed";
    case BPU_ERR_RUNTASK_HBRT_RI_GET_OUTPUT_FAILED:
      return "bpu model runtask hbrt fi get output failed";
    case BPU_ERR_RUNTASK_SET_FC_FAILED:
      return "bpu model runtask set function call failed";
    case BPU_ERR_RUNTASK_PRE_READ_OUTPUT_FAILED:
      return "bpu model runtask pre read output failed";
    case BPU_ERR_RUNTASK_NOT_DONE:
      return "bpu model runtask is not done";
    case BPU_ERR_RUNTASK_RELEASE_FAILED:
      return "bpu model runtask release failed";
    case BPU_ERR_HBDK_VERSION_ERR:
      return "hbdk version error";
    case BPU_ERR_GET_MODEL_VERSION_FAILED:
      return "bpu get model version failed";
    case BPU_ERR_GET_HBM_VERSION_FAILED:
      return "bpu get hbm version failed";
    case BPU_ERR_RUNTASK_GET_OUTPUT_FAILED:
      return "bpu runtask get output failed";
    case BPU_ERR_RUNTASK_SET_INPUT_SIZE_ERR:
      return "bpu runtask input size not match";
    case BPU_ERR_RUNTASK_SET_INPUT_TYPE_ERR:
      return "bpu runtask input data type invalid";
    case BPU_ERR_RESIZE_OUTPUT_MEM_INVALID:
      return "bpu resize output memory invalid";
    case BPU_ERR_RESIZE_OUTPUT_SHAPE_INVALID:
      return "bpu resize output shape invalid";
    case BPU_ERR_RESIZE_INPUT_SHAPE_INVALID:
      return "bpu resize input shape invalid";
    case BPU_ERR_RESIZE_INPUT_MEM_INVALID:
      return "bpu resize input memory invalid";
    case BPU_ERR_RUNTASK_INPUT_SHAPE_LAYOUT_INVALID:
      return "bpu runtask input shape and layout invalid";
    case BPU_ERR_INVALID_LAYOUT:
      return "bpu input layout invalid or unsupported";
    case BPU_ERR_OUTPUT_NUM_LESS_THAN_BBOX_NUMBER:
      return "bpu output number less than bbox number";
    case BPU_ERR_PYM_NO_SUITABLE_LEVEL:
      return "bpu pyramid no suitable level";
    case BPU_ERR_RUNTASK_SET_INPUT_CAMERA_BUFF_NULLPTR:
      return "bpu model runtask parameter input camera buffer is null";
    case BPU_ERR_RUNTASK_SET_INPUT_NORMALIZE_ROI_ERROR:
      return "bpu model runtask set input normalize roi failed";
    case BPU_ERR_RUNTASK_SET_INPUT_UNSUPPORTED_FEATURE:
      return "bpu model runtask set input unsupported feature";
    case BPU_ERR_RUNTASK_SET_INPUT_RESIZE_RIO_FAILED:
      return "bpu model runtask set input resize rio failed";
    case BPU_ERR_CHECK_MODEL_VALIDITY_FAILED:
      return "bpu model check model validity failed";
    case BPU_ERR_MODEL_OUTPUT_NUM_ERROR:
      return "bpu model output num invalid";
    case BPU_ERR_VIO_BASE:
      return "vio base error";
    case BPU_ERR_VIO_PAESER_FAIL:
      return "vio parse config failed";
    case BPU_ERR_VIO_OPEN_CFG_FAIL:
      return "vio open config file failed";
    case BPU_ERR_VIO_INIT_FAIL:
      return "vio init failed";
    case BPU_ERR_VIO_START_FAIL:
      return "vio start failed";
    case BPU_ERR_VIO_STOP_FAIL:
      return "vio stop failed";
    case BPU_ERR_VIO_PYM_IS_BUSY:
      return "vio pym is busy";
    case BPU_ERR_VIO_SIF_OPEN_DEV_FAIL:
      return "vio sif open device failed";
    case BPU_ERR_VIO_SIF_INIT_FAIL:
      return "vio sif init failed";
    case BPU_ERR_VIO_SIF_UPDATE_FAIL:
      return "vio sif update failed";
    case BPU_ERR_VIO_SIF_STOP_FAIL:
      return "vio sif stop failed";
    case BPU_ERR_VIO_SIF_START_FAIL:
      return "vio sif start failed";
    case BPU_ERR_VIO_SIF_PARSER_FAIL:
      return "vio sif parser config failed";
    case BPU_ERR_VIO_SIF_EPOLL_CREATE_FAIL:
      return "vio sif epoll create failed";
    case BPU_ERR_VIO_SIF_EPOLL_CTL_FAIL:
      return "vio sif epoll ctl failed";
    case BPU_ERR_VIO_SIF_EPOLL_WAIT_FAIL:
      return "vio sif epoll wait failed";
    case BPU_ERR_VIO_SIF_STOP_WORKING:
      return "vio sif stop working";
    case BPU_ERR_VIO_IPU_INIT_FAIL:
      return "vio ipu init failed";
    case BPU_ERR_VIO_IPU_DEINIT_FAIL:
      return "vio ipu deinit failed";
    case BPU_ERR_VIO_IPU_START_FAIL:
      return "vio ipu start failed";
    case BPU_ERR_VIO_IPU_STOP_FAIL:
      return "vio ipu stop failed";
    case BPU_ERR_VIO_IPU_PARSER_FAIL:
      return "vio ipu parse config failed";
    case BPU_ERR_VIO_IPU_EPOLL_CREATE_FAIL:
      return "vio ipu epoll create failed";
    case BPU_ERR_VIO_IPU_EPOLL_CTL_FAIL:
      return "vio ipu epoll ctl failed";
    case BPU_ERR_VIO_IPU_EPOLL_WAIT_FAIL:
      return "vio ipu epoll wait failed";
    case BPU_ERR_VIO_IPU_STOP_WORKING:
      return "vio ipu stop working";
    case BPU_ERR_CAM_BASE:
      return "cam base error";
    case BPU_ERR_CAM_PARSE_BOARD_CFG_FAIL:
      return "cam parse board cfg error";
    case BPU_ERR_CAM_PARSE_MIPI_CFG_FAIL:
      return "cam parse mipi cfg error";
    case BPU_ERR_CAM_DLOPEN_LIBRARY_FAIL:
      return "open cam library failed";
    case BPU_ERR_CAM_INIT_FAIL:
      return "cam init failed";
    case BPU_ERR_CAM_DEINIT_FAILL:
      return "cam deinit failed";
    case BPU_ERR_CAM_START_FAIL:
      return "cam start failed";
    case BPU_ERR_CAM_STOP_FAIL:
      return "cam stop failed";
    case BPU_ERR_CAM_INVALID_PARAM:
      return "cam invalid param";
    case BPU_ERR_ENABLE_PREEMPTION_FAILED:
      return "set model prior failed";
    case BPU_ERR_RELEASE_PYM_ERROR:
      return "release pym chn frame failed";
    case BPU_ERR_SET_PYM_CHN_ATTR_ERROR:
      return "pym resize set pym chn attr error";
    case BPU_ERR_SET_GRP_ATTR_ERROR:
      return "pym resize set grp attr error";
    case BPU_ERR_STOP_GRP_ERROR:
      return "pym resize stop grp error";
    case BPU_ERR_DESTROY_GRP_ERROR:
      return "pym resize destroy grp error";
    case BPU_ERR_RELEASE_CHN_FRAME_ERROR:
      return "pym resize release chn frame error";
    case BPU_ERR_GET_CHN_FRAME_ERROR:
      return "pym resize get chn frame error";
    case BPU_ERR_SEND_FRAME_ERROR:
      return "pym resize send frame error";
    case BPU_ERR_START_GRP_ERROR:
      return "pym resize start grp error";
    case BPU_ERR_ENABLE_CHN_ERROR:
      return "pym resize enable chn error";
    case BPU_ERR_PYM_RESIZE_INPUT_LAYOUT_SET_ERROR:
      return "pym resize input layout set error";
    case BPU_ERR_PYM_RESIZE_INPUT_NDIM_SET_ERROR:
      return "pym resize input dim set error";
    case BPU_ERR_PYM_RESIZE_OUTPUT_LAYOUT_SET_ERROR:
      return "pym resize output layout set error";
    case BPU_ERR_PYM_RESIZE_OUTPUT_NDIM_SET_ERROR:
      return "pym resize output ndim set error";
    case BPU_ERR_PYM_RESIZE_INPUT_SHAPE_SET_ERROR:
      return "pym resize input shape set error";
    case BPU_ERR_PYM_RESIZE_OUTPUT_SHAPE_SET_ERROR:
      return "pym resize output shape set error";
    case BPU_ERR_PYM_RESIZE_INPUT_MEM_SIZE_SET_ERROR:
      return "pym resize input mem size set error";
    case BPU_ERR_PYM_RESIZE_OUTPUT_MEM_SIZE_SET_ERROR:
      return "pym resize output mem size set error";
    case BPU_ERR_INVALID_DATA_TYPE:
      return "bpu error invalid data type";
    case BPU_ERR_ADD_PADDING_FAILED:
      return "bpu error add padding.";
    case BPU_ERR_REMOVE_PADDING_FAILED:
      return "bpu error remove padding.";
    case BPU_ERR_LOG_REGISTER_FAILED:
      return "bpu error log register.";
  }
  return "invalid bpu error code";
}

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // BPU_ERROR_CODE_H_
