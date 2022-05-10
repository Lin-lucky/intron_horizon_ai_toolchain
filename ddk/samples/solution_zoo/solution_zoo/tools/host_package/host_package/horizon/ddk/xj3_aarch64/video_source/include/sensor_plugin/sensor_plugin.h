/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIN_MIPI_CAM_VIN_SENSOR_PLUGIN_H_
#define VIDEO_SOURCE_VIN_MIPI_CAM_VIN_SENSOR_PLUGIN_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "vio/hb_mipi_api.h"
#include "vio/hb_vin_api.h"

typedef enum SensorType {
  kSENSOR_TYPE_INVALID = 0,
  kLINEAR_SENSOR_TYPE,
  kDOL2_SENSOR_TYPE,
  kDOL3_SENSOR_TYPE,
  kSENSOR_TYPE_MAX
} IotSensorType_E;

typedef struct SensorPluginCtrlInfo {
  const char *sensor_name;
  int sensor_type;
  int need_clk;
  void *sensor_ops;
  void *sensor_fd;
} SnsPlgCtrlInfo;

typedef struct SensorPluginModule {
  const char *module;
  int (*get_sns_attr)(SnsPlgCtrlInfo *sensor_info,
      MIPI_SENSOR_INFO_S *pst_sns_attr);
  int (*get_mipi_attr)(SnsPlgCtrlInfo *sensor_info, MIPI_ATTR_S *pst_mipi_attr);
  int (*get_vin_dev_attr)(SnsPlgCtrlInfo *sensor_info,
      VIN_DEV_ATTR_S *pstDevAttr);
  int (*get_vin_dev_attr_ex)(SnsPlgCtrlInfo *sensor_info,
      VIN_DEV_ATTR_EX_S *pstDevAttrEx);
  int (*get_vin_pipe_attr)(SnsPlgCtrlInfo *sensor_info,
      VIN_PIPE_ATTR_S *pstPipeAttr);
  int (*get_dis_attr)(SnsPlgCtrlInfo *sensor_info, VIN_DIS_ATTR_S *pstDisAttr);
  int (*get_ldc_attr)(SnsPlgCtrlInfo *sensor_info, VIN_LDC_ATTR_S *pstLdcAttr);
} SnsPlgModule;

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  // VIDEO_SOURCE_VIN_MIPI_CAM_VIN_SENSOR_PLUGIN_H_
