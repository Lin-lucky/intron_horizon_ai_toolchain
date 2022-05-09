/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_MIPI_CAM_DATA_TYPE_H_
#define VIDEO_SOURCE_MIPI_CAM_DATA_TYPE_H_
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <string>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "vio/hb_sys.h"
#include "vio/hb_mipi_api.h"
#include "vio/hb_vin_api.h"
#ifdef __cplusplus
}
#endif  /* __cplusplus */

namespace videosource {

enum MipiSensorType {
  kSENSOR_ID_INVALID = 0,
  kIMX327_30FPS_1952P_RAW12_LINEAR,   // 1
  kIMX327_30FPS_2228P_RAW12_DOL2,     // 2
  kAR0233_30FPS_1080P_RAW12_954_PWL,  // 3
  kAR0233_30FPS_1080P_RAW12_960_PWL,  // 4
  kOS8A10_30FPS_2160P_RAW10_LINEAR,   // 5
  kOS8A10_30FPS_2160P_RAW10_DOL2,     // 6
  kOV10635_30FPS_720p_954_YUV,        // 7
  kOV10635_30FPS_720p_960_YUV,        // 8
  kSIF_TEST_PATTERN_1080P,           // 9
  kFEED_BACK_RAW12_1952P,             // 10
  kSIF_TEST_PATTERN_YUV_720P,         // 11
  kSIF_TEST_PATTERN_12M_RAW12,        // 12
  kSIF_TEST_PATTERN_4K,               // 13
  kS5KGM1SP_30FPS_4000x3000_RAW10,    // 14
  kOV10635_30FPS_720p_960_YUV_LINE_CONCATENATED,  // 15
  kF37_30FPS_1080P_RAW10_LINEAR,      // 16
  kSC8238_30FPS_2160P_RAW10_LINEAR,   // 17
  kSC8238_30FPS_2160P_RAW10_DOL2,     // 18
  kIMX586_30FPS_2160P_RAW10_LINEAR,   // 19
  kIMX586_30FPS_2160P_RAW10_DOL2,     // 20
  kSAMPLE_SENOSR_ID_MAX
};

enum VinSourceType {
  kVin_SOURCE_INVALID = 0,
  kSIF_BUF_SOURCE,
  kISP_BUF_SOURCE,
  kVin_SOURCE_MAX
};

struct SensorInfo {
  int sensor_id;
  int sensor_plugin_en;
  std::string sensor_plugin_path;
  std::string sensor_plugin_name;
  int sensor_plugin_type;
  int sensor_port;
  int i2c_bus;
  int need_clk;
  int mirror_en;
  int serdes_index;
  int serdes_port;
  int extra_mode;
  int gpio_num;
  int gpio_pin;
  int gpio_level;
};

struct MipiInfo {
  int mipi_index;
  int vc_index;
  int dol2_vc_index;
};

struct SifInfo {
  int need_md;
  int sif_out_buf_num;
};

struct IspInfo {
  int temper_mode;
  int isp_out_buf_num;
  int isp_3a_en;
  int cfa_pattern;
};

struct DweInfo {
  int ldc_en;
  int dis_en;
};

struct VinMipiCamConfig {
  int bind_chn_id;
  int vin_vps_mode;
  // sensor config
  SensorInfo sensor_info;
  // mipi config
  MipiInfo mipi_info;
  // sif config
  SifInfo sif_info;
  // isp config
  IspInfo isp_info;
  // dwe config
  DweInfo dwe_info;
};

struct CamParams {
  MIPI_SENSOR_INFO_S sensor_info;
  MIPI_ATTR_S mipi_info;
};

struct VinParams {
  VIN_DEV_ATTR_S dev_info;
  VIN_DEV_ATTR_EX_S devex_info;
  VIN_PIPE_ATTR_S pipe_info;
  VIN_DIS_ATTR_S dis_info;
  VIN_LDC_ATTR_S ldc_info;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_MIPI_CAM_DATA_TYPE_H_
