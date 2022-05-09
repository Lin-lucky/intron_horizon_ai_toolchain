/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VPS_DATA_TYPE_H_
#define VIDEO_SOURCE_VPS_DATA_TYPE_H_
#include <vector>
#include <string>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "vio/hb_vio_interface.h"
#include "vio/hb_vps_api.h"
#include "vio/hb_vp_api.h"
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#define MAX_GDC_NUM                              (2)
#define MAX_MIPIID_NUM                           (4)
#define MAX_CHN_NUM                              (7)
#define MAX_PIPE_NUM                             (6)
#define MAX_POOL_CNT                             (32)
#define GDC_PIPE_ID                              (MAX_PIPE_NUM+1)
#define TIME_MICRO_SECOND                        (1000000)
#define __FILENAME__ (strrchr(__FILE__, '/') ?\
        (strrchr(__FILE__, '/') + 1):__FILE__)

#define TYPE_NAME(x)    #x

#define CHECK_PARAMS_VALID(p)\
  do {\
    if (p == nullptr)\
    return -1;\
  } while (0)

namespace videosource {

enum VpsGdcType {
  kGDC_TYPE_INVALID = 0,
  kISP_DDR_GDC,
  kIPU_CHN_DDR_GDC,
  kPYM_DDR_GDC,
  kGDC_TYPE_MAX
};

struct VpsIpuInfo {
  int grp_id;
  int ipu_chn;
  hb_vio_buffer_t ipu_buf;
};

struct VpsPymInfo {
  int grp_id;
  int pym_chn;
  int buf_index;
  pym_buffer_t pym_buf;
  hb_vio_buffer_t src_buf;
};

struct VpsGroupDebugInfo {
  int vps_dump_num;
  int vps_layer_dump;
};

struct VpsGroupInputInfo {
  int group_w;
  int group_h;
  int frame_depth;
};

struct VpsGdcInfo {
  int gdc_en;
  int gdc_w;
  int gdc_h;
  int frame_depth;
  int gdc_type;
  int rotate;
  int bind_ipu_chn;
  std::string file_path;
};

struct VpsRoiInfo {
  int roi_en;
  int roi_x;
  int roi_y;
  int roi_w;
  int roi_h;
};

struct VpsScaleInfo {
  int scale_en;
  int scale_w;
  int scale_h;
};

struct VpsChnAttr {
  int ipu_chn_en;
  int pym_chn_en;
  int ipu_frame_depth;
  int ipu_frame_timeout;
  VpsRoiInfo ipu_roi;
  VpsScaleInfo ipu_scale;
};

struct VpsChnInfo {
  std::vector<int> ipu_valid_chn;
  std::vector<int> pym_valid_chn;
};

struct VpsConfig  {
  VpsConfig() {}
  ~VpsConfig() {}
  VpsGroupDebugInfo debug_cfg_;
  VpsGroupInputInfo input_cfg_;
  VpsGdcInfo gdc_cfg_[MAX_GDC_NUM] = { 0 };
  VpsChnAttr chn_cfg_[MAX_CHN_NUM] = { 0 };
  VPS_PYM_CHN_ATTR_S pym_cfg_ = { 0 };
  VpsChnInfo chn_info_;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VPS_DATA_TYPE_H_
