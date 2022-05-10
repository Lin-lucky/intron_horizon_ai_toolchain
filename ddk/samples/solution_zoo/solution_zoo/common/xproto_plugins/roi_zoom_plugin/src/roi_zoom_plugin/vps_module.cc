/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "roi_zoom_plugin/vps_module.h"

#include <math.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <fstream>

#include "hobotlog/hobotlog.hpp"
extern "C" {
#include "vio/hb_mipi_api.h"
#include "vio/hb_sys.h"
#include "vio/hb_venc.h"
#include "vio/hb_vin_api.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_vp_api.h"
#include "vio/hb_vps_api.h"
}

#define IPU_UPSCALE
#define CPOST_2GROUP
// #define PRINT_TIME

namespace xproto {

uint32_t ds_group_id_ = 1;
uint32_t ds_view_id_ = 1;
uint32_t ds_track1_id_ = 2;
uint32_t ds_track2_id_ = 3;
uint32_t us_group_id_ = 2;
uint32_t us2_group_id_ = 3;
uint32_t us_channel_id_ = 5;

uint32_t width_max_ = 1920;
uint32_t height_max_ = 1080;
uint32_t pym_width_ = 960;
uint32_t pym_height_ = 540;
uint32_t two_face_width_ = 480;

VPS_PYM_CHN_ATTR_S vps1_us_1080p_attr;
VPS_PYM_CHN_ATTR_S vps1_us_720p_attr;
VPS_PYM_CHN_ATTR_S vps1_us_960_attr;

std::string getCurrentTimeString() {
  time_t tCurTime = time(NULL);
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm cDateTime = {0};
  localtime_r((const time_t *)&tCurTime, &cDateTime);
  char szInfo[128] = {0};
  snprintf(szInfo, sizeof(szInfo), "%04d-%02d-%02dT%02d:%02d:%02d.%03d",
           cDateTime.tm_year + 1900, cDateTime.tm_mon + 1, cDateTime.tm_mday,
           cDateTime.tm_hour, cDateTime.tm_min, cDateTime.tm_sec,
           (static_cast<int>(tv.tv_usec / 1000)));
  return szInfo;
}

int vps_grp_crop_update(int grp, int chn, int l, int t, int w, int h) {
  VPS_CROP_INFO_S chn_crop_info;
  memset(&chn_crop_info, 0x0, sizeof(VPS_CROP_INFO_S));
  chn_crop_info.en = 1;
  chn_crop_info.cropRect.x = l;
  chn_crop_info.cropRect.y = t;
  chn_crop_info.cropRect.width = w;
  chn_crop_info.cropRect.height = h;
  int nRet = HB_VPS_SetChnCrop(grp, chn, &chn_crop_info);
#ifdef PRINT_TIME
  if (nRet) {
    LOGE << "_______________group:" << grp << " chn:" << chn
         << " update crop box fail, ret = " << nRet
         << " time:" << getCurrentTimeString().c_str();
  } else {
    LOGE << "_______________group:" << grp << " chn:" << chn
         << " update crop box success, time:" << getCurrentTimeString().c_str();
  }
#else
  if (nRet) {
    LOGE << "usb vpsmodule dynamic set group:" << grp << " chn:" << chn
         << " crop fail, return nRet" << nRet << ", crop box:" << l << " " << t
         << " " << w << " " << h;
  }
#endif
  return nRet;
}

float pym_us_ratio[6] = {1.28, 1.6, 2, 2.56, 3.2, 4};

int us_grp_attr_1080p[6][2] = {{1502, 846}, {1202, 676}, {962, 543},
                               {752, 424},  {602, 340},  {482, 272}};

int us_grp_attr_960[6][2] = {{752, 424}, {602, 340}, {482, 272},
                             {376, 212}, {300, 168}, {240, 135}};
int pym_us_config[6][5] = {
    {419, 236, 3002, 1688, 50}, {719, 404, 2402, 1352, 40},
    {959, 539, 1922, 1082, 32}, {1169, 658, 1502, 846, 25},
    {1319, 742, 1202, 678, 20}, {1440, 809, 962, 542, 16}};

int us_grp_vps_chn_attr_init_1080p(VPS_PYM_CHN_ATTR_S *attr) {
  memset(attr, 0x0, sizeof(VPS_PYM_CHN_ATTR_S));
  attr->frame_id = 0;
  attr->ds_uv_bypass = 0;
  attr->ds_layer_en = 5;
  attr->us_layer_en = 6;
  attr->us_uv_bypass = 0;
  attr->timeout = 2000;
  attr->frameDepth = 1;

  for (auto i = 0; i < 6; i++) {
    attr->us_info[i].factor = pym_us_config[i][4];
    attr->us_info[i].roi_x = ceil(pym_us_config[i][0] / 2.0);
    attr->us_info[i].roi_y = ceil(pym_us_config[i][1] / 2.0);
    attr->us_info[i].roi_width = us_grp_attr_1080p[i][0];
    attr->us_info[i].roi_height = us_grp_attr_1080p[i][1];
    printf("vps1 us%d factor:%d x:%d y:%d w:%d h:%d\n", i,
           attr->us_info[i].factor, attr->us_info[i].roi_x,
           attr->us_info[i].roi_y, attr->us_info[i].roi_width,
           attr->us_info[i].roi_height);
  }
  return 0;
}

int us_grp_attr_720p[6][2] = {{1002, 564}, {802, 452}, {642, 362},
                              {503, 284},  {402, 228}, {322, 182}};
int us_grp_vps_chn_attr_init_720p(VPS_PYM_CHN_ATTR_S *attr) {
  memset(attr, 0x0, sizeof(VPS_PYM_CHN_ATTR_S));
  attr->frame_id = 0;
  attr->ds_uv_bypass = 0;
  attr->ds_layer_en = 5;
  attr->us_layer_en = 6;
  attr->us_uv_bypass = 0;
  attr->timeout = 2000;
  attr->frameDepth = 1;

  for (auto i = 0; i < 6; i++) {
    attr->us_info[i].factor = pym_us_config[i][4];
    attr->us_info[i].roi_x = ceil(pym_us_config[i][0] / 3.0);
    attr->us_info[i].roi_y = ceil(pym_us_config[i][1] / 3.0);
    attr->us_info[i].roi_width = us_grp_attr_720p[i][0];
    attr->us_info[i].roi_height = us_grp_attr_720p[i][1];
    printf("vps1 us%d factor:%d x:%d y:%d w:%d h:%d\n", i,
           attr->us_info[i].factor, attr->us_info[i].roi_x,
           attr->us_info[i].roi_y, attr->us_info[i].roi_width,
           attr->us_info[i].roi_height);
  }
  return 0;
}

int us_grp_vps_chn_attr_init_960(VPS_PYM_CHN_ATTR_S *attr) {
  memset(attr, 0x0, sizeof(VPS_PYM_CHN_ATTR_S));
  attr->frame_id = 0;
  attr->ds_uv_bypass = 0;
  attr->ds_layer_en = 0;  // 5;
  attr->us_layer_en = 6;
  attr->us_uv_bypass = 0;
  attr->timeout = 2000;
  attr->frameDepth = 1;

  for (auto i = 0; i < 6; i++) {
    attr->us_info[i].factor = pym_us_config[i][4];
    attr->us_info[i].roi_x = ceil(pym_us_config[i][0] / 4.0);
    attr->us_info[i].roi_y = ceil(pym_us_config[i][1] / 4.0);
    attr->us_info[i].roi_width = us_grp_attr_960[i][0];
    attr->us_info[i].roi_height = us_grp_attr_960[i][1];
    printf("vps1 us%d factor:%d x:%d y:%d w:%d h:%d\n", i,
           attr->us_info[i].factor, attr->us_info[i].roi_x,
           attr->us_info[i].roi_y, attr->us_info[i].roi_width,
           attr->us_info[i].roi_height);
  }
  return 0;
}

int us_vps_grp_init(void) {
#ifdef IPU_UPSCALE
  int ret = 0;
  VPS_CHN_ATTR_S chn_attr;
  memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
  chn_attr.enScale = 1;
  chn_attr.width = pym_width_;
  chn_attr.height = pym_height_;
  chn_attr.frameDepth = 1;

  VPS_CROP_INFO_S chn_crop_info;
  memset(&chn_crop_info, 0x0, sizeof(VPS_CROP_INFO_S));
  chn_crop_info.en = 1;
  chn_crop_info.cropRect.width = pym_width_;
  chn_crop_info.cropRect.height = pym_height_;

  VPS_GRP_ATTR_S grp_attr;
  memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
  grp_attr.maxW = pym_width_;
  grp_attr.maxH = pym_height_;
  grp_attr.frameDepth = 2;
  ret = HB_VPS_CreateGrp(us_group_id_, &grp_attr);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_CreateGrp error!!! group:" << us_group_id_;
    return ret;
  }

  ret = HB_VPS_SetChnAttr(us_group_id_, us_channel_id_, &chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetChnAttr error!!! group:" << us_group_id_
         << " chn:" << us_channel_id_;
    return ret;
  }

  ret = HB_VPS_SetChnCrop(us_group_id_, us_channel_id_, &chn_crop_info);
  if (ret) {
    LOGE << "HB_VPS_SetChnCrop error!!!, group:" << us_group_id_
         << " chn:" << us_channel_id_;
    return ret;
  }

  ret = HB_VPS_EnableChn(us_group_id_, us_channel_id_);
  if (ret) {
    LOGE << "HB_VPS_EnableChn group:" << us_group_id_
         << "chn:" << us_channel_id_ << " error!!!";
    return ret;
  }

  struct HB_SYS_MOD_S src_mod, dst_mod;
  src_mod.enModId = HB_ID_VPS;
  src_mod.s32DevId = ds_group_id_;
  src_mod.s32ChnId = ds_track1_id_;
  dst_mod.enModId = HB_ID_VPS;
  dst_mod.s32DevId = us_group_id_;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_Bind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_Bind group:" << ds_group_id_
         << " and group: " << us_group_id_ << " error!!!";
    return ret;
  }

#ifdef CPOST_2GROUP
  // {{ track2
  ret = HB_VPS_CreateGrp(us2_group_id_, &grp_attr);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_CreateGrp error!!! group:" << us2_group_id_;
    return ret;
  }

  ret = HB_VPS_SetChnAttr(us2_group_id_, us_channel_id_, &chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetChnAttr error!!! group:" << us2_group_id_
         << " chn:" << us_channel_id_;
    return ret;
  }

  ret = HB_VPS_SetChnCrop(us2_group_id_, us_channel_id_, &chn_crop_info);
  if (ret) {
    LOGE << "HB_VPS_SetChnCrop error!!!, group:" << us2_group_id_
         << " chn:" << us_channel_id_;
    return ret;
  }

  ret = HB_VPS_EnableChn(us2_group_id_, us_channel_id_);
  if (ret) {
    LOGE << "HB_VPS_EnableChn group:" << us2_group_id_
         << "chn:" << us_channel_id_ << " error!!!";
    return ret;
  }

  src_mod.s32DevId = ds_group_id_;
  src_mod.s32ChnId = ds_track2_id_;
  dst_mod.s32DevId = us2_group_id_;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_Bind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_Bind group:" << ds_group_id_
         << " and group: " << us2_group_id_ << " error!!!";
    return ret;
  }
  // }}
#endif
  return 0;
#else
  us_grp_vps_chn_attr_init_960(&vps1_us_960_attr);
  // us_grp_vps_chn_attr_init_1080p(&vps1_us_1080p_attr);
  VPS_GRP_ATTR_S grp_attr;
  memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
  grp_attr.maxW = pym_width_;
  grp_attr.maxH = pym_height_;
  int ret = HB_VPS_CreateGrp(us_group_id_, &grp_attr);
  if (ret) {
    LOGE << "HB_VPS_CreateGrp:" << us_group_id_ << " error!!!";
    return ret;
  }

  // HB_SYS_SetVINVPSMode(us_group_id_, VIN_OFFLINE_VPS_OFFINE);
  us_channel_id_ = 6;
  VPS_CHN_ATTR_S chn_attr;
  memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
  chn_attr.enScale = 1;
  chn_attr.width = pym_width_;
  chn_attr.height = pym_height_;
  chn_attr.frameDepth = 0;
  ret = HB_VPS_SetChnAttr(us_group_id_, us_channel_id_, &chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetChnAttr:" << us_group_id_ << " error!!!";
    return ret;
  }

  VPS_CROP_INFO_S chn_crop_info;
  memset(&chn_crop_info, 0x0, sizeof(VPS_CROP_INFO_S));
  chn_crop_info.en = 1;
  chn_crop_info.cropRect.width = width_max_;
  chn_crop_info.cropRect.height = height_max_;
  ret = HB_VPS_SetChnCrop(us_group_id_, us_channel_id_, &chn_crop_info);
  if (ret) {
    LOGE << "HB_VPS_SetChnCrop error!!!, group:" << ds_group_id_
         << " chn:" << us_channel_id_;
    return ret;
  }

  ret = HB_VPS_EnableChn(us_group_id_, us_channel_id_);
  if (ret) {
    LOGE << "HB_VPS_EnableChn group:" << us_group_id_
         << "chn:" << us_channel_id_ << " error!!!";
    return ret;
  }

  VPS_PYM_CHN_ATTR_S pym_chn_attr;
  memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
  // memcpy(&pym_chn_attr, &vps1_us_960_attr, sizeof(VPS_PYM_CHN_ATTR_S));

  ret = HB_VPS_SetPymChnAttr(us_group_id_, us_group_id_, &pym_chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetPymChnAttr group:" << us_group_id_
         << "chn:" << us_channel_id_ << " error!!!";
    return ret;
  }
  return 0;
#if 0
  ret = HB_VPS_CreateGrp(us_group_id_ + 1, &grp_attr);
  if (ret) {
    LOGE << "HB_VPS_CreateGrp:" << us_group_id_ << " error!!!";
    return ret;
  }
  ret = HB_VPS_SetChnAttr(us_group_id_ + 1, 6, &chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetChnAttr:" << us_group_id_ << " error!!!";
    return ret;
  }

  VPS_PYM_CHN_ATTR_S pym_chn_attr;
  memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
  memcpy(&pym_chn_attr, &vps1_us_960_attr, sizeof(VPS_PYM_CHN_ATTR_S));

  ret = HB_VPS_SetPymChnAttr(us_group_id_ + 1, 6, &pym_chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetPymChnAttr group:" << us_group_id_ + 1 << "chn:" << 6
         << " error!!!";
    return ret;
  }
  HB_VPS_EnableChn(us_group_id_ + 1, 6);

  struct HB_SYS_MOD_S src_mod, dst_mod;
  src_mod.enModId = HB_ID_VPS;
  src_mod.s32DevId = ds_group_id_;
  src_mod.s32ChnId = ds_channel_id_;
  dst_mod.enModId = HB_ID_VPS;
  dst_mod.s32DevId = us_group_id_;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_Bind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_Bind group:" << ds_group_id_
         << " and group: " << us_group_id_ << " error!!!";
    return ret;
  }

  src_mod.enModId = HB_ID_VPS;
  src_mod.s32DevId = us_group_id_;
  src_mod.s32ChnId = us_channel_id_;
  dst_mod.enModId = HB_ID_VPS;
  dst_mod.s32DevId = us_group_id_ + 1;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_Bind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_Bind group:" << ds_group_id_
         << " and group: " << us_group_id_ << " error!!!";
    return ret;
  }
#endif
  return 0;
#endif
}

int us_vps_grp_deinit() {
  int ret = 0;
#ifdef IPU_UPSCALE
  struct HB_SYS_MOD_S src_mod, dst_mod;
  src_mod.enModId = HB_ID_VPS;
  src_mod.s32DevId = ds_group_id_;
  src_mod.s32ChnId = ds_track1_id_;

  dst_mod.enModId = HB_ID_VPS;
  dst_mod.s32DevId = us_group_id_;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_UnBind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_UnBind vps failed, group chn:" << ds_group_id_ << " "
         << ds_track1_id_ << ", " << us_group_id_ << " " << us_channel_id_;
    return ret;
  }

#ifdef CPOST_2GROUP
  src_mod.s32DevId = ds_group_id_;
  src_mod.s32ChnId = ds_track2_id_;
  dst_mod.s32DevId = us2_group_id_;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_UnBind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_UnBind vps failed, group chn:" << ds_group_id_ << " "
         << ds_track1_id_ << ", " << us2_group_id_ << " " << us_channel_id_;
    return ret;
  }
#endif

  usleep(30000);
  ret = HB_VPS_StopGrp(us_group_id_);
  if (ret) {
    printf("HB_VPS_StopGrp chn1 error!!!\n");
    return ret;
  }
  ret = HB_VPS_DestroyGrp(us_group_id_);
  if (ret) {
    printf("HB_VPS_DestroyGrp chn1 error!!!\n");
  }

#ifdef CPOST_2GROUP
  ret = HB_VPS_StopGrp(us2_group_id_);
  if (ret) {
    printf("HB_VPS_StopGrp chn1 error!!!\n");
    return ret;
  }
  ret = HB_VPS_DestroyGrp(us2_group_id_);
  if (ret) {
    printf("HB_VPS_DestroyGrp chn1 error!!!\n");
  }
#endif
#else
  struct HB_SYS_MOD_S src_mod, dst_mod;
  src_mod.enModId = HB_ID_VPS;
  src_mod.s32DevId = ds_group_id_;
  src_mod.s32ChnId = ds_channel_id_;

  dst_mod.enModId = HB_ID_VPS;
  dst_mod.s32DevId = us_group_id_;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_UnBind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_UnBind vps failed, group chn:" << ds_group_id_ << " "
         << ds_channel_id_ << ", " << us_group_id_ << " " << us_channel_id_;
    return ret;
  }
  src_mod.enModId = HB_ID_VPS;
  src_mod.s32DevId = us_group_id_;
  src_mod.s32ChnId = us_channel_id_;

  dst_mod.enModId = HB_ID_VPS;
  dst_mod.s32DevId = us_group_id_ + 1;
  dst_mod.s32ChnId = 0;
  ret = HB_SYS_UnBind(&src_mod, &dst_mod);
  if (ret) {
    LOGE << "HB_SYS_UnBind vps failed, group chn:" << ds_group_id_ << " "
         << ds_channel_id_ << ", " << us_group_id_ << " " << us_channel_id_;
    return ret;
  }

  usleep(30000);
  ret = HB_VPS_StopGrp(us_group_id_);
  if (ret) {
    printf("HB_VPS_StopGrp chn1 error!!!\n");
    return ret;
  }
  ret = HB_VPS_DestroyGrp(us_group_id_);
  if (ret) {
    printf("HB_VPS_DestroyGrp chn1 error!!!\n");
  }

  ret = HB_VPS_StopGrp(us_group_id_ + 1);
  if (ret) {
    printf("HB_VPS_StopGrp chn1 error!!!\n");
    return ret;
  }
  ret = HB_VPS_DestroyGrp(us_group_id_ + 1);
  if (ret) {
    printf("HB_VPS_DestroyGrp chn1 error!!!\n");
  }
#endif
  return ret;
}

int ds_vps_grp_init() {
  VPS_GRP_ATTR_S grp_attr;
  memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
  grp_attr.maxW = width_max_;
  grp_attr.maxH = height_max_;
  grp_attr.frameDepth = 1;
  int ret = HB_VPS_CreateGrp(ds_group_id_, &grp_attr);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_CreateGrp error!!! group:" << ds_group_id_;
    return ret;
  }

  VPS_CHN_ATTR_S chn_attr;
  memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
  chn_attr.enScale = 1;
  chn_attr.frameDepth = 1;
  chn_attr.width = pym_width_;
  chn_attr.height = pym_height_;

  // {{ roi view
  ret = HB_VPS_SetChnAttr(ds_group_id_, ds_view_id_, &chn_attr);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_SetChnAttr error!!! group:" << ds_group_id_
         << " chn:" << ds_view_id_;
    return ret;
  }

  VPS_CROP_INFO_S chn_crop_info;
  memset(&chn_crop_info, 0x0, sizeof(VPS_CROP_INFO_S));
  chn_crop_info.en = 1;
  chn_crop_info.cropRect.width = width_max_;
  chn_crop_info.cropRect.height = height_max_;
  ret = HB_VPS_SetChnCrop(ds_group_id_, ds_view_id_, &chn_crop_info);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_SetChnCrop error!!!, group:" << ds_group_id_
         << " chn:" << ds_view_id_;
    return ret;
  }

  ret = HB_VPS_EnableChn(ds_group_id_, ds_view_id_);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_EnableChn group:" << ds_group_id_
         << "chn:" << ds_view_id_ << " error!!!";
    return ret;
  }
  // }}

  // {{ track1
  chn_attr.frameDepth = 0;
  ret = HB_VPS_SetChnAttr(ds_group_id_, ds_track1_id_, &chn_attr);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_SetChnAttr error!!! group:" << ds_group_id_
         << " chn:" << ds_track1_id_;
    return ret;
  }

  ret = HB_VPS_SetChnCrop(ds_group_id_, ds_track1_id_, &chn_crop_info);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_SetChnCrop error!!!, group:" << ds_group_id_
         << " chn:" << ds_track1_id_;
    return ret;
  }

  ret = HB_VPS_EnableChn(ds_group_id_, ds_track1_id_);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_EnableChn group:" << ds_group_id_
         << "chn:" << ds_track1_id_ << " error!!!";
    return ret;
  }
  // }}

#ifdef CPOST_2GROUP
  // {{ track2
  chn_attr.frameDepth = 0;
  ret = HB_VPS_SetChnAttr(ds_group_id_, ds_track2_id_, &chn_attr);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_SetChnAttr error!!! group:" << ds_group_id_
         << " chn:" << ds_track2_id_;
    return ret;
  }

  ret = HB_VPS_SetChnCrop(ds_group_id_, ds_track2_id_, &chn_crop_info);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_SetChnCrop error!!!, group:" << ds_group_id_
         << " chn:" << ds_track2_id_;
    return ret;
  }

  ret = HB_VPS_EnableChn(ds_group_id_, ds_track2_id_);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_EnableChn group:" << ds_group_id_
         << "chn:" << ds_track2_id_ << " error!!!";
    return ret;
  }
#endif
  return 0;
}

int ds_vps_grp_deinit() {
  int ret = HB_VPS_StopGrp(ds_group_id_);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_StopGrp:" << ds_group_id_
         << " Failed. ret = " << ret;
    return ret;
  }
  return 0;
}

VpsModule::VpsModule() : timeout_(40) {}

VpsModule::~VpsModule() { DeInit(); }

int VpsModule::Init() {
  int ret = ds_vps_grp_init();
  if (ret) return ret;
  ret = us_vps_grp_init();
  if (ret) return ret;
  ret = SysInit();
  if (ret) return ret;
  init_ = true;
  LOGE << "usb vps module vps init success!";
  return 0;
}

int VpsModule::SysInit() {
  int ret = HB_SYS_Init();
  if (ret != 0) {
    LOGE << "HB_SYS_Init Failed! ret = " << ret;
    return ret;
  }

  VP_CONFIG_S struVpConf;
  memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
  struVpConf.u32MaxPoolCnt = 32;
  ret = HB_VP_SetConfig(&struVpConf);
  if (ret != 0) {
    LOGE << "HB_VP_SetConfig Failed! ret = " << ret;
    return ret;
  }
  ret = HB_VP_Init();
  if (ret != 0) {
    LOGE << "HB_VP_Init Failed! ret = " << ret;
    return ret;
  }

  uint32_t image_size = width_max_ * height_max_;
  buffer_ =
      reinterpret_cast<hb_vio_buffer_t *>(malloc(sizeof(hb_vio_buffer_t)));
  memset(buffer_, 0, sizeof(hb_vio_buffer_t));
  buffer_->img_addr.width = width_max_;
  buffer_->img_addr.height = height_max_;
  buffer_->img_addr.stride_size = width_max_;
  buffer_->img_info.planeCount = 2;
  buffer_->img_info.img_format = 8;
  ret = HB_SYS_Alloc(&buffer_->img_addr.paddr[0],
                     reinterpret_cast<void **>(&buffer_->img_addr.addr[0]),
                     image_size);
  if (ret == 0) {
    printf("mmzAlloc venc 1080p paddr0 = 0x%lx, vaddr = 0x%p \n",
           buffer_->img_addr.paddr[0], buffer_->img_addr.addr[0]);
  } else {
    LOGE << "usb vps init alloc buffer fail, ret = " << ret;
    return ret;
  }

  ret = HB_SYS_Alloc(&buffer_->img_addr.paddr[1],
                     reinterpret_cast<void **>(&buffer_->img_addr.addr[1]),
                     image_size / 2);
  if (ret != 0) {
    LOGE << "usb vps init alloc buffer fail, ret = " << ret;
    return ret;
  }

  printf("mmzAlloc venc 1080p paddr1 = 0x%lx, vaddr = 0x%p \n",
         buffer_->img_addr.paddr[1], buffer_->img_addr.addr[1]);
  memset(buffer_->img_addr.addr[0], 0, image_size);
  memset(buffer_->img_addr.addr[1], 0, image_size / 2);

  view_buf =
      reinterpret_cast<hb_vio_buffer_t *>(malloc(sizeof(hb_vio_buffer_t)));
  memset(view_buf, 0, sizeof(hb_vio_buffer_t));

  track1_buf =
      reinterpret_cast<hb_vio_buffer_t *>(malloc(sizeof(hb_vio_buffer_t)));
  memset(track1_buf, 0, sizeof(hb_vio_buffer_t));

  track2_buf =
      reinterpret_cast<hb_vio_buffer_t *>(malloc(sizeof(hb_vio_buffer_t)));
  memset(track2_buf, 0, sizeof(hb_vio_buffer_t));
  return 0;
}

int VpsModule::Input(std::shared_ptr<VideoRoiData> &data) {
  // copy data to buffer
  memcpy(buffer_->img_addr.addr[0], data->y_virtual_addr,
         data->width * data->height);
  memcpy(buffer_->img_addr.addr[1], data->uv_virtual_addr,
         data->width * data->height / 2);
#ifdef PRINT_TIME
  printf("\n_____________________send data time:%s\n",
         getCurrentTimeString().c_str());
#endif
  int ret = HB_VPS_SendFrame(ds_group_id_, buffer_, timeout_);
  if (ret != 0) {
    LOGE << "usb vpsmodule HB_VPS_SendFrame Failed. ret = " << ret;
    return ret;
  }
  return 0;
}

int VpsModule::UpdateViewRoi(const RoiInfo &info) {
  std::lock_guard<std::mutex> lk(mutex_);
  vps_grp_crop_update(ds_group_id_, ds_view_id_, info.x, info.y, info.width,
                      info.height);
  return 0;
}

int VpsModule::UpdateTrackRoi(const RoiInfo &info, const RoiInfo *info_tmp,
                              const bool is_second) {
  std::lock_guard<std::mutex> lk(mutex_);
#ifdef CPOST_2GROUP
  if (!is_second) {
    vps_grp_crop_update(ds_group_id_, ds_track1_id_, info_tmp->x, info_tmp->y,
                        info_tmp->width, info_tmp->height);
    vps_grp_crop_update(us_group_id_, us_channel_id_, info.x, info.y,
                        info.width, info.height);
  } else {
    vps_grp_crop_update(ds_group_id_, ds_track2_id_, info_tmp->x, info_tmp->y,
                        info_tmp->width, info_tmp->height);
    vps_grp_crop_update(us2_group_id_, us_channel_id_, info.x, info.y,
                        info.width, info.height);
  }
#else
  vps_grp_crop_update(ds_group_id_, ds_track1_id_, info_tmp->x, info_tmp->y,
                      info_tmp->width, info_tmp->height);
  vps_grp_crop_update(us_group_id_, us_channel_id_, info.x, info.y, info.width,
                      info.height);
#endif
  return 0;
}
int VpsModule::OutputViewData(std::shared_ptr<VideoRoiData> &out_data) {
  return GetFrame(ds_group_id_, ds_view_id_, view_buf, out_data);
}

int VpsModule::OutputTrackData(std::shared_ptr<VideoRoiData> &out_data,
                               const bool is_second) {
#ifdef CPOST_2GROUP
  if (!is_second) {
    return GetFrame(us_group_id_, us_channel_id_, track1_buf, out_data);
  } else {
    return GetFrame(us2_group_id_, us_channel_id_, track2_buf, out_data);
  }
#else
  return GetFrame(us_group_id_, us_channel_id_, track1_buf, out_data);
#endif
}

int VpsModule::GetFrame(const int group, const int channel,
                        hb_vio_buffer_t *gdc_buf,
                        std::shared_ptr<VideoRoiData> &out_data) {
  std::lock_guard<std::mutex> lk(mutex_);
  int ret = HB_VPS_GetChnFrame(group, channel, gdc_buf, timeout_);
#ifdef PRINT_TIME
  if (ret == 0) {
    LOGE << "_______________group:" << group << " chn:" << channel
         << " get data success, time:" << getCurrentTimeString().c_str();
  } else {
    // LOGE << "usb vpsmodule HB_VPS_GetChnFrame failed!, group:" << group
    //     << " chn:" << channel << " return ret:" << ret;
    LOGE << "_______________group:" << group << " chn:" << channel
         << " get data fail, ret=" << ret
         << " time : " << getCurrentTimeString().c_str();
    return ret;
  }
#else
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_GetChnFrame failed!, group:" << group
         << " chn:" << channel << " return ret:" << ret;
    return ret;
  }
#endif
  // printf("------- GetFrame group:%d, channel:%d success!\n", group, channel);
  // printf("------------after crop get frame width:%d, height:%d\n",
  //        gdc_buf->img_addr.width, gdc_buf->img_addr.height);
  uint32_t width = gdc_buf->img_addr.width;
  uint32_t height = gdc_buf->img_addr.height;
  int size = gdc_buf->img_addr.width * gdc_buf->img_addr.height;
  int size1 = gdc_buf->img_addr.width * gdc_buf->img_addr.height / 2;
  out_data->y_virtual_addr = static_cast<char *>(malloc(size));
  out_data->uv_virtual_addr = static_cast<char *>(malloc(size1));
  char *src_buf = gdc_buf->img_addr.addr[0];
  char *src_buf1 = gdc_buf->img_addr.addr[1];
  uint32_t stride = gdc_buf->img_addr.stride_size;
  uint32_t i = 0;

  if (width == stride) {
    memcpy(out_data->y_virtual_addr, src_buf, size);
    memcpy(out_data->uv_virtual_addr, src_buf1, size1);
  } else {
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy(out_data->y_virtual_addr + i * width, src_buf + i * stride, width);
    }

    // jump over stride - width UV
    for (i = 0; i < height / 2; i++) {
      memcpy(out_data->uv_virtual_addr + size + i * width,
             src_buf1 + i * stride, width);
    }
  }

  out_data->width = width;
  out_data->height = height;
  HB_VPS_ReleaseChnFrame(group, channel, gdc_buf);
  return 0;
}

int VpsModule::Process(const RoiInfo &info, std::shared_ptr<VideoRoiData> &data,
                       const RoiInfo *info_tmp, const bool is_second,
                       const bool send_video) {
  if (!init_) return -1;
  int group = ds_group_id_;
  int channel = ds_view_id_;
  // todo 判断是否需要重新配置crop区域
  if (info_tmp == nullptr) {
    vps_grp_crop_update(ds_group_id_, ds_view_id_, info.x, info.y, info.width,
                        info.height);
  } else {
    channel = us_channel_id_;
#ifdef CPOST_2GROUP
    if (!is_second) {
      group = us_group_id_;
      vps_grp_crop_update(ds_group_id_, ds_track1_id_, info_tmp->x, info_tmp->y,
                          info_tmp->width, info_tmp->height);
      vps_grp_crop_update(us_group_id_, us_channel_id_, info.x, info.y,
                          info.width, info.height);
    } else {
      group = us2_group_id_;
      vps_grp_crop_update(ds_group_id_, ds_track2_id_, info_tmp->x, info_tmp->y,
                          info_tmp->width, info_tmp->height);
      vps_grp_crop_update(us2_group_id_, us_channel_id_, info.x, info.y,
                          info.width, info.height);
    }
#else
    group = us_group_id_;
    vps_grp_crop_update(ds_group_id_, ds_track1_id_, info_tmp->x, info_tmp->y,
                        info_tmp->width, info_tmp->height);
    vps_grp_crop_update(us_group_id_, us_channel_id_, info.x, info.y,
                        info.width, info.height);
#endif
  }

  int ret = 0;
  if (send_video) {
    // copy data to buffer
    memcpy(buffer_->img_addr.addr[0], data->y_virtual_addr,
           data->width * data->height);
    memcpy(buffer_->img_addr.addr[1], data->uv_virtual_addr,
           data->width * data->height / 2);
    ret = HB_VPS_SendFrame(ds_group_id_, buffer_, timeout_);
    if (ret != 0) {
      LOGE << "usb vpsmodule HB_VPS_SendFrame Failed. ret = " << ret;
      return ret;
    }
  }

  hb_vio_buffer_t *gdc_buf = view_buf;
  ret = HB_VPS_GetChnFrame(group, channel, gdc_buf, timeout_);
  if (ret != 0) {
    LOGE << "usb vpsmodule Get frame failed! group:" << group
         << " chn:" << channel << " ret:" << ret;
    return ret;
  }

  // printf("------------after crop get frame width:%d, height:%d\n",
  //        gdc_buf->img_addr.width, gdc_buf->img_addr.height);
  int size = gdc_buf->img_addr.width * gdc_buf->img_addr.height;
  int size1 = gdc_buf->img_addr.width * gdc_buf->img_addr.height / 2;
  uint32_t width = gdc_buf->img_addr.width;
  uint32_t height = gdc_buf->img_addr.height;
  if (data->y_virtual_addr &&
      (width != data->width || height != data->height)) {
    free(data->y_virtual_addr);
    data->y_virtual_addr = static_cast<char *>(malloc(width * height));
  }
  if (data->uv_virtual_addr &&
      (width != data->width || height != data->height)) {
    free(data->uv_virtual_addr);
    data->uv_virtual_addr = static_cast<char *>(malloc(width * height / 2));
  }
  char *src_buf = gdc_buf->img_addr.addr[0];
  char *src_buf1 = gdc_buf->img_addr.addr[1];
  uint32_t stride = gdc_buf->img_addr.stride_size;
  uint32_t i = 0;

  if (width == stride) {
    memcpy(data->y_virtual_addr, src_buf, size);
    memcpy(data->uv_virtual_addr, src_buf1, size1);
  } else {
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy(data->y_virtual_addr + i * width, src_buf + i * stride, width);
    }

    // jump over stride - width UV
    for (i = 0; i < height / 2; i++) {
      memcpy(data->uv_virtual_addr + size + i * width, src_buf1 + i * stride,
             width);
    }
  }

  data->width = width;
  data->height = height;
  HB_VPS_ReleaseChnFrame(group, channel, gdc_buf);
  return 0;
}

int VpsModule::Process(std::shared_ptr<VideoRoiData> &in_data,
                       std::shared_ptr<VideoRoiData> &out_data,
                       const RoiInfo &info, const RoiInfo *info_tmp,
                       const bool is_second, const bool send_video) {
  if (!init_) return -1;
  int group = ds_group_id_;
  int channel = ds_view_id_;
  hb_vio_buffer_t *gdc_buf = view_buf;
  // todo 判断是否需要重新配置crop区域
  if (info_tmp == nullptr) {
    vps_grp_crop_update(ds_group_id_, ds_view_id_, info.x, info.y, info.width,
                        info.height);
  } else {
    channel = us_channel_id_;
#ifdef CPOST_2GROUP
    if (!is_second) {
      group = us_group_id_;
      gdc_buf = track1_buf;
      vps_grp_crop_update(ds_group_id_, ds_track1_id_, info_tmp->x, info_tmp->y,
                          info_tmp->width, info_tmp->height);
      vps_grp_crop_update(us_group_id_, us_channel_id_, info.x, info.y,
                          info.width, info.height);
    } else {
      group = us2_group_id_;
      gdc_buf = track2_buf;
      vps_grp_crop_update(ds_group_id_, ds_track2_id_, info_tmp->x, info_tmp->y,
                          info_tmp->width, info_tmp->height);
      vps_grp_crop_update(us2_group_id_, us_channel_id_, info.x, info.y,
                          info.width, info.height);
    }
#else
    group = us_group_id_;
    gdc_buf = track1_buf;
    vps_grp_crop_update(ds_group_id_, ds_track1_id_, info_tmp->x, info_tmp->y,
                        info_tmp->width, info_tmp->height);
    vps_grp_crop_update(us_group_id_, us_channel_id_, info.x, info.y,
                        info.width, info.height);
#endif
  }

  // if ((int)us_channel_id_ == channel)
  printf("-------group:%d, channel:%d, roi x:%d, y:%d, width:%d, height:%d\n",
         group, channel, info.x, info.y, info.width, info.height);

  int ret = 0;
  // copy data to buffer
  if (send_video) {
    int image_size = in_data->width * in_data->height;
    memcpy(buffer_->img_addr.addr[0], in_data->y_virtual_addr, image_size);
    memcpy(buffer_->img_addr.addr[1], in_data->uv_virtual_addr, image_size / 2);
    ret = HB_VPS_SendFrame(ds_group_id_, buffer_, timeout_);
    if (ret != 0) {
      LOGE << "usb vpsmodule HB_VPS_SendFrame Failed. ret = " << ret;
      return ret;
    }
  }

  ret = HB_VPS_GetChnFrame(group, channel, gdc_buf, timeout_);
  if (ret != 0) {
    LOGE << "usb vpsmodule Get frame failed! group:" << group
         << " chn:" << channel << " ret:" << ret;
    return ret;
  }

  printf(
      "------------after crop get frame width:%d, height:%d, group:%d, "
      "chn:%d\n",
      gdc_buf->img_addr.width, gdc_buf->img_addr.height, group, channel);
  uint32_t width = gdc_buf->img_addr.width;
  uint32_t height = gdc_buf->img_addr.height;
  int size = gdc_buf->img_addr.width * gdc_buf->img_addr.height;
  int size1 = gdc_buf->img_addr.width * gdc_buf->img_addr.height / 2;
  out_data->y_virtual_addr = static_cast<char *>(malloc(size));
  out_data->uv_virtual_addr = static_cast<char *>(malloc(size1));
  char *src_buf = gdc_buf->img_addr.addr[0];
  char *src_buf1 = gdc_buf->img_addr.addr[1];
  uint32_t stride = gdc_buf->img_addr.stride_size;
  uint32_t i = 0;

  if (width == stride) {
    memcpy(out_data->y_virtual_addr, src_buf, size);
    memcpy(out_data->uv_virtual_addr, src_buf1, size1);
  } else {
    printf("-----------jump over stride, group:%d, chn:%d\n", group, channel);
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy(out_data->y_virtual_addr + i * width, src_buf + i * stride, width);
    }

    // jump over stride - width UV
    for (i = 0; i < height / 2; i++) {
      memcpy(out_data->uv_virtual_addr + size + i * width,
             src_buf1 + i * stride, width);
    }
  }

  out_data->width = width;
  out_data->height = height;
  HB_VPS_ReleaseChnFrame(group, channel, gdc_buf);
  return 0;
}

int VpsModule::Start() {
  int ret = HB_VPS_StartGrp(ds_group_id_);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_StartGrp:" << ds_group_id_
         << "  Failed. ret = " << ret;
    return ret;
  }
  ret = HB_VPS_StartGrp(us_group_id_);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_StartGrp:" << us_group_id_
         << "  Failed. ret = " << ret;
    return ret;
  }

#ifdef CPOST_2GROUP
  printf("------------usb vpsmodule start us2_group_id_:%d\n", us2_group_id_);
  ret = HB_VPS_StartGrp(us2_group_id_);
  if (ret) {
    LOGE << "usb vpsmodule HB_VPS_StartGrp:" << us2_group_id_
         << "  Failed. ret = " << ret;
    return ret;
  }
#endif
  printf("------------usb vpsmodule start vps group success!!!\n");
  return 0;
}

int VpsModule::Stop() {
  LOGI << "usb roi VpsModule Stop begin";
  ds_vps_grp_deinit();
  us_vps_grp_deinit();
  LOGI << "usb roi VpsModule Stop end !!!";
  return 0;
}

int VpsModule::SysUnit() {
  HB_SYS_Free(buffer_->img_addr.paddr[0], &buffer_->img_addr.addr[0]);
  HB_SYS_Free(buffer_->img_addr.paddr[1], &buffer_->img_addr.addr[1]);
  if (buffer_) {
    free(buffer_);
  }
  if (view_buf) {
    free(view_buf);
  }
  if (track1_buf) {
    free(track1_buf);
  }
  if (track2_buf) {
    free(track2_buf);
  }
  return 0;
}

int VpsModule::DeInit() {
  if (!init_) {
    return 0;
  }
  LOGI << "usb roi VpsModule Deinit begin";
  SysUnit();
  LOGI << "usb roi VpsModule Deinit end !!!";
  return 0;
}

}  // namespace xproto

