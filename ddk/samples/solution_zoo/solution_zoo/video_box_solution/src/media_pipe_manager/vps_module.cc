/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "media_pipe_manager/vps_module.h"

#include <fstream>

#include "hobotlog/hobotlog.hpp"
#ifdef __cplusplus
extern "C" {
#include "hb_vps_api.h"
}
#endif

namespace solution {
namespace video_box {

VpsModule::VpsModule()
    : group_id_(-1), timeout_(40), frameDepth_(-1), buffer_index_(0) {}

VpsModule::~VpsModule() {}

int VpsModule::Init(uint32_t group_id, const PipeModuleInfo *module_info) {
  int ret = 0;
  group_id_ = group_id;
  frameDepth_ = module_info->frame_depth;
  buffers_.resize(frameDepth_);
  VPS_GRP_ATTR_S grp_attr;
  memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
  grp_attr.maxW = module_info->input_width;
  grp_attr.maxH = module_info->input_height;
  grp_attr.frameDepth = module_info->frame_depth;
  ret = HB_VPS_CreateGrp(group_id, &grp_attr);
  if (ret) {
    LOGE << "HB_VPS_CreateGrp Failed. group:" << group_id << " ret = " << ret;
    return ret;
  }
  VPS_CHN_ATTR_S chn_attr;
  memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
  chn_attr.enScale = 1;
  chn_attr.width = module_info->output_width;
  chn_attr.height = module_info->output_height;
  chn_attr.frameDepth = module_info->frame_depth;
  // Now only handle one chanel case
  ret = HB_VPS_SetChnAttr(group_id, 6, &chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetChnAttr Failed. group:" << group_id << " ret = " << ret;
    return ret;
  }

  VPS_PYM_CHN_ATTR_S pym_chn_attr;
  memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
  pym_chn_attr.timeout = 2000;
  pym_chn_attr.ds_layer_en = 8;
  pym_chn_attr.us_layer_en = 0;
  pym_chn_attr.frame_id = 0;
  pym_chn_attr.frameDepth = module_info->frame_depth;
  pym_chn_attr.ds_info[1].factor = 32;
  pym_chn_attr.ds_info[1].roi_x = 0;
  pym_chn_attr.ds_info[1].roi_y = 0;
  pym_chn_attr.ds_info[1].roi_width = 1920;
  pym_chn_attr.ds_info[1].roi_height = 1080;
  pym_chn_attr.ds_info[5].factor = 32;
  pym_chn_attr.ds_info[5].roi_x = 0;
  pym_chn_attr.ds_info[5].roi_y = 0;
  pym_chn_attr.ds_info[5].roi_width = 960;
  pym_chn_attr.ds_info[5].roi_height = 540;
  // pym_chn_attr.ds_info[6].factor = 32;
  // pym_chn_attr.ds_info[6].roi_x = 0;
  // pym_chn_attr.ds_info[6].roi_y = 0;
  // pym_chn_attr.ds_info[6].roi_width = 960;
  // pym_chn_attr.ds_info[6].roi_height = 540;
  ret = HB_VPS_SetPymChnAttr(group_id, 6, &pym_chn_attr);
  if (ret) {
    LOGE << "HB_VPS_SetPymChnAttr Failed. group:" << group_id
         << " ret = " << ret;
    return ret;
  }

  HB_VPS_EnableChn(group_id, 6);
  return ret;
}

int VpsModule::Input(void *data) {
  int ret = 0;
  hb_vio_buffer_t *hb_vio_buf = (hb_vio_buffer_t *)data;
  ret = HB_VPS_SendFrame(group_id_, hb_vio_buf, timeout_);
  if (ret != 0) {
    LOGE << "HB_VPS_SendFrame Failed. group:" << group_id_ << " ret = " << ret;
    return ret;
  }
  return ret;
}

int VpsModule::Output(void **data) {
  int ret = 0;
  uint32_t index = buffer_index_ % frameDepth_;
  ret = HB_VPS_GetChnFrame(group_id_, 6, &buffers_[index], timeout_);
  if (ret != 0) {
    LOGW << "HB_VPS_GetChnFrame Failed. group:" << group_id_
         << " ret = " << ret;
    data = nullptr;
    return ret;
  }
  buffer_index_++;
  *data = &buffers_[index];

  return ret;
}

int VpsModule::OutputBufferFree(void *data) {
  int ret = 0;
  if (data != nullptr) {
    ret = HB_VPS_ReleaseChnFrame(group_id_, 6, (pym_buffer_t *)data);
    if (ret != 0) {
      LOGE << "HB_VPS_ReleaseChnFrame Failed. group:" << group_id_
           << " ret = " << ret;
      return ret;
    }
    return ret;
  } else {
    return -1;
  }
}

int VpsModule::Start() {
  int ret = 0;
  ret = HB_VPS_StartGrp(group_id_);
  if (ret) {
    LOGE << "HB_VPS_StartGrp Failed. group:" << group_id_ << " ret = " << ret;
    return ret;
  }
  return ret;
}

int VpsModule::Stop() {
  int ret = 0;
  ret = HB_VPS_StopGrp(group_id_);
  if (ret) {
    LOGE << "HB_VPS_StopGrp Failed. group:" << group_id_ << " ret = " << ret;
    return ret;
  }
  ret = HB_VPS_DestroyGrp(group_id_);
  if (ret) {
    LOGE << "HB_VPS_DestroyGrp Failed. group:" << group_id_ << " ret = " << ret;
    return ret;
  }
  return ret;
}

int VpsModule::DeInit() {
  int ret = 0;
  ret = HB_VPS_DestroyGrp(group_id_);
  if (ret) {
    LOGE << "HB_VPS_DestroyGrp Failed. group:" << group_id_ << " ret = " << ret;
    return ret;
  }
  return ret;
}

}  // namespace vision
}  // namespace horizon