/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-16
 *  @Version: v0.0.1
 *  @Brief: implemenation of video process system.
 */

#include "video_source/vps/x3/vps_module_hapi.h"
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "hobotlog/hobotlog.hpp"
#include "utils/log.h"
#include "video_source/video_buffer/buffer_manager.h"
#include "video_source/vps/vps_data_type.h"

namespace videosource {

int VpsModuleHapi::GetVpsIpuChn(std::vector<int> &ipu_chn_list) {
  if (init_flag_ == false) {
    LOGE << "vps module has not init";
    return -1;
  }
  ipu_chn_list = vps_cfg_->chn_info_.ipu_valid_chn;
  return 0;
}

int VpsModuleHapi::GetVpsPymChn(std::vector<int> &pym_chn_list) {
  if (init_flag_ == false) {
    LOGE << "vps module has not init";
    return -1;
  }
  pym_chn_list = vps_cfg_->chn_info_.pym_valid_chn;
  return 0;
}

int VpsModuleHapi::GetConfig(std::shared_ptr<void> &output_cfg) {
  if (init_flag_ == false) {
    LOGE << "vps module has not init";
    return -1;
  }
  output_cfg = vps_cfg_;
  return 0;
}

void VpsModuleHapi::SetConfig(std::shared_ptr<void> &input_cfg) {
  VpsConfig* input = reinterpret_cast<VpsConfig*>(input_cfg.get());
  std::shared_ptr<VpsConfig> input_cfg_tmp(input);
  vps_cfg_ = input_cfg_tmp;
}

int VpsModuleHapi::Init(
    const std::vector<std::string> &config_file_list) {
  int ret = -1;
  bool find_vps_file = false;

  // static vps file config
  if (vps_input_width_ == 0 || vps_input_height_ == 0) {
    LOGE << "vps input size has not set, grp_id: " << group_id_;
    return -1;
  }
  for (auto vps_config_file : config_file_list) {
    ret = VpsLoadConfig(vps_config_file);
    if (ret) {
      LOGE << "load config failed, ret: " << ret;
      return ret;
    }
    if (vps_input_width_ == vps_cfg_->input_cfg_.group_w
        && vps_input_height_ == vps_cfg_->input_cfg_.group_h) {
      LOGI << "find vps config file: " << vps_config_file
        << " vps_input_width: " << vps_input_width_
        << " vps_input_height: " << vps_input_height_;
      find_vps_file = true;
      break;
    }
  }
  if (find_vps_file == false) {
    LOGE << "[ERROR] can not find vps file, "
      << " vin output size dismatch vps input size, please check"
      << " vps_input_width: " << vps_input_width_
      << " vps_input_width: " << vps_input_height_;
    return -1;
  }
  ret = HbCheckConfig(vps_cfg_);
  if (ret) {
    LOGE << "check vps config failed, ret: " << ret;
    return -1;
  }
  ret = VpsInit();
  if (ret) {
    LOGE << "vps init falied, ret: " << ret;
    return ret;
  }

  init_flag_ = true;
  LOGI << "VpsModuleHapi init success...";
  return 0;
}

int VpsModuleHapi::Init(std::shared_ptr<VpsConfig> &vps_cfg) {
  int ret = -1;

  // dynamic vps file config
  ret = HbCheckConfig(vps_cfg_);
  if (ret) {
    LOGE << "check vps config failed, ret: " << ret;
    return -1;
  }

  BufferManager &manager = BufferManager::Get();
  ret = manager.Init();
  if (ret) {
    LOGE << "buffer manager init failed, ret: " << ret;
    return ret;
  }
  vp_init_flag_ = true;

  vps_cfg_ = vps_cfg;
  ret = VpsInit();
  if (ret) {
    LOGE << "vps init falied, ret: " << ret;
    return ret;
  }

  init_flag_ = true;
  return 0;
}

int VpsModuleHapi::DeInit() {
  int ret = -1;

  ret = VpsDeInit();
  if (ret) {
    LOGE << "vps deinit falied, ret: " << ret;
    return ret;
  }

  if (vp_init_flag_ == true) {
    BufferManager &manager = BufferManager::Get();
    ret = manager.DeInit();
    if (ret) {
      LOGE << "buffer manager deinit failed, ret: " << ret;
      return ret;
    }
  }

  vps_input_width_ = 0;
  vps_input_height_ = 0;
  vps_sync_state_ = false;
  vp_init_flag_ = false;
  init_flag_ = false;
  LOGI << "VpsModuleHapi deinit success...";
  return 0;
}

int VpsModuleHapi::Start() {
  int ret = -1;

  ret = VpsStart();
  if (ret) {
    LOGE << "vps start falied, ret: " << ret;
    return ret;
  }

  start_flag_ = true;
  LOGI << "VpsModuleHapi start success...";
  return 0;
}

int VpsModuleHapi::Stop() {
  int ret = -1;

  start_flag_ = false;
  ret = VpsStop();
  if (ret) {
    LOGE << "vps stop falied, ret: " << ret;
    return ret;
  }

  LOGI << "VpsModuleHapi stop success...";
  return 0;
}

int VpsModuleHapi::RunVpsProcess(void *input_nv12) {
  int ret = -1;
  hb_vio_buffer_t *fb_buf =
    reinterpret_cast<hb_vio_buffer_t*>(input_nv12);
  HOBOT_CHECK(vps_cfg_);
  int timeout = vps_cfg_->pym_cfg_.timeout;

  uint32_t height = fb_buf->img_addr.height;
  uint32_t stride = fb_buf->img_addr.stride_size;
  uint32_t size_y = stride * height;
  uint32_t size_uv = size_y / 2;
  char *vir_ptr0 = fb_buf->img_addr.addr[0];
  uint32_t phy_ptr0 = fb_buf->img_addr.paddr[0];
  char *vir_ptr1 = fb_buf->img_addr.addr[1];
  uint32_t phy_ptr1 = fb_buf->img_addr.paddr[1];

  /* cache flush */
  BufferManager &manager = BufferManager::Get();
  bool vb_cache_en = manager.GetVbCacheEnable();
  if (vb_cache_en == true) {
    if (vir_ptr0) {
      ret = HB_SYS_CacheFlush(phy_ptr0, vir_ptr0, size_y);
      if (ret) {
        LOGE << "feedback pym process, cache flush y failed,"
          << " ret:" << ret;
        return ret;
      }
    }
    if (vir_ptr1) {
      ret = HB_SYS_CacheFlush(phy_ptr1, vir_ptr1, size_uv);
      if (ret) {
        LOGE << "feedback pym process, cache flush uv failed,"
          << " ret:" << ret;
        return ret;
      }
    }
  }

  /* send frame to vps module */
  ret = HB_VPS_SendFrame(group_id_, fb_buf, timeout);
  if (ret) {
      LOGE << " HB_VPS_SendFrame error, ret: " << ret;
    return ret;
  }
  // LOGW << "send frame to vps module success...";

  return 0;
}

int VpsModuleHapi::SendFrame(void *input_nv12) {
  int ret = -1;

  ret = RunVpsProcess(input_nv12);
  if (ret) {
    LOGE << "run vps process failed, ret: " << ret;
    return ret;
  }
  // image get pyramid data, use sync state
  if (vps_sync_state_ == true) {
    sem_post(&vps_sync_sem_);
  }

  return 0;
}

int VpsModuleHapi::GetFrame(int vps_chn, void *output_nv12) {
  int ret = -1;
  int grp_id = group_id_;

  std::vector<int> &ipu_valid_chn = vps_cfg_->chn_info_.ipu_valid_chn;
  hb_vio_buffer_t *ipu_buf =
    reinterpret_cast<hb_vio_buffer_t*>(output_nv12);
  size_t ipu_chn_num = ipu_valid_chn.size();
  if (ipu_chn_num <= 0 || ipu_chn_num >= MAX_CHN_NUM) {
      LOGE << " ipu chn num is invalid: " << ipu_chn_num
      << " group_id: " << grp_id;
    return -1;
  }

  // first ipu chn data
  int ipu_chn = ipu_valid_chn[0];
  int timeout = vps_cfg_->chn_cfg_[ipu_chn].ipu_frame_timeout;
  if (vps_chn == ipu_chn) {
    ret = HbTimeoutWait(ipu_sem_, timeout);
    if (ret < 0) {
      LOGE << "Wait Ipu Data Timeout!!!" << " group_id: " << grp_id;
      return ret;
    }
    VpsIpuInfo ipu_info = { 0 };
    auto rv = ipu_rq_->Pop(ipu_info);
    if (!rv) {
      LOGE << "No Ipu info in RingQueue!" << " group_id: " << grp_id;
      return -1;
    }
    *ipu_buf = ipu_info.ipu_buf;
    return 0;
  }

  // other ipu chn data
  for (size_t i = 1; i < ipu_chn_num; i++) {
    int ipu_chn = ipu_valid_chn[i];
    if (ipu_chn != vps_chn) continue;
    int timeout = vps_cfg_->chn_cfg_[ipu_chn].ipu_frame_timeout;
    ret = HB_VPS_GetChnFrame(grp_id, ipu_chn, ipu_buf, timeout);
    if (ret) {
      if (start_flag_ == true) {
        LOGE << " HB_VPS_GetChnFrame error, ret: " << ret
          << " ipu_chn_num: " << ipu_chn_num
          << " grp_id: " << grp_id
          << " ipu_chn: " << ipu_chn
          << " timeout: " << timeout;
      }
      return ret;
    }
    LOGD << "vps get ipu chn frame success, "
      << " group_id: " << grp_id
      << " ipu_chn: " << ipu_chn
      << " frame_id: " << ipu_buf->img_info.frame_id
      << " time_stamp: " << ipu_buf->img_info.time_stamp;
    static int count = 0;
    bool dump_en = false;
    int dump_num = 0;
    GetDumpNum("./vps_output.txt", dump_en, dump_num);
    if (dump_en == true && (count < dump_num || dump_num < 0)) {
      HbDumpIpuData(grp_id, ipu_chn, ipu_buf);
    }
  }
  return 0;
}

int VpsModuleHapi::FreeFrame(int vps_chn, void *input_nv12) {
  int ret = -1;
  int grp_id = group_id_;
  std::vector<int> &ipu_valid_chn = vps_cfg_->chn_info_.ipu_valid_chn;
  size_t ipu_chn_num = ipu_valid_chn.size();
  if (ipu_chn_num <= 0 || ipu_chn_num >= MAX_CHN_NUM) {
      LOGE << " ipu chn num is invalid: " << ipu_chn_num
      << " group_id: " << grp_id;
    return -1;
  }

  for (size_t i = 0; i < ipu_chn_num; i++) {
    int ipu_chn = ipu_valid_chn[i];
    if (ipu_chn != vps_chn) continue;
    ret = HB_VPS_ReleaseChnFrame(grp_id, ipu_chn, input_nv12);
    if (ret != 0) {
      LOGE << " HB_VPS_ReleaseChnFrame error, ret: " << ret;
      return ret;
    }
  }

  return 0;
}

int VpsModuleHapi::GetImageBuffer(
    std::shared_ptr<ImageFrame> &output_buf_info) {
  int ret = -1;
  if (init_flag_ == false) {
    LOGE << "vps has not init!";
    return -1;
  }
  int width = vps_cfg_->input_cfg_.group_w;
  int height = vps_cfg_->input_cfg_.group_h;
  int stride = width;
  if (width <= 0 || width > 4096 || height <= 0 || width > 4096) {
    LOGE << "width: " << width << " height: " << height
      << " is error!!!";
    return -1;
  }

  if (vp_init_flag_ == false) {
    LOGE << "vp has not init";
    return -1;
  }
  BufferManager &manager = BufferManager::Get();
  ret = manager.GetImageBuffer(width, height, stride, output_buf_info);
  if (ret) {
    LOGE << "get image buffer error, ret: " << ret;
    return ret;
  }

  return 0;
}

int VpsModuleHapi::GetPyramidFrame(int vps_chn,
    void *output_src, void *output_pym) {
  int ret = -1;
  int chret = -1;
  int grp_id = group_id_;
  size_t pym_chn_num = vps_cfg_->chn_info_.pym_valid_chn.size();
  if (pym_chn_num == 0) {
    LOGW << "pyramid chn has not enable, pym_chn_num: " << pym_chn_num;
    return 0;
  }
  int pym_chn = vps_cfg_->chn_info_.pym_valid_chn[0];
  int vps_layer_dump = vps_cfg_->debug_cfg_.vps_layer_dump;
  VpsPymInfo pym_info = { 0 };

  pym_buffer_t *pym_buf = reinterpret_cast<pym_buffer_t *>(output_pym);
  uint64_t timeout = static_cast<uint64_t>(vps_cfg_->pym_cfg_.timeout);

  ret = HbTimeoutWait(pym_sem_, timeout);
  if (ret < 0) {
    LOGE << "Wait Pym Data Timeout!!!" << " group_id: " << grp_id;
    return ret;
  }
  auto rv = pym_rq_->Pop(pym_info);
  if (!rv) {
    LOGE << "No Pym info in RingQueue!" << " group_id: " << grp_id;
    return -1;
  }
  *pym_buf = pym_info.pym_buf;
  HbAllocPymBuffer();
  int ds_layer_num = vps_cfg_->pym_cfg_.ds_layer_en;
  chret = HbCheckPymData(pym_info, ds_layer_num);
  if (chret != 0) {
    LOGE << " vio check pym error chret: " << chret;
    chret = HB_VPS_ReleaseChnFrame(pym_info.grp_id, pym_info.pym_chn,
        &pym_info.pym_buf);
    if (chret != 0) {
       LOGE << " HB_VPS_ReleaseChnFrame error, ret: " << ret;
      return chret;
    }
    HbFreePymBuffer();
    return chret;
  }
  // dump pramid frame
  {
    static int count = 0;
    bool dump_en = false;
    int dump_num = 0;
    GetDumpNum("./vps_output.txt", dump_en, dump_num);
    if (vps_dump_num_-- > 0
        || (dump_en == true && (count < dump_num || dump_num < 0))) {
      if (vps_layer_dump == MAX_PYM_DS_NUM) {
        // dump all pym base layer
        HbDumpPymData(grp_id, pym_chn, pym_buf);
      }
      if (vps_layer_dump >=0 && vps_layer_dump < MAX_PYM_DS_NUM) {
        // dump any one pym layer(including base and roi layer)
        HbDumpPymLayerData(grp_id, pym_chn, vps_layer_dump, pym_buf);
      }
    }
  }
  return 0;
}

int VpsModuleHapi::FreePyramidFrame(int vps_chn,
    void *input_src, void *input_pym) {
  int ret = -1;
  int grp_id = group_id_;
  size_t pym_chn_num = vps_cfg_->chn_info_.pym_valid_chn.size();
  if (pym_chn_num == 0) {
    LOGW << "pyramid chn has not enable, pym_chn_num: " << pym_chn_num;
    return 0;
  }

  ret = HB_VPS_ReleaseChnFrame(grp_id, vps_chn, input_pym);
  if (ret != 0) {
    LOGE << " HB_VPS_ReleaseChnFrame error, ret: " << ret;
    return ret;
  }
  HbFreePymBuffer();
  return 0;
}

int VpsModuleHapi::HbChnInfoInit() {
  int grp_id = group_id_;
  int ipu_chn_index, pym_chn_index;
  int ipu_chn_en, pym_chn_en;

  ipu_chn_index = 0;
  pym_chn_index = 0;
  for (int chn_idx = 0; chn_idx < MAX_CHN_NUM; chn_idx++) {
    ipu_chn_en = vps_cfg_->chn_cfg_[chn_idx].ipu_chn_en;
    pym_chn_en = vps_cfg_->chn_cfg_[chn_idx].pym_chn_en;

    if (pym_chn_en == 1) {
      vps_cfg_->chn_info_.pym_valid_chn.push_back(chn_idx);
      pym_chn_index++;
      LOGI << "group_id: " << grp_id
        << " pym_chn_index: " << pym_chn_index
        << " pym_chn_valid: " << chn_idx;
      // if pym chn enable, vps can not get ipu chn data
      continue;
    }
    if (ipu_chn_en == 1) {
      vps_cfg_->chn_info_.ipu_valid_chn.push_back(chn_idx);
      ipu_chn_index++;
      LOGI << "group_id: " << grp_id
        << " ipu_chn_index: " << ipu_chn_index
        << " ipu_chn_valid: " << chn_idx;
    }
  }
  HOBOT_CHECK(pym_chn_index <= 1) << "a group only support max a pym chn";
  HOBOT_CHECK(ipu_chn_index > 0 || pym_chn_index > 0)
    << " ipu chn and pym chn not enable";
  return 0;
}

int VpsModuleHapi::VpsLoadConfig(const std::string &input_config_file) {
  int ret = -1;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;

  ret = LoadConfigFile(input_config_file, json_cfg);
  if (ret) {
    LOGE << "load config file failed, ret: " << ret;
    return ret;
  }

  vps_cfg_ = std::make_shared<VpsConfig>();
  // 1. parse debug config info
  auto json_cfg_debug = json_cfg->GetSubConfig("debug");
  if (json_cfg_debug == nullptr) {
    LOGW << "vps json cfg debug parameter is not exit!";
  } else {
    vps_cfg_->debug_cfg_.vps_dump_num =
      json_cfg_debug->GetIntValue("vps_dump_num");
    vps_cfg_->debug_cfg_.vps_layer_dump =
      json_cfg_debug->GetIntValue("vps_layer_dump");
  }

  // 2. parse input config info
  auto json_cfg_input = json_cfg->GetSubConfig("input");
  if (json_cfg_input == nullptr) {
    LOGE << "vps json cfg input parameter is not exit!";
    return -1;
  } else {
    vps_cfg_->input_cfg_.group_w =
      json_cfg_input->GetIntValue("width");
    vps_cfg_->input_cfg_.group_h =
      json_cfg_input->GetIntValue("height");
  }

  // 3. parse gdc config info
  auto json_cfg_gdc_total = json_cfg->GetSubConfig("gdc");
  if (json_cfg_gdc_total == nullptr) {
    LOGE << "vps json cfg gdc parameter is not exit!";
    return -1;
  }
  for (int i = 0; i < MAX_GDC_NUM; i++) {
    std::string gdc_name = "gdc" + std::to_string(i);
    auto json_cfg_gdc = json_cfg_gdc_total->GetSubConfig(gdc_name);
    if (json_cfg_gdc == nullptr) continue;
    vps_cfg_->gdc_cfg_[i].gdc_en =
      json_cfg_gdc->GetIntValue("enable");
    vps_cfg_->gdc_cfg_[i].gdc_type =
      json_cfg_gdc->GetIntValue("gdc_type");
    vps_cfg_->gdc_cfg_[i].frame_depth =
      json_cfg_gdc->GetIntValue("frame_depth");
    vps_cfg_->gdc_cfg_[i].rotate =
      json_cfg_gdc->GetIntValue("rotate");
    vps_cfg_->gdc_cfg_[i].bind_ipu_chn =
      json_cfg_gdc->GetIntValue("bind_ipu_chn");
    vps_cfg_->gdc_cfg_[i].file_path =
      json_cfg_gdc->GetSTDStringValue("path");
    vps_cfg_->gdc_cfg_[i].gdc_w =
      vps_cfg_->input_cfg_.group_w;
    vps_cfg_->gdc_cfg_[i].gdc_h =
      vps_cfg_->input_cfg_.group_h;
  }
  // 4. parse ipu config info
  auto json_cfg_ipu = json_cfg->GetSubConfig("ipu");
  CHECK_PARAMS_VALID(json_cfg_ipu);
  for (int i = 0; i < MAX_CHN_NUM; i++) {
    std::string chn_name = "chn" + std::to_string(i);
    auto json_cfg_chn = json_cfg_ipu->GetSubConfig(chn_name);
    if (json_cfg_chn == nullptr) continue;
    vps_cfg_->chn_cfg_[i].ipu_chn_en =
      json_cfg_chn->GetIntValue("ipu_chn_en");
    vps_cfg_->chn_cfg_[i].pym_chn_en =
      json_cfg_chn->GetIntValue("pym_chn_en");
    vps_cfg_->chn_cfg_[i].ipu_frame_depth =
      json_cfg_chn->GetIntValue("frame_depth");
    vps_cfg_->chn_cfg_[i].ipu_frame_timeout =
      json_cfg_chn->GetIntValue("timeout");
    // 4.1 ipu roi info
    vps_cfg_->chn_cfg_[i].ipu_roi.roi_en =
      json_cfg_chn->GetIntValue("roi_en");
    vps_cfg_->chn_cfg_[i].ipu_roi.roi_x =
      json_cfg_chn->GetIntValue("roi_x");
    vps_cfg_->chn_cfg_[i].ipu_roi.roi_y =
      json_cfg_chn->GetIntValue("roi_y");
    vps_cfg_->chn_cfg_[i].ipu_roi.roi_w =
      json_cfg_chn->GetIntValue("roi_w");
    vps_cfg_->chn_cfg_[i].ipu_roi.roi_h =
      json_cfg_chn->GetIntValue("roi_h");
    // 4.2 ipu scale info
    vps_cfg_->chn_cfg_[i].ipu_scale.scale_w =
      json_cfg_chn->GetIntValue("scale_w");
    vps_cfg_->chn_cfg_[i].ipu_scale.scale_h =
      json_cfg_chn->GetIntValue("scale_h");
  }

  // 5. parse pyramid config info
  auto json_cfg_pym = json_cfg->GetSubConfig("pym");
  CHECK_PARAMS_VALID(json_cfg_pym);
  // 5.1 parse pyramid ctrl config info
  auto json_cfg_pym_ctrl = json_cfg_pym->GetSubConfig("pym_ctrl_config");
  CHECK_PARAMS_VALID(json_cfg_pym_ctrl);
  vps_cfg_->pym_cfg_.frame_id =
    json_cfg_pym_ctrl->GetIntValue("frame_id");
  vps_cfg_->pym_cfg_.ds_layer_en =
    json_cfg_pym_ctrl->GetIntValue("ds_layer_en");
  vps_cfg_->pym_cfg_.ds_uv_bypass =
    json_cfg_pym_ctrl->GetIntValue("ds_uv_bypass");
  vps_cfg_->pym_cfg_.us_layer_en =
    json_cfg_pym_ctrl->GetIntValue("us_layer_en");
  vps_cfg_->pym_cfg_.us_uv_bypass =
    json_cfg_pym_ctrl->GetIntValue("us_uv_bypass");
  vps_cfg_->pym_cfg_.frameDepth =
    json_cfg_pym_ctrl->GetIntValue("frame_depth");
  vps_cfg_->pym_cfg_.timeout =
    json_cfg_pym_ctrl->GetIntValue("timeout");
  // 5.2 parse pyramid downscale config info
  auto json_cfg_pym_ds = json_cfg_pym->GetSubConfig("pym_ds_config");
  CHECK_PARAMS_VALID(json_cfg_pym_ds);
  for (int ds_idx = 0 ; ds_idx < MAX_PYM_DS_NUM; ds_idx++) {
    if (ds_idx % 4 == 0) continue;
    std::string factor_name = "factor_" + std::to_string(ds_idx);
    std::string roi_x_name = "roi_x_" + std::to_string(ds_idx);
    std::string roi_y_name = "roi_y_" + std::to_string(ds_idx);
    std::string roi_w_name = "roi_w_" + std::to_string(ds_idx);
    std::string roi_h_name = "roi_h_" + std::to_string(ds_idx);
    vps_cfg_->pym_cfg_.ds_info[ds_idx].factor =
      json_cfg_pym_ds->GetIntValue(factor_name);
    vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_x =
      json_cfg_pym_ds->GetIntValue(roi_x_name);
    vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_y =
      json_cfg_pym_ds->GetIntValue(roi_y_name);
    vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_width =
      json_cfg_pym_ds->GetIntValue(roi_w_name);
    vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_height =
      json_cfg_pym_ds->GetIntValue(roi_h_name);
  }
  // 5.3 parse pyramid upscale config info
  auto json_cfg_pym_us = json_cfg_pym->GetSubConfig("pym_us_config");
  CHECK_PARAMS_VALID(json_cfg_pym_us);
  for (int us_idx = 0 ; us_idx < MAX_PYM_US_NUM; us_idx++) {
    std::string factor_name = "factor_" + std::to_string(us_idx);
    std::string roi_x_name = "roi_x_" + std::to_string(us_idx);
    std::string roi_y_name = "roi_y_" + std::to_string(us_idx);
    std::string roi_w_name = "roi_w_" + std::to_string(us_idx);
    std::string roi_h_name = "roi_h_" + std::to_string(us_idx);
    vps_cfg_->pym_cfg_.us_info[us_idx].factor =
      json_cfg_pym_us->GetIntValue(factor_name);
    vps_cfg_->pym_cfg_.us_info[us_idx].roi_x =
      json_cfg_pym_us->GetIntValue(roi_x_name);
    vps_cfg_->pym_cfg_.us_info[us_idx].roi_y =
      json_cfg_pym_us->GetIntValue(roi_y_name);
    vps_cfg_->pym_cfg_.us_info[us_idx].roi_width =
      json_cfg_pym_us->GetIntValue(roi_w_name);
    vps_cfg_->pym_cfg_.us_info[us_idx].roi_height =
      json_cfg_pym_us->GetIntValue(roi_h_name);
  }

  return 0;
}

int VpsModuleHapi::HbCheckConfig(std::shared_ptr<VpsConfig> &vps_cfg) {
  if (vps_cfg == nullptr) {
    LOGE << "vps config is nullptr";
    return -1;
  }
  LOGI << "******** vps config group_id: " << group_id_ << " start *********";
  // 1. vps debug config
  LOGI << "vps_dump_num: "    << vps_cfg_->debug_cfg_.vps_dump_num;
  LOGI << "vps_layer_dump: "  << vps_cfg_->debug_cfg_.vps_layer_dump;

  // 2. vps input config
  LOGI << "vps_input_width: "   << vps_cfg_->input_cfg_.group_w;
  LOGI << "vps_input_height: "  << vps_cfg_->input_cfg_.group_h;

  // 3. gdc config
  for (int gdc_idx = 0; gdc_idx < MAX_GDC_NUM; gdc_idx++) {
    LOGI << "gdc_" << gdc_idx << "_en: "
      << vps_cfg_->gdc_cfg_[gdc_idx].gdc_en;
    LOGI << "gdc_" << gdc_idx << "_frame_depth: "
      << vps_cfg_->gdc_cfg_[gdc_idx].frame_depth;
    LOGI << "gdc_" << gdc_idx << "_src_type: "
      << vps_cfg_->gdc_cfg_[gdc_idx].gdc_type;
    LOGI << "gdc_" << gdc_idx << "_rotate: "
      << vps_cfg_->gdc_cfg_[gdc_idx].rotate;
    LOGI << "gdc_" << gdc_idx << "_bind_ipu_chn: "
      << vps_cfg_->gdc_cfg_[gdc_idx].bind_ipu_chn;
    LOGI << "gdc_" << gdc_idx << "_file_path: "
      << vps_cfg_->gdc_cfg_[gdc_idx].file_path;
  }
  // 4. chn config
  for (int chn_idx = 0 ; chn_idx < MAX_CHN_NUM; chn_idx++) {
    LOGI << "chn_index: "<< chn_idx << " ipu_chn_en: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_chn_en;
    LOGI << "chn_index: "<< chn_idx << " pym_chn_en: "
      << vps_cfg_->chn_cfg_[chn_idx].pym_chn_en;
    LOGI << "chn_index: "<< chn_idx << " ipu_frame_depth: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_frame_depth;
    LOGI << "chn_index: "<< chn_idx << " ipu_frame_timeout: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_frame_timeout;
    // 4.1 ipu chn scale config
    LOGI << "chn_index: "<< chn_idx << " scale_w: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_scale.scale_w;
    LOGI << "chn_index: "<< chn_idx << " scale_h: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_scale.scale_h;
    // 4.2 ipu chn roi config
    LOGI << "chn_index: "<< chn_idx << " roi_en: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_en;
    LOGI << "chn_index: "<< chn_idx << " roi_x: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_x;
    LOGI << "chn_index: "<< chn_idx << " roi_y: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_y;
    LOGI << "chn_index: "<< chn_idx << " roi_w: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_w;
    LOGI << "chn_index: "<< chn_idx << " roi_h: "
      << vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_h;
  }
  // 5. pym config
  LOGI << "--------pym config start---------";
  LOGI << "frame_id: "     << vps_cfg_->pym_cfg_.frame_id;
  LOGI << "ds_layer_en: "  << vps_cfg_->pym_cfg_.ds_layer_en;
  LOGI << "ds_uv_bypass: " << vps_cfg_->pym_cfg_.ds_uv_bypass;
  LOGI << "us_layer_en: "
    << static_cast<int>(vps_cfg_->pym_cfg_.us_layer_en);
  LOGI << "us_uv_bypass: "
    << static_cast<int>(vps_cfg_->pym_cfg_.us_uv_bypass);
  LOGI << "frameDepth: " << vps_cfg_->pym_cfg_.frameDepth;
  LOGI << "timeout: "    << vps_cfg_->pym_cfg_.timeout;
  // 5.2 pym downscale config
  for (int ds_idx = 0 ; ds_idx < MAX_PYM_DS_NUM; ds_idx++) {
    if (ds_idx % 4 == 0) continue;
    LOGI << "ds_pym_layer: "<< ds_idx << " " << "ds roi_x: "
      << vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_x;
    LOGI << "ds_pym_layer: "<< ds_idx << " " << "ds roi_y: "
      << vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_y;
    LOGI << "ds_pym_layer: "<< ds_idx << " " << "ds roi_width: "
      << vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_width;
    LOGI << "ds_pym_layer: "<< ds_idx << " " << "ds roi_height: "
      << vps_cfg_->pym_cfg_.ds_info[ds_idx].roi_height;
    LOGI << "ds_pym_layer: "<< ds_idx << " " << "ds factor: "
      << static_cast<int>(vps_cfg_->pym_cfg_.ds_info[ds_idx].factor);
  }
  /* 4.3 pym upscale config */
  for (int us_idx = 0 ; us_idx < MAX_PYM_US_NUM; us_idx++) {
    LOGI << "us_pym_layer: "<< us_idx << " " << "us roi_x: "
      << vps_cfg_->pym_cfg_.us_info[us_idx].roi_x;
    LOGI << "us_pym_layer: "<< us_idx << " " << "us roi_y: "
      << vps_cfg_->pym_cfg_.us_info[us_idx].roi_y;
    LOGI << "us_pym_layer: "<< us_idx << " " << "us roi_width: "
      << vps_cfg_->pym_cfg_.us_info[us_idx].roi_width;
    LOGI << "us_pym_layer: "<< us_idx << " " << "us roi_height: "
      << vps_cfg_->pym_cfg_.us_info[us_idx].roi_height;
    LOGI << "us_pym_layer: "<< us_idx << " " << "us factor: "
      << static_cast<int>(vps_cfg_->pym_cfg_.us_info[us_idx].factor);
  }
  LOGI << "---------pym config end----------";

  HbChnInfoInit();
  LOGI << "******** vps config: " << group_id_ << " end *********";

  return 0;
}

int VpsModuleHapi::HbGetVpsFrameDepth() {
  int frame_depth = 2;  // default gdc buffer num
  VpsGdcInfo gdc_cfg = { 0 };
  VpsGdcType gdc_src_type = kGDC_TYPE_INVALID;

  for (int gdc_idx = 0; gdc_idx < MAX_GDC_NUM; gdc_idx++) {
    gdc_cfg = vps_cfg_->gdc_cfg_[gdc_idx];
    gdc_src_type = static_cast<VpsGdcType>(gdc_cfg.gdc_type);
    if (gdc_cfg.gdc_en == 1) {
      if (gdc_src_type == kISP_DDR_GDC ||
          gdc_src_type == kIPU_CHN_DDR_GDC) {
        frame_depth = gdc_cfg.frame_depth;
        break;
      }
    }
  }

  // vps input frame depth is used by gdc module
  vps_cfg_->input_cfg_.frame_depth = frame_depth;
  return frame_depth;
}

int VpsModuleHapi::HbGetGdcData(const char *gdc_name,
    char **gdc_data, int *gdc_len) {
  LOGD << "start to set GDC";
  std::ifstream ifs(gdc_name);
  if (!ifs.is_open()) {
    LOGE << "GDC file open failed!";
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  auto len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  auto buf = new char[len];
  ifs.read(buf, len);

  *gdc_data = buf;
  *gdc_len = len;

  return 0;
}

int VpsModuleHapi::HbSetGdcInfo(int gdc_idx, int group_id,
    const VpsGdcInfo &info) {
  int ret = 0;
  char *gdc_data = nullptr;
  int gdc_group_id = group_id;
  int gdc_len = 0;
  int gdc_w = info.gdc_w;
  int gdc_h = info.gdc_h;
  VpsGdcType gdc_src_type = static_cast<VpsGdcType>(info.gdc_type);
  int gdc_frame_depth = info.frame_depth;
  int gdc_chn = info.bind_ipu_chn;
  const char *gdc_path = info.file_path.c_str();
  ROTATION_E gdc_rotate = static_cast<ROTATION_E>(info.rotate);

  LOGD << "gdc_idx: " << gdc_idx
    << " gdc_src_type: " << gdc_src_type
    << " gdc_rotate: " << gdc_rotate
    << " gdc_path: " << gdc_path;

  ret = HbGetGdcData(gdc_path, &gdc_data, &gdc_len);
  if (ret) {
    LOGE << "Hb get gdc data failed!";
    return -1;
  }
  LOGD << "gdc_data addr: " << reinterpret_cast<void *>(gdc_data)
    << " gdc_len: " << gdc_len;

  switch (gdc_src_type) {
    case kISP_DDR_GDC: {
      ret = HB_VPS_SetGrpGdc(gdc_group_id, gdc_data, gdc_len, gdc_rotate);
      if (ret) {
        LOGE << "HB_VPS_SetGrpGdc error!!!";
      } else {
        LOGI << "HB_VPS_SetGrpGdc ok: gdc_group_id = " << gdc_group_id;
      }
      break;
    }
    case kIPU_CHN_DDR_GDC: {
      ret =  HB_VPS_SetChnGdc(gdc_group_id, gdc_chn, gdc_data, gdc_len,
          gdc_rotate);
      if (ret) {
        LOGE << "HB_VPS_SetChnGdc error!!!";
      } else {
        LOGI << "HB_VPS_SetChnGdc ok, gdc_group_id: " << gdc_group_id;
      }
      break;
    }
    case kPYM_DDR_GDC: {
      gdc_group_id_[gdc_idx] = GDC_PIPE_ID + gdc_idx;
      gdc_group_id = gdc_group_id_[gdc_idx];
      ret = HbVpsCreateGrp(gdc_group_id, gdc_w, gdc_h, gdc_frame_depth);
      if (ret) {
        LOGE << "HbVpsCreateGrp failed!!!";
      }

      ret = HB_VPS_SetGrpGdc(group_id, gdc_data, gdc_len, gdc_rotate);
      if (ret) {
        LOGE << "HB_VPS_SetGrpGdc error!!!";
      } else {
        LOGI << "HB_VPS_SetGrpGdc ok: gdc group_id = " << gdc_group_id;
      }
      break;
    }
    default:
      LOGE << "not support gdc type:" << gdc_src_type;
      break;
  }
  free(gdc_data);

  return ret;
}

int VpsModuleHapi::SetGdcInfo(VpsGdcType type) {
  int ret = -1;
  int gdc_group_id = group_id_;
  VpsGdcInfo gdc_cfg = { 0 };

  /* set gdc */
  for (int gdc_idx = 0; gdc_idx < MAX_GDC_NUM; gdc_idx++) {
    gdc_cfg = vps_cfg_->gdc_cfg_[gdc_idx];
    VpsGdcType gdc_type = static_cast<VpsGdcType>(gdc_cfg.gdc_type);
    if (gdc_cfg.gdc_en == 1 && gdc_type == type) {
      ret = HbSetGdcInfo(gdc_idx, gdc_group_id, gdc_cfg);
      if (ret) {
        LOGE << "Set GDC failed!, index: " << gdc_idx << "ret: " << ret;
        return ret;
      }
    }
  }

  return 0;
}

int VpsModuleHapi::GetGdcInfo(void *buf) {
  int ret = -1;
  VpsGdcInfo gdc_cfg = { 0 };
  int gdc_group_id = -1;
  int gdc_chn = -1;
  int timeout = 2000;
  VpsGdcType gdc_src_type = kGDC_TYPE_INVALID;

  hb_vio_buffer_t *gdc_buf = reinterpret_cast<hb_vio_buffer_t*>(buf);
  /* get gdc info */
  for (int gdc_idx = 0; gdc_idx < MAX_GDC_NUM; gdc_idx++) {
    gdc_cfg = vps_cfg_->gdc_cfg_[gdc_idx];
    gdc_src_type = static_cast<VpsGdcType>(gdc_cfg.gdc_type);
    if (gdc_cfg.gdc_en == 1 && gdc_src_type == kPYM_DDR_GDC) {
      gdc_group_id = GDC_PIPE_ID + gdc_idx;
      gdc_chn = 0;
      ret = HB_VPS_GetChnFrame(gdc_group_id, gdc_chn, gdc_buf, timeout);
      if (ret) {
        LOGE << "Get Chn GDC frame failed!, index: "
          << gdc_idx << "ret: " << ret;
        return ret;
      }
    }
  }

  return 0;
}

int VpsModuleHapi::VpsCreateGetDataThread() {
  LOGI << "Enter VpsCreateGetDataThread,group_id: " << group_id_
    << " start_flag: " << pym_start_flag_;

  // create get pym thread
  size_t pym_chn_num = vps_cfg_->chn_info_.pym_valid_chn.size();
  if (pym_chn_num == 0) {
    LOGW << "pyramid chn has not enable, pym thread will not create";
  } else {
    if (pym_start_flag_ == false && pym_thread_ == nullptr) {
      pym_thread_ = std::make_shared<std::thread>(
          &VpsModuleHapi::HbGetPymDataThread, this);
      pym_start_flag_ = true;
    }
  }

  // create get ipu thread if no pym thread
  std::vector<int> &ipu_valid_chn = vps_cfg_->chn_info_.ipu_valid_chn;
  size_t ipu_chn_num = ipu_valid_chn.size();
  if (ipu_chn_num <= 0 || ipu_chn_num >= MAX_CHN_NUM) {
      LOGW << " ipu output disable, ipu_chn_num: " << ipu_chn_num
      << " group_id: " << group_id_;
  } else {
    if (ipu_start_flag_ == false && ipu_thread_ == nullptr) {
      ipu_thread_ = std::make_shared<std::thread>(
          &VpsModuleHapi::HbGetIpuDataThread, this);
      ipu_start_flag_ = true;
    }
  }
  return 0;
}

int VpsModuleHapi::VpsDestoryGetDataThread() {
  LOGI << "Enter VpsDestoryGetDataThread,group_id: " << group_id_
    << " start_flag: " << pym_start_flag_;

  // destory pym thread
  if (pym_start_flag_ == true && pym_thread_ != nullptr) {
    pym_start_flag_ = false;
    pym_thread_->join();
    pym_thread_ = nullptr;
  }

  // destory ipu thread
  if (ipu_start_flag_ == true && ipu_thread_ != nullptr) {
    ipu_start_flag_ = false;
    ipu_thread_->join();
    ipu_thread_ = nullptr;
  }
  LOGI << "Quit VpsDestoryDataThread, group_id: " << group_id_;
  return 0;
}

int VpsModuleHapi::HbVpsCreateGrp(int group_id, int grp_w, int grp_h,
    int grp_depth) {
  int ret = -1;
  VPS_GRP_ATTR_S grp_attr;

  memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
  grp_attr.maxW = grp_w;
  grp_attr.maxH = grp_h;
  grp_attr.frameDepth = grp_depth;
  LOGD << "vps create group, grp_w: " << grp_w
    << " grp_h: " << grp_h << " grp_depth: " << grp_depth;
  ret = HB_VPS_CreateGrp(group_id, &grp_attr);
  if (ret) {
    LOGE << "HB_VPS_CreateGrp error!!!";
    return ret;
  } else {
    LOGI << "created a group ok:GrpId: " << group_id;
  }

  return 0;
}

int VpsModuleHapi::VpsInit() {
  int ret = -1;
  int group_id = group_id_;
  int grp_width, grp_height, grp_frame_depth;
  int ipu_chn_en, pym_chn_en, ipu_frame_depth;
  int scale_en, scale_w, scale_h;
  int roi_en, roi_x, roi_y, roi_w, roi_h;
  int ds2_roi_en, ds2_roi_x, ds2_roi_y, ds2_roi_w, ds2_roi_h;
  int ipu_ds2_en = 0;
  int ipu_ds2_crop_en = 0;
  VPS_CHN_ATTR_S chn_attr;
  VPS_PYM_CHN_ATTR_S pym_chn_attr;

  LOGI << "Enter vpsmodule init," << " group_id: " << group_id_;
  vps_dump_num_ = vps_cfg_->debug_cfg_.vps_dump_num;
  grp_width = vps_cfg_->input_cfg_.group_w;
  grp_height = vps_cfg_->input_cfg_.group_h;
  HOBOT_CHECK(grp_width > 0) << "group width is invalid: " << grp_width;
  HOBOT_CHECK(grp_height > 0) << "group height is invalid: " << grp_height;
  grp_frame_depth = HbGetVpsFrameDepth();
  ret = HbVpsCreateGrp(group_id, grp_width, grp_height, grp_frame_depth);
  HOBOT_CHECK(ret == 0) << "HbVpsCreateGrp failed!";
  ret = SetGdcInfo(kISP_DDR_GDC);
  HOBOT_CHECK(ret == 0) << "set group gdc failed!";

  for (int chn_idx = 0; chn_idx < MAX_CHN_NUM; chn_idx++) {
    /* 1. set ipu chn */
    ipu_chn_en = vps_cfg_->chn_cfg_[chn_idx].ipu_chn_en;
    pym_chn_en = vps_cfg_->chn_cfg_[chn_idx].pym_chn_en;
    ipu_frame_depth = vps_cfg_->chn_cfg_[chn_idx].ipu_frame_depth;
    roi_en = vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_en;
    roi_x = vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_x;
    roi_y = vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_y;
    roi_w = vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_w;
    roi_h = vps_cfg_->chn_cfg_[chn_idx].ipu_roi.roi_h;
    scale_w = vps_cfg_->chn_cfg_[chn_idx].ipu_scale.scale_w;
    scale_h = vps_cfg_->chn_cfg_[chn_idx].ipu_scale.scale_h;
    if (chn_idx == 2) {
      ds2_roi_en = roi_en;
      ds2_roi_x = roi_x;
      ds2_roi_y = roi_y;
      ds2_roi_w = roi_w;
      ds2_roi_y = roi_h;
    }

    /* set ipu chn */
    if (ipu_chn_en == 1) {
      memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
      vps_cfg_->chn_cfg_[chn_idx].ipu_scale.scale_en = 1;
      scale_en = 1;
      chn_attr.enScale = scale_en;
      chn_attr.width = scale_w;
      chn_attr.height = scale_h;
      chn_attr.frameDepth = ipu_frame_depth;
      ret = HB_VPS_SetChnAttr(group_id, chn_idx, &chn_attr);
      if (ret) {
        LOGE << "HB_VPS_SetChnAttr error!!!";
        return ret;
      } else {
        LOGI << "vps set ipu chn Attr ok: GrpId: " << group_id
          << " chn_id: " << chn_idx;
      }
      if (roi_en == 1) {
        int crop_chn = chn_idx;
        if (chn_idx == 6 && ipu_ds2_en == 0) {
          /* set ipu chn2 attr */
          crop_chn = 2;
          memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
          chn_attr.enScale = scale_en;
          chn_attr.width = scale_w;
          chn_attr.height = scale_h;
          chn_attr.frameDepth = 0;
          ret = HB_VPS_SetChnAttr(group_id, crop_chn, &chn_attr);
          if (ret) {
            LOGE << "HB_VPS_SetChnAttr error!!!";
            return ret;
          } else {
            LOGI << "vps set ipu chn Attr ok: GrpId: " << group_id
              << " chn_id: " << crop_chn;
          }
        }
        if (chn_idx == 6 && ipu_ds2_crop_en == 1) {
          LOGW << "ipu chn6 is not need set crop, because chn2 has been"
            << " crop(chn6 and chn2 is shared crop chn settings)";
          HOBOT_CHECK(ds2_roi_en == roi_en);
          HOBOT_CHECK(ds2_roi_x == roi_x);
          HOBOT_CHECK(ds2_roi_y == roi_y);
          HOBOT_CHECK(ds2_roi_w == roi_w);
          HOBOT_CHECK(ds2_roi_h == roi_h);
        } else {
          VPS_CROP_INFO_S crop_info;
          memset(&crop_info, 0, sizeof(VPS_CROP_INFO_S));
          crop_info.en = roi_en;
          crop_info.cropRect.width = roi_w;
          crop_info.cropRect.height = roi_h;
          crop_info.cropRect.x = roi_x;
          crop_info.cropRect.y = roi_y;
          LOGI << "crop en: " << crop_info.en
            << " crop width: " << crop_info.cropRect.width
            << " crop height: " << crop_info.cropRect.height
            << " crop x: " << crop_info.cropRect.x
            << " crop y: " << crop_info.cropRect.y;
          ret = HB_VPS_SetChnCrop(group_id, crop_chn, &crop_info);
          if (ret) {
            LOGE << "HB_VPS_SetChnCrop error!!!";
            return ret;
          } else {
            LOGI << "vps set ipu crop ok: GrpId: " << group_id
              << " chn_id: " << chn_idx;
          }
          if (chn_idx == 2) {
            ipu_ds2_crop_en = 1;
          }
        }
      }
      if (chn_idx == 2) {
        ipu_ds2_en = 1;
      }
      ret = SetGdcInfo(kIPU_CHN_DDR_GDC);
      HOBOT_CHECK(ret == 0) << "set ipu chn gdc failed!";
    }
    /* set pym chn */
    if (pym_chn_en == 1) {
      memset(&pym_chn_attr, 0, sizeof(VPS_PYM_CHN_ATTR_S));
      memcpy(&pym_chn_attr, &vps_cfg_->pym_cfg_,
          sizeof(VPS_PYM_CHN_ATTR_S));
      ret = HB_VPS_SetPymChnAttr(group_id, chn_idx, &pym_chn_attr);
      if (ret) {
        LOGE << "HB_VPS_SetPymChnAttr error,ret: " << ret
          << " GrpId: " << group_id << " chn_id: " << chn_idx;
        return ret;
      } else {
        LOGI << "vps set pym chn Attr ok: GrpId: " << group_id
          << " chn_id: " << chn_idx;
      }
      LOGD << "ds5 factor: " << pym_chn_attr.ds_info[5].factor
        << " roi_x: " << pym_chn_attr.ds_info[5].roi_x
        << " roi_y: " << pym_chn_attr.ds_info[5].roi_y
        << " roi_width: " << pym_chn_attr.ds_info[5].roi_width
        << " roi_height: " << pym_chn_attr.ds_info[5].roi_height;

      LOGD << "ds6 factor: " << pym_chn_attr.ds_info[6].factor
        << " roi_x: " << pym_chn_attr.ds_info[6].roi_x
        << " roi_y: " << pym_chn_attr.ds_info[6].roi_y
        << " roi_width: " << pym_chn_attr.ds_info[6].roi_width
        << " roi_height: " << pym_chn_attr.ds_info[6].roi_height;
    }
    /* enable vps chn */
    if ((ipu_chn_en == 1) || (pym_chn_en == 1)) {
      HB_VPS_EnableChn(group_id, chn_idx);
    }
  }

  sem_init(&pym_sem_, 0, 0);
  sem_init(&ipu_sem_, 0, 0);
  if (vps_sync_state_ == true) {
    sem_init(&vps_sync_sem_, 0, 0);
  }
  LOGI << "vps init success...";
  return 0;
}

int VpsModuleHapi::VpsDeInit() {
  int ret = -1;
  int gdc_group_id, gdc_id;
  int group_id = group_id_;

  ret = HB_VPS_DestroyGrp(group_id);
  if (ret < 0) {
    LOGE << "vps deinit failed!";
    return ret;
  }

  /* gdc vpsgroup deinit*/
  for (int gdc_idx = 0; gdc_idx < MAX_GDC_NUM; gdc_idx++) {
    gdc_group_id = gdc_group_id_[gdc_idx];
    gdc_id = GDC_PIPE_ID + gdc_idx;
    if (gdc_group_id == gdc_id) {
      ret = HB_VPS_DestroyGrp(group_id);
      if (ret < 0) {
        LOGE << "vps deinit failed!, gdc_group_id: " << gdc_group_id;
        return ret;
      }
    }
  }
  sem_destroy(&pym_sem_);
  sem_destroy(&ipu_sem_);
  if (vps_sync_state_ == true) {
    sem_destroy(&vps_sync_sem_);
    vps_sync_state_ = false;
  }
  return 0;
}

int VpsModuleHapi::VpsStart() {
  int ret = -1;
  int group_id = group_id_;

  LOGI << "Enter Vps Start, group_id: " << group_id;
  ret = HB_VPS_StartGrp(group_id);
  if (ret) {
    LOGE << "group_id: " <<  group_id
      << " HB_VPS_StartGrp error, ret: " << ret;
    return ret;
  }
  ret = VpsCreateGetDataThread();
  if (ret) {
    LOGE << "Vps create pym thread failed!!!";
    return ret;
  }

  return 0;
}

int VpsModuleHapi::VpsStop() {
  int ret = -1;
  int group_id = group_id_;

  LOGI << "Enter vps module stop, group_id: " << group_id;
  ret = VpsDestoryGetDataThread();
  if (ret) {
    LOGE << "Vps destory pym thread failed!!!";
    return ret;
  }
  LOGI << "HB_VPS_StopGrp, group_id: " << group_id;
  ret = HB_VPS_StopGrp(group_id);
  if (ret) {
    LOGE << "HB_VPS_StopGrp error!!!";
    return ret;
  } else {
    LOGI << "stop grp ok: grp_id = " << group_id;
  }

  LOGI << "Quit vps module stop, group_id: " << group_id;
  return 0;
}

void VpsModuleHapi::HbGetIpuDataThread() {
  int ret = -1;
  int grp_id = group_id_;
  int ipu_fps = 0;

  std::vector<int> &ipu_valid_chn = vps_cfg_->chn_info_.ipu_valid_chn;
  size_t ipu_chn_num = ipu_valid_chn.size();
  if (ipu_chn_num <= 0 || ipu_chn_num >= MAX_CHN_NUM) {
      LOGE << " ipu chn num is invalid: " << ipu_chn_num
      << " group_id: " << grp_id
      << " HbGetIpuDataThread need quit...";
    return;
  }
  // choose first valid ipu chn
  int ipu_chn = ipu_valid_chn[0];
  int timeout = vps_cfg_->chn_cfg_[ipu_chn].ipu_frame_timeout;
  int max_ipu_buffer = vps_cfg_->chn_cfg_[ipu_chn].ipu_frame_depth;
  ipu_rq_ = std::make_shared<RingQueue<VpsIpuInfo>>();
  ipu_rq_->Init(max_ipu_buffer, [](VpsIpuInfo &elem) {
    int ret;
    ret = HB_VPS_ReleaseChnFrame(elem.grp_id, elem.ipu_chn, &elem.ipu_buf);
    if (ret) {
       LOGE << "HB_VPS_ReleaseChnFrame error, ret: " << ret;
     }
    });

  ipu_start_time_ = std::chrono::system_clock::now();
  LOGI << "Enter Get IPU Data Thread, group_id: " << grp_id
    << " start_flag: " << ipu_start_flag_
    << " max_ipu_buffer is: " << max_ipu_buffer;
  while (ipu_start_flag_) {
    VpsIpuInfo ipu_info = { 0 };
    ret = HB_VPS_GetChnFrame(grp_id, ipu_chn, &ipu_info.ipu_buf, timeout);
    if (ret) {
      if (ipu_start_flag_ == true) {
        LOGE << " HB_VPS_GetChnFrame error, ret: " << ret
          << " ipu_chn_num: " << ipu_chn_num
          << " grp_id: " << grp_id
          << " ipu_chn: " << ipu_chn
          << " timeout: " << timeout;
      }
      continue;
    }
    LOGD << "vps get ipu chn frame success, "
      << " group_id: " << grp_id
      << " ipu_chn: " << ipu_chn
      << " frame_id: " << ipu_info.ipu_buf.img_info.frame_id
      << " time_stamp: " << ipu_info.ipu_buf.img_info.time_stamp;
    static int count = 0;
    bool dump_en = false;
    int dump_num = 0;
    GetDumpNum("./vps_output.txt", dump_en, dump_num);
    if (dump_en == true && (count < dump_num || dump_num < 0)) {
      HbDumpIpuData(grp_id, ipu_chn, &ipu_info.ipu_buf);
    }

    ipu_info.grp_id = grp_id;
    ipu_info.ipu_chn = ipu_chn;

    // callback release vin_module src frame,with the exception of mipi_cam
    if (cb_func_) {
      cb_func_(ipu_info.ipu_buf.img_info.frame_id);
    }

    /* calculate ipu fps */
    {
      ipu_fps++;
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - ipu_start_time_);
      if (cost_time.count() >= 1000) {
        LOGW << "group_id:" << grp_id << " ipu_fps:" << ipu_fps;
        ipu_start_time_ = std::chrono::system_clock::now();
        ipu_fps = 0;
      }
    }
    ipu_rq_->Push(ipu_info);
    sem_post(&ipu_sem_);
  }
  ipu_rq_->Clear();
  LOGD << "Quit get ipu data thread, group_id: " << grp_id;
}

void VpsModuleHapi::HbGetPymDataThread() {
  // start get pym data
  int ret = -1;
  int grp_id = group_id_;
  int index = 0;
  VpsPymInfo pym_info;
  size_t pym_chn_num = vps_cfg_->chn_info_.pym_valid_chn.size();
  if (pym_chn_num == 0) {
    LOGW << "pyramid chn has not enable, quit HbGetDataThread";
    return;
  }
  int pym_chn = vps_cfg_->chn_info_.pym_valid_chn[0];
  int timeout = vps_cfg_->pym_cfg_.timeout;
  int frame_depth = vps_cfg_->pym_cfg_.frameDepth;
  int max_pym_buffer = frame_depth - 1;
  // int max_pym_buffer = frame_depth;
  int vio_fps = 0;

  LOGI << "Enter Get PYM Data Thread, group_id: " << grp_id
    << " start_flag: " << pym_start_flag_
    << " max_pym_buffer is: " << max_pym_buffer;
  pym_rq_ = std::make_shared<RingQueue<VpsPymInfo>>();
  pym_rq_->Init(max_pym_buffer, [](VpsPymInfo &elem) {
      int ret;
      ret = HB_VPS_ReleaseChnFrame(elem.grp_id, elem.pym_chn, &elem.pym_buf);
      if (ret) {
      LOGE << "HB_VPS_ReleaseChnFrame error, ret: " << ret;
      }
      });

  pym_start_time_ = std::chrono::system_clock::now();
  while (pym_start_flag_) {
    memset(&pym_info, 0, sizeof(VpsPymInfo));

    ret = HbManagerPymBuffer(max_pym_buffer);
    if (ret) {
      LOGE << "group_id: " << grp_id
        << " HbManagerPymBuffer error, ret: " << ret;
      continue;
    }
    // image get pyramid data, use sync state
    if (vps_sync_state_ == true) {
      uint64_t ts_wait = 2000;  // 2000ms
      ret = HbTimeoutWait(vps_sync_sem_, ts_wait);
      if (ret < 0) {
        continue;
      }
    }
    ret = HB_VPS_GetChnFrame(grp_id, pym_chn, &pym_info.pym_buf, timeout);
    if (ret) {
      if (pym_start_flag_) {
        LOGE << "pipe_id: " << grp_id
          << " HB_VPS_GetChnFrame error, ret: " << ret
          << " grp_id: " << grp_id
          << " pym_chn: " << pym_chn
          << " timeout: " << timeout;
      }
      continue;
    }
    pym_info.grp_id = grp_id;
    pym_info.pym_chn = pym_chn;
    pym_info.buf_index = (index++) % frame_depth;

    // callback release vin_module src frame,with the exception of mipi_cam
    if (cb_func_) {
      cb_func_(pym_info.pym_buf.pym_img_info.frame_id);
    }

    /* calculate vio fps */
    {
      vio_fps++;
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - pym_start_time_);
      if (cost_time.count() >= 1000) {
        LOGW << "group_id:" << grp_id << " vio_fps:" << vio_fps;
        pym_start_time_ = std::chrono::system_clock::now();
        vio_fps = 0;
      }
    }

    /* print pym info */
    HbPrintPymInfo(pym_info, last_pym_info_);
    /* store last pym info */
    last_pym_info_ = pym_info;
    pym_rq_->Push(pym_info);
    sem_post(&pym_sem_);
  }
  pym_rq_->Clear();
  LOGD << "Quit get pym data thread, group_id: " << grp_id;
}

}  // namespace videosource
