/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-08
 *  @Version: v0.0.1
 *  @Brief: implemenation of video process system.
 */
#include "video_source/vps/vps_module.h"
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>
#include "video_source/video_buffer/buffer_manager.h"
#include "utils/log.h"
#include "hobotlog/hobotlog.hpp"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#if defined(X3) || defined(J3)
#include "vio/hb_vio_interface.h"
#elif defined(J5)
#include "vio/hb_vpm_data_info.h"
#endif
#ifdef __cplusplus
}
#endif  /* __cplusplus */

namespace videosource {

#if defined(X3) || defined(J3)
#define DOWN_SCALE_MAX          24
#define DOWN_SCALE_MAIN_MAX     6
#define UP_SCALE_MAX            6
#elif defined(J5)
#define DOWN_SCALE_MAIN_MAX     5
#define DOWN_SCALE_ROI_MAX      6
#define UP_SCALE_MAIN_MAX       1
#endif

int VpsModule::LoadConfigFile(
    const std::string &input_config_file,
    std::shared_ptr<JsonConfigWrapper> &output_json_cfg) {
  LOGI << "Load vin module config file:" << input_config_file;
  std::ifstream ifs(input_config_file.c_str());
  if (!ifs) {
    LOGE << "Open config file " << input_config_file << " failed";
    return -1;
  }

  Json::Value cfg_jv;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;
  ifs >> cfg_jv;
  json_cfg.reset(new JsonConfigWrapper(cfg_jv));
  output_json_cfg = json_cfg;
  return 0;
}

void VpsModule::GetDumpNum(const std::string &input,
    bool &dump_en, int &dump_num) {
  if (0 == access(input.c_str(), F_OK)) {
    dump_en = true;
    std::string str;
    std::ifstream ifs(input);
    std::getline(ifs, str);
    if (str.empty()) {
      dump_num = -1;
    } else {
      // trim the spaces in the beginning
      str.erase(0, str.find_first_not_of(' '));
      // trim the spaces in the end
      str.erase(str.find_last_not_of(' ') + 1, std::string::npos);
      dump_num = std::stoi(str);
    }
  }
}

int VpsModule::GetSrcContext(void* ctx, int &chn, void* &vio_buf) {
  if (nullptr == ctx) {
    LOGE << "ctx is nullptr";
    return -1;
  }
  auto *vps_frame_ctx = reinterpret_cast<FrameContext*>(ctx);
  chn = vps_frame_ctx->vps_ipu_chn;
  vio_buf = reinterpret_cast<void*>(vps_frame_ctx->vio_buf);
  return 0;
}

void* VpsModule::CreateSrcContext(int chn, void* vio_buf) {
  auto *vps_frame_ctx = reinterpret_cast<FrameContext*>(
      std::calloc(1, sizeof(FrameContext)));
  if (nullptr == vps_frame_ctx) {
    LOGE << "std::calloc vps frame context failed";
    return nullptr;
  }

  if (chn < 0 || chn > MAX_CHN_NUM) {
    LOGE << "vps ipu channel num is valid, chn: " << chn;
    return nullptr;
  }

  if (vio_buf == nullptr) {
    LOGE << "vio src buffer is nullptr";
    return nullptr;
  }

  vps_frame_ctx->vin_buf_index = -1;
  vps_frame_ctx->vps_ipu_chn = chn;
  vps_frame_ctx->vio_buf =
    reinterpret_cast<hb_vio_buffer_t*>(vio_buf);

  return vps_frame_ctx;
}

void* VpsModule::CreateSrcAddrInfo() {
  auto *pvio_image = reinterpret_cast<void*>(
      std::calloc(1, sizeof(hb_vio_buffer_t)));
  if (nullptr == pvio_image) {
    LOGE << "std::calloc pym buffer failed";
    return nullptr;
  }

  return pvio_image;
}

void* VpsModule::CreatePymAddrInfo() {
#if defined(X3) || defined(J3)
  auto *pvio_image = reinterpret_cast<void*>(
      std::calloc(1, sizeof(pym_buffer_t)));
#elif defined(J5)
  auto *pvio_image = reinterpret_cast<void*>(
      std::calloc(1, sizeof(pym_buffer_v2_t)));
#endif
  if (nullptr == pvio_image) {
    LOGE << "std::calloc pym buffer failed";
    return nullptr;
  }

  return pvio_image;
}

int VpsModule::ConvertVioBufInfo(std::shared_ptr<ImageFrame> src_img,
    void *dst) {
  if (src_img == nullptr || dst == nullptr) {
    LOGE << "input parameter is nullptr";
    return -1;
  }
  auto dst_buf = static_cast<hb_vio_buffer_t*>(dst);
  auto sys_ts_ms = src_img->system_time_stamp_;
  struct timeval tv;
  tv.tv_sec = sys_ts_ms / 1000;
  tv.tv_usec = (sys_ts_ms % 1000) * 1000;
  dst_buf->img_info.tv = tv;
  dst_buf->img_info.frame_id = src_img->frame_id_;
  dst_buf->img_info.time_stamp = src_img->time_stamp_;
  dst_buf->img_info.planeCount = 2;
  dst_buf->img_info.img_format = 8;  // SIF_FORMAT_YUV=8
  dst_buf->img_addr.width = src_img->src_info_.width;
  dst_buf->img_addr.height = src_img->src_info_.height;
  dst_buf->img_addr.stride_size = src_img->src_info_.stride;
  dst_buf->img_addr.paddr[0] = src_img->src_info_.y_paddr;
  dst_buf->img_addr.paddr[1] = src_img->src_info_.c_paddr;
  dst_buf->img_addr.addr[0] =
    reinterpret_cast<char*>(src_img->src_info_.y_vaddr);
  dst_buf->img_addr.addr[1] =
    reinterpret_cast<char*>(src_img->src_info_.c_vaddr);

  return 0;
}

int VpsModule::ConvertSrcInfo(void *src_buf,
    std::shared_ptr<ImageFrame> src_img) {
  if (nullptr == src_buf) {
    return -1;
  }
  if (src_img == nullptr) {
    src_img = std::make_shared<ImageFrame>();
  }
#if defined(X3) || defined(J3) || defined(J5)
  auto src_buffer = static_cast<hb_vio_buffer_t*>(src_buf);
#endif

  src_img->channel_id_ = channel_id_;
  src_img->frame_id_ = src_buffer->img_info.frame_id;
  src_img->time_stamp_ = src_buffer->img_info.time_stamp;
  auto tv = src_buffer->img_info.tv;   // struct timeval
  auto sys_tv_ms = ((uint64_t)tv.tv_sec * 1000) + \
    ((uint64_t)tv.tv_usec / 1000);
  src_img->system_time_stamp_ = sys_tv_ms;

  src_img->src_info_.width = src_buffer->img_addr.width;
  src_img->src_info_.height = src_buffer->img_addr.height;
  src_img->src_info_.stride = src_buffer->img_addr.stride_size;
  src_img->src_info_.y_paddr = src_buffer->img_addr.paddr[0];
  src_img->src_info_.c_paddr = src_buffer->img_addr.paddr[1];
  src_img->src_info_.y_vaddr =
    reinterpret_cast<uint64_t>(src_buffer->img_addr.addr[0]);
  src_img->src_info_.c_vaddr =
    reinterpret_cast<uint64_t>(src_buffer->img_addr.addr[1]);

  LOGD << "xj3 channel_id:" << src_img->channel_id_
    << " frame_id: " << src_img->frame_id_
    << " time_stamp: " << src_img->time_stamp_
    << " system_time_stamp: " << src_img->system_time_stamp_
    << " width:"   << src_img->src_info_.width
    << " height:"  << src_img->src_info_.height
    << " stride:"  << src_img->src_info_.stride
    << " y_paddr:" << src_img->src_info_.y_paddr
    << " c_paddr:" << src_img->src_info_.c_paddr
    << " y_vaddr:" << src_img->src_info_.y_vaddr
    << " c_vaddr:" << src_img->src_info_.c_vaddr;

  return 0;
}


int VpsModule::ConvertPymInfo(void *pym_buf,
    std::shared_ptr<PyramidFrame> pym_img) {
  if (nullptr == pym_buf) {
    LOGE << "input pyramid buffer is nullptr";
    return -1;
  }
  if (pym_img == nullptr) {
    pym_img = std::make_shared<PyramidFrame>();
  }

#if defined(X3) || defined(J3)
  pym_img->bl_ds_.resize(DOWN_SCALE_MAX);
  pym_img->roi_us_.resize(UP_SCALE_MAX);
  auto pym_buffer = static_cast<pym_buffer_t*>(pym_buf);
  pym_img->channel_id_ = channel_id_;
  pym_img->frame_id_ = pym_buffer->pym_img_info.frame_id;
  pym_img->time_stamp_ = pym_buffer->pym_img_info.time_stamp;
  auto tv = pym_buffer->pym_img_info.tv;   // struct timeval
  auto sys_tv_ms = ((uint64_t)tv.tv_sec * 1000) + \
    ((uint64_t)tv.tv_usec / 1000);
  pym_img->system_time_stamp_ = sys_tv_ms;

  // test data delay(ms)
  {
    static uint64_t last_frame_id;
    std::string time_cost = "./time_cost.txt";
    if (0 == access(time_cost.c_str(), F_OK)) {
      auto frame_id_diff = pym_img->frame_id_ - last_frame_id;
      last_frame_id  = pym_img->frame_id_;
      struct timeval tv;
      gettimeofday(&tv, NULL);
      auto curr_tv_ms = ((uint64_t)tv.tv_sec * 1000) + \
                        ((uint64_t)tv.tv_usec / 1000);
      if (pym_img->time_stamp_) {
        auto delay_ms = curr_tv_ms - sys_tv_ms;
        LOGW << " channel_id: " << pym_img->channel_id_
          << " (sif->isp->ipu->pym) frame_id: " << pym_img->frame_id_
          << " frame_id_diff: " << frame_id_diff
          << " delay: " << delay_ms << "ms";

      } else {
        auto delay_ms = curr_tv_ms - sys_tv_ms;
        LOGW << " channel_id: " << pym_img->channel_id_
          << " (ddr->ipu->pym) frame_id: " << pym_img->frame_id_
          << " frame_id_diff: " << frame_id_diff
          << " cost_time:  " << delay_ms << "ms";
      }
    }
  }
  LOGD << "ConvertPymInfo channel_id: " << pym_img->channel_id_
    << " frame_id: " << pym_img->frame_id_
    << " timestamp: " << pym_img->time_stamp_;

  // downscale pyramid info
  for (int i = 0; i < DOWN_SCALE_MAX; ++i) {
    address_info_t *pym_addr = NULL;
    if (i % 4 == 0) {
      pym_addr = reinterpret_cast<address_info_t *>(&pym_buffer->pym[i / 4]);
    } else {
      pym_addr = reinterpret_cast<address_info_t *>(
          &pym_buffer->pym_roi[i / 4][i % 4 - 1]);
    }
    if (i == 0) {
      // source pyramid info
      pym_img->src_info_.width = pym_addr->width;
      pym_img->src_info_.height = pym_addr->height;
      pym_img->src_info_.stride = pym_addr->stride_size;
      pym_img->src_info_.y_paddr = pym_addr->paddr[0];
      pym_img->src_info_.c_paddr = pym_addr->paddr[1];
      pym_img->src_info_.y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->addr[0]);
      pym_img->src_info_.c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->addr[1]);
    LOGD << "xj3 channel_id:" << pym_img->channel_id_
      << " src layer index:" << i
      << " width:"   << pym_img->src_info_.width
      << " height:"  << pym_img->src_info_.height
      << " stride:"  << pym_img->src_info_.stride
      << " y_paddr:" << pym_img->src_info_.y_paddr
      << " c_paddr:" << pym_img->src_info_.c_paddr
      << " y_vaddr:" << pym_img->src_info_.y_vaddr
      << " c_vaddr:" << pym_img->src_info_.c_vaddr;
    }
    pym_img->bl_ds_[i].width = pym_addr->width;
    pym_img->bl_ds_[i].height = pym_addr->height;
    pym_img->bl_ds_[i].stride = pym_addr->stride_size;
    pym_img->bl_ds_[i].y_paddr = pym_addr->paddr[0];
    pym_img->bl_ds_[i].c_paddr = pym_addr->paddr[1];
    pym_img->bl_ds_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->addr[0]);
    pym_img->bl_ds_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->addr[1]);
    LOGD << "xj3 channel_id:" << pym_img->channel_id_
      << " bl_ds layer index:" << i
      << " width:"   << pym_img->bl_ds_[i].width
      << " height:"  << pym_img->bl_ds_[i].height
      << " stride:"  << pym_img->bl_ds_[i].stride
      << " y_paddr:" << pym_img->bl_ds_[i].y_paddr
      << " c_paddr:" << pym_img->bl_ds_[i].c_paddr
      << " y_vaddr:" << pym_img->bl_ds_[i].y_vaddr
      << " c_vaddr:" << pym_img->bl_ds_[i].c_vaddr;
  }
  // upscale pyramid info
  for (int i = 0; i < UP_SCALE_MAX; ++i) {
    pym_img->roi_us_[i].width = pym_buffer->us[i].width;
    pym_img->roi_us_[i].height = pym_buffer->us[i].height;
    pym_img->roi_us_[i].stride = pym_buffer->us[i].stride_size;
    pym_img->roi_us_[i].y_paddr = pym_buffer->us[i].paddr[0];
    pym_img->roi_us_[i].c_paddr = pym_buffer->us[i].paddr[1];
    pym_img->roi_us_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_buffer->us[i].addr[0]);
    pym_img->roi_us_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_buffer->us[i].addr[1]);
    LOGD << "xj3 channel_id:" << pym_img->channel_id_
      << " roi_us layer index:" << i
      << " width:"   << pym_img->roi_us_[i].width
      << " height:"  << pym_img->roi_us_[i].height
      << " stride:"  << pym_img->roi_us_[i].stride
      << " y_paddr:" << pym_img->roi_us_[i].y_paddr
      << " c_paddr:" << pym_img->roi_us_[i].c_paddr
      << " y_vaddr:" << pym_img->roi_us_[i].y_vaddr
      << " c_vaddr:" << pym_img->roi_us_[i].c_vaddr;
  }
#elif defined(J5)
  auto pym_addr = static_cast<pym_buffer_v2_t*>(pym_buf);
  pym_img->bl_ds_.resize(DOWN_SCALE_MAIN_MAX);
  pym_img->gs_ds_.resize(DOWN_SCALE_MAIN_MAX);
  pym_img->roi_ds_.resize(DOWN_SCALE_ROI_MAX);
  pym_img->roi_us_.resize(UP_SCALE_MAIN_MAX);
  pym_img->channel_id_ = channel_id_;
  pym_img->frame_id_ = pym_addr->pym_img_info.frame_id;
  pym_img->time_stamp_ = pym_addr->pym_img_info.time_stamp;

  /* 1. src out */
  {
    i = -1;
    pym_img->src_info_.width = pym_addr->src_out.width;
    pym_img->src_info_.height = pym_addr->src_out.height;
    pym_img->src_info_.stride = pym_addr->src_out.stride_size;
    pym_img->src_info_.y_paddr = pym_addr->src_out.paddr[0];
    pym_img->src_info_.c_paddr = pym_addr->src_out.paddr[1];
    pym_img->src_info_.y_vaddr =
      reinterpret_cast<uint64_t>(pym_addr->src_out.addr[0]);
    pym_img->src_info_.c_vaddr =
      reinterpret_cast<uint64_t>(pym_addr->src_out.addr[1]);
    LOGD << "j5 channel_id:" << pym_img->channel_id_
      << " src layer index:" << i
      << " width:"   << pym_img->src_info_.width
      << " height:"  << pym_img->src_info_.height
      << " stride:"  << pym_img->src_info_.stride
      << " y_paddr:" << pym_img->src_info_.y_paddr
      << " c_paddr:" << pym_img->src_info_.c_paddr
      << " y_vaddr:" << pym_img->src_info_.y_vaddr
      << " c_vaddr:" << pym_img->src_info_.c_vaddr;
  }

  /* 2. bl downscale pym convert */
  for (i = 0; i < DOWN_SCALE_MAIN_MAX; ++i) {
    pym_img->bl_ds_[i].width = pym_addr->bl[i].width;
    pym_img->bl_ds_[i].height = pym_addr->bl[i].height;
    pym_img->bl_ds_[i].stride = pym_addr->bl[i].stride_size;
    pym_img->bl_ds_[i].y_paddr = pym_addr->bl[i].paddr[0];
    pym_img->bl_ds_[i].c_paddr = pym_addr->bl[i].paddr[1];
    pym_img->bl_ds_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->bl[i].addr[0]);
    pym_img->bl_ds_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->bl[i].addr[1]);

    LOGD << "j5 channel_id:" << pym_img->channel_id_
      << " bl_ds layer index:" << i
      << " width:"   << pym_img->bl_ds_[i].width
      << " height:"  << pym_img->bl_ds_[i].height
      << " stride:"  << pym_img->bl_ds_[i].stride
      << " y_paddr:" << pym_img->bl_ds_[i].y_paddr
      << " c_paddr:" << pym_img->bl_ds_[i].c_paddr
      << " y_vaddr:" << pym_img->bl_ds_[i].y_vaddr
      << " c_vaddr:" << pym_img->bl_ds_[i].c_vaddr;
  }

  /* 3. gs downscale pym convert */
  for (i = 0; i < DOWN_SCALE_MAIN_MAX; ++i) {
    pym_img->gs_ds_[i].width = pym_addr->gs[i].width;
    pym_img->gs_ds_[i].height = pym_addr->gs[i].height;
    pym_img->gs_ds_[i].stride = pym_addr->gs[i].stride_size;
    pym_img->gs_ds_[i].y_paddr = pym_addr->gs[i].paddr[0];
    pym_img->gs_ds_[i].c_paddr = pym_addr->gs[i].paddr[1];
    pym_img->gs_ds_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->gs[i].addr[0]);
    pym_img->gs_ds_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->gs[i].addr[1]);

    LOGD << "j5 channel_id:" << pym_img->channel_id_
      << " gs_ds layer index:" << i
      << " width:"   << pym_img->gs_ds_[i].width
      << " height:"  << pym_img->gs_ds_[i].height
      << " stride:"  << pym_img->gs_ds_[i].stride
      << " y_paddr:" << pym_img->gs_ds_[i].y_paddr
      << " c_paddr:" << pym_img->gs_ds_[i].c_paddr
      << " y_vaddr:" << pym_img->gs_ds_[i].y_vaddr
      << " c_vaddr:" << pym_img->gs_ds_[i].c_vaddr;
  }

  /* 4. roi downscale convert */
  for (i = 0; i < DOWN_SCALE_ROI_MAX; ++i) {
    pym_img->roi_ds_[i].width = pym_addr->ds_roi[i].width;
    pym_img->roi_ds_[i].height = pym_addr->ds_roi[i].height;
    pym_img->roi_ds_[i].stride = pym_addr->ds_roi[i].stride_size;
    pym_img->roi_ds_[i].y_paddr = pym_addr->ds_roi[i].paddr[0];
    pym_img->roi_ds_[i].c_paddr = pym_addr->ds_roi[i].paddr[1];
    pym_img->roi_ds_[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->ds_roi[i].addr[0]);
    pym_img->roi_ds_[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->ds_roi[i].addr[1]);

    LOGD << "j5 channel_id:" << pym_img->channel_id_
      << " roi_ds layer index:" << i
      << " width:"   << pym_img->roi_ds_[i].width
      << " height:"  << pym_img->roi_ds_[i].height
      << " stride:"  << pym_img->roi_ds_[i].stride
      << " y_paddr:" << pym_img->roi_ds_[i].y_paddr
      << " c_paddr:" << pym_img->roi_ds_[i].c_paddr
      << " y_vaddr:" << pym_img->roi_ds_[i].y_vaddr
      << " c_vaddr:" << pym_img->roi_ds_[i].c_vaddr;
  }

  /* 5. roi upscale convert */
  for (i = 0; i < UP_SCALE_MAIN_MAX; ++i) {
    pym_img->roi_us_[i].width = pym_addr->us_roi.width;
    pym_img->roi_us_[i].height = pym_addr->us_roi.height;
    pym_img->roi_us_[i].stride = pym_addr->us_roi.stride_size;
    pym_img->roi_us_[i].y_paddr = pym_addr->us_roi.paddr[0];
    pym_img->roi_us_[i].c_paddr = pym_addr->us_roi.paddr[1];
    pym_img->roi_us_[i].y_vaddr =
      reinterpret_cast<uint64_t>(pym_addr->us_roi.addr[0]);
    pym_img->roi_us_[i].c_vaddr =
      reinterpret_cast<uint64_t>(pym_addr->us_roi.addr[1]);

    LOGD << "j5 channel_id:" << pym_img->channel_id_
      << " roi_us layer index:" << i
      << " width:"   << pym_img->roi_us_[i].width
      << " height:"  << pym_img->roi_us_[i].height
      << " stride:"  << pym_img->roi_us_[i].stride
      << " y_paddr:" << pym_img->roi_us_[i].y_paddr
      << " c_paddr:" << pym_img->roi_us_[i].c_paddr
      << " y_vaddr:" << pym_img->roi_us_[i].y_vaddr
      << " c_vaddr:" << pym_img->roi_us_[i].c_vaddr;
  }
#endif

  return 0;
}

int VpsModule::HbDumpToFile2Plane(char *filename, char *src_buf,
    char *src_buf1, unsigned int size, unsigned int size1,
    int width, int height, int stride) {
  FILE *yuvFd = NULL;
  char *buffer = NULL;
  int i = 0;

  yuvFd = fopen(filename, "w+");

  if (yuvFd == NULL) {
    LOGE << "open file: " << filename << "failed";
    return -1;
  }

  buffer = reinterpret_cast<char *>(malloc(size + size1));

  if (buffer == NULL) {
    LOGE << "malloc falied";
    fclose(yuvFd);
    return -1;
  }

  if (width == stride) {
    memcpy(buffer, src_buf, size);
    memcpy(buffer + size, src_buf1, size1);
  } else {
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy(buffer+i*width, src_buf+i*stride, width);
    }

    // jump over stride - width UV
    for (i = 0; i < height/2; i++) {
      memcpy(buffer+size+i*width, src_buf1+i*stride, width);
    }
  }

  fwrite(buffer, 1, size + size1, yuvFd);

  fflush(yuvFd);

  if (yuvFd)
    fclose(yuvFd);
  if (buffer)
    free(buffer);

  pr_debug("filedump(%s, size(%d) is successed!!\n", filename, size);

  return 0;
}

int VpsModule::HbDumpPymData(int grp_id, int pym_chn,
    pym_buffer_t *out_pym_buf) {
  int i;
  char file_name[100] = {0};

  for (i = 0; i < 6; i++) {
    snprintf(file_name,
        sizeof(file_name),
        "grp%d_chn%d_pym_out_basic_layer_DS%d_%d_%d.yuv",
        grp_id, pym_chn, i * 4,
        out_pym_buf->pym[i].width,
        out_pym_buf->pym[i].height);
    HbDumpToFile2Plane(
        file_name,
        out_pym_buf->pym[i].addr[0],
        out_pym_buf->pym[i].addr[1],
        out_pym_buf->pym[i].width * out_pym_buf->pym[i].height,
        out_pym_buf->pym[i].width * out_pym_buf->pym[i].height / 2,
        out_pym_buf->pym[i].width,
        out_pym_buf->pym[i].height,
        out_pym_buf->pym[i].stride_size);
    for (int j = 0; j < 3; j++) {
      snprintf(file_name,
          sizeof(file_name),
          "grp%d_chn%d_pym_out_roi_layer_DS%d_%d_%d.yuv",
          grp_id, pym_chn, i * 4 + j + 1,
          out_pym_buf->pym_roi[i][j].width,
          out_pym_buf->pym_roi[i][j].height);
      if (out_pym_buf->pym_roi[i][j].width != 0)
        HbDumpToFile2Plane(
            file_name,
            out_pym_buf->pym_roi[i][j].addr[0],
            out_pym_buf->pym_roi[i][j].addr[1],
            out_pym_buf->pym_roi[i][j].width *
            out_pym_buf->pym_roi[i][j].height,
            out_pym_buf->pym_roi[i][j].width *
            out_pym_buf->pym_roi[i][j].height / 2,
            out_pym_buf->pym_roi[i][j].width,
            out_pym_buf->pym_roi[i][j].height,
            out_pym_buf->pym_roi[i][j].stride_size);
    }
    snprintf(file_name,
        sizeof(file_name),
        "grp%d_chn%d_pym_out_us_layer_US%d_%d_%d.yuv",
        grp_id, pym_chn, i,
        out_pym_buf->us[i].width,
        out_pym_buf->us[i].height);
    if (out_pym_buf->us[i].width != 0)
      HbDumpToFile2Plane(
          file_name,
          out_pym_buf->us[i].addr[0],
          out_pym_buf->us[i].addr[1],
          out_pym_buf->us[i].width * out_pym_buf->us[i].height,
          out_pym_buf->us[i].width * out_pym_buf->us[i].height / 2,
          out_pym_buf->us[i].width,
          out_pym_buf->us[i].height,
          out_pym_buf->us[i].stride_size);
  }

  return 0;
}

int VpsModule::HbDumpPymLayerData(int grp_id, int pym_chn,
    int layer, pym_buffer_t *out_pym_buf) {
  char file_name[100] = {0};
  address_info_t pym_addr;
  int y_len, uv_len;

  uint32_t frame_id = out_pym_buf->pym_img_info.frame_id;
  uint64_t ts = out_pym_buf->pym_img_info.time_stamp;

  pr_debug("ts:%lu, frameID:%d\n", ts, frame_id);

  if (out_pym_buf == NULL) {
    LOGE << "out_pym_buf is nullptr";
    return -1;
  }

  if (layer % 4 == 0) {
    pym_addr = out_pym_buf->pym[layer / 4];
    snprintf(file_name,
        sizeof(file_name),
        "%d_grp%d_chn%d_pym_out_basic_layer_DS%d_%d_%d.yuv",
        dump_index_++, grp_id, pym_chn, layer,
        pym_addr.width, pym_addr.height);
  } else {
    pym_addr = out_pym_buf->pym_roi[layer / 4][layer % 4 - 1];
    snprintf(file_name,
        sizeof(file_name),
        "%d_grp%d_chn%d_pym_out_roi_layer_DS%d_%d_%d.yuv",
        dump_index_++, grp_id, pym_chn, layer,
        pym_addr.width, pym_addr.height);
  }

  if (pym_addr.width == 0 ||
      pym_addr.height == 0) {
    LOGE << "pym_width: " << pym_addr.width
      << " pym_height" << pym_addr.height << "error!!!";
    return -1;
  }

  y_len = pym_addr.width * pym_addr.height;
  uv_len = y_len / 2;
  HbDumpToFile2Plane(file_name,
      pym_addr.addr[0],
      pym_addr.addr[1],
      y_len, uv_len,
      pym_addr.width,
      pym_addr.height,
      pym_addr.stride_size);

  return 0;
}

int VpsModule::HbDumpIpuData(int grp_id, int ipu_chn,
    hb_vio_buffer_t *out_ipu_buf) {
  char file_name[100] = {0};
  uint32_t frame_id = out_ipu_buf->img_info.frame_id;

  snprintf(file_name,
      sizeof(file_name),
      "grp%d_chn%d_frame%u_%d_%d.yuv",
      grp_id, ipu_chn, frame_id,
      out_ipu_buf->img_addr.width,
      out_ipu_buf->img_addr.height);

  HbDumpToFile2Plane(
      file_name,
      out_ipu_buf->img_addr.addr[0],
      out_ipu_buf->img_addr.addr[1],
      out_ipu_buf->img_addr.width * out_ipu_buf->img_addr.height,
      out_ipu_buf->img_addr.width * out_ipu_buf->img_addr.height / 2,
      out_ipu_buf->img_addr.width,
      out_ipu_buf->img_addr.height,
      out_ipu_buf->img_addr.stride_size);

  return 0;
}

int VpsModule::HbCheckPymData(VpsPymInfo &pym_info,
    const int &ds_layer_num) {
  int i;
  int ret = 0;
  address_info_t pym_addr;
  pym_buffer_t *pym_buf = &pym_info.pym_buf;
  uint64_t ts_cur;
  int frame_id;

  frame_id = pym_info.pym_buf.pym_img_info.frame_id;
  ts_cur = pym_info.pym_buf.pym_img_info.time_stamp;
  pr_debug("group_id:%d buf_index[%d] pym_chn:%d"
      " frame_id:%d ts_cur: %lu \n",
      pym_info.grp_id,
      pym_info.buf_index,
      pym_info.pym_chn,
      frame_id, ts_cur);

  for (i = 0; i < ds_layer_num; ++i) {
    if (i % 4 == 0) {  // base pym layer
      pym_addr = pym_buf->pym[i / 4];
      if (pym_addr.width == 0 || pym_addr.height == 0) {
        ret = -1;
      }
    } else {  // roi pym layer
      pym_addr = pym_buf->pym_roi[i / 4][i % 4 - 1];
    }
    pr_debug("layer:%d, width:%d, height:%d, stride_size:%d, "
        "y_addr:%p, uv_addr:%p\n",
        i,
        pym_addr.width, pym_addr.height, pym_addr.stride_size,
        pym_addr.addr[0], pym_addr.addr[1]);
    if (ret) {
      LOGE << "vio check pym failed"
        << " pym_layer: " << i
        << " width: " << pym_addr.width
        << " height: " << pym_addr.height;
      return ret;
    }
  }

  return 0;
}

int VpsModule::HbTimeoutWait(sem_t &sem,
    const uint64_t &timeout_ms) {
  int32_t ret = -1;
  uint64_t end_time = 0;
  struct timeval currtime;
  struct timespec ts;

  if (timeout_ms != 0) {
    gettimeofday(&currtime, NULL);
    end_time = (((((uint64_t) currtime.tv_sec) * TIME_MICRO_SECOND) +
          currtime.tv_usec) + (timeout_ms * 1000));
    ts.tv_sec = (end_time / TIME_MICRO_SECOND);
    ts.tv_nsec = ((end_time % TIME_MICRO_SECOND) * 1000);
    ret = sem_timedwait(&sem, &ts);
  } else {
    ret = sem_wait(&sem);
  }
  return ret;
}

void VpsModule::HbAllocPymBuffer() {
  std::lock_guard<std::mutex> lg(mutex_);
  consumed_pym_buffers_++;
}

void VpsModule::HbFreePymBuffer() {
  std::lock_guard<std::mutex> lg(mutex_);
  if (0 == consumed_pym_buffers_)
    return;
  consumed_pym_buffers_--;
}

int VpsModule::HbManagerPymBuffer(int max_buf_num) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  int queue_size, total_buf_num;

  if (pym_rq_ == nullptr) {
    LOGE << "pym ring_queue is nullptr";
    return -1;
  }
  queue_size = pym_rq_->Size();
  total_buf_num = queue_size + consumed_pym_buffers_;
  pr_info("group_id:%d queue_size:%d consumed_pym_buffers:%d"
      " total_buf_num:%d max_buf_num:%d\n",
      group_id_, queue_size, consumed_pym_buffers_,
      total_buf_num, max_buf_num);
  if (total_buf_num >= max_buf_num) {
  /**
   * Attention:
   * 1. if consumed_pym_buffers is more than the (max_buf_num - 1),
   *    vpsmodule will not discard this pym frame.
   * 2. if queue_size is more than the max_buf_num,
   *    vpsmodule will discard this pym frame.
   */
    LOGI  << "group_id: " << group_id_
      << " PYM Data is Full!"
      << " total_buf_num: " << total_buf_num
      << " queue_size: " << queue_size
      << " consumed_pym_buffers: " << consumed_pym_buffers_
      << " max_buf_num: " << max_buf_num;
    if (queue_size >= max_buf_num) {
      VpsPymInfo pym_info = { 0 };
      if (false == pym_rq_->Peek(pym_info)) {
        LOGE << "pym queue is empty";
        return -1;
      }
      auto frame_id = pym_info.pym_buf.pym_img_info.frame_id;
      LOGW << "group_id: " << group_id_
        << " PYM RingQueue will discard first pym frame data,"
        << " frame_id: " << frame_id;
      ret = sem_trywait(&pym_sem_);
      if (ret) {
        LOGE << "sem try wait failure, ret: " << ret;
        return ret;
      }
      pym_rq_->DeleteOne();
    }
  }
  return 0;
}

void VpsModule::HbPrintPymInfo(const VpsPymInfo &curr_pym_info,
    const VpsPymInfo &last_pym_info) {
  /* current pym info */
  int curr_frame_id = curr_pym_info.pym_buf.pym_img_info.frame_id;
  uint64_t curr_ts = curr_pym_info.pym_buf.pym_img_info.time_stamp;
  auto curr_tv = curr_pym_info.pym_buf.pym_img_info.tv;  // struct timeval
  auto curr_tv_ms = ((uint64_t)curr_tv.tv_sec * 1000) + \
                    ((uint64_t)curr_tv.tv_usec / 1000);
  /* last pym info */
  uint64_t last_ts = last_pym_info.pym_buf.pym_img_info.time_stamp;
  int last_frame_id = last_pym_info.pym_buf.pym_img_info.frame_id;
  auto last_tv = last_pym_info.pym_buf.pym_img_info.tv;  // struct timeval
  auto last_tv_ms = ((uint64_t)last_tv.tv_sec * 1000) + \
                    ((uint64_t)last_tv.tv_usec / 1000);
  /* pym frame ts diff value */
  auto ts_diff = curr_ts - last_ts;
  auto frame_id_diff = curr_frame_id - last_frame_id;
  auto tv_ms_diff = curr_tv_ms - last_tv_ms;
  pr_warn("pipe_id:%d buf_index[%d] pym_chn:%d"
      " curr_frame_id:%d, frame_id_diff:%d"
      " curr_ts:%lu ts_diff:%lu"
      " curr_tv_ms:%lu tv_ms_diff:%lu"
      " width:%d height:%d\n",
      curr_pym_info.grp_id,
      curr_pym_info.buf_index,
      curr_pym_info.pym_chn,
      curr_frame_id, frame_id_diff,
      curr_ts, ts_diff,
      curr_tv_ms, tv_ms_diff,
      curr_pym_info.pym_buf.pym[0].width,
      curr_pym_info.pym_buf.pym[0].height);
}

}  // namespace videosource
