/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-08
 *  @Version: v0.0.1
 *  @Brief: implemenation of video buffer manager.
 */

#include "video_source/video_buffer/buffer_manager.h"
#include <cstdint>
#include <string.h>
#include "buffer_manager.h"
#include "hobotlog/hobotlog.hpp"

namespace videosource {

int BufferManager::Init() {
  std::lock_guard<std::mutex> lg(mutex_);
  LOGD << "enter BufferManager Init...";
  int ret = -1;
  ret = VpInit();
  if (ret) {
    LOGE << "video buffer manager vp init falied";
    return ret;
  }
  return 0;
}

int BufferManager::DeInit() {
  std::lock_guard<std::mutex> lg(mutex_);
  LOGD << "enter BufferManager DeInit...";
  int ret = -1;
  ret = VpDeInit();
  if (ret) {
    LOGE << "video buffer manager vp deinit falied";
    return ret;
  }
  return 0;
}

int BufferManager::AllocBufLane(const uint32_t &size,
    std::shared_ptr<ImageFrame> &output) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  uint64_t paddr;
  void *vaddr = nullptr;
  std::shared_ptr<ImageFrame> raw_buf = nullptr;

  raw_buf = std::make_shared<ImageFrame>();
  ret = AllocVpBufLane(size, paddr, vaddr);
  if (ret) {
    LOGE << "alloc vp y_buffer lane failed, ret: " << ret;
    return ret;
  }
  raw_buf->src_info_.y_paddr = paddr;
  raw_buf->src_info_.y_vaddr = reinterpret_cast<uint64_t>(vaddr);
  raw_buf->src_info_.c_paddr = 0;
  raw_buf->src_info_.c_vaddr = 0;
  output = raw_buf;
  LOGD << "alloc buflane success, "
    << " y_paddr: " << vaddr
    << " c_paddr: " << paddr;
  return 0;
}

int BufferManager::FreeBufLane(
    const std::shared_ptr<ImageFrame> &input) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  uint64_t paddr;
  void *vaddr = nullptr;

  if (vp_init_ == false) {
    LOGE << "vp has not init";
    return -1;
  }

  if (input == nullptr) {
    LOGE << "free buf lane is nullptr";
    return -1;
  }

  paddr = input->src_info_.y_paddr;
  vaddr = reinterpret_cast<void*>(input->src_info_.y_vaddr);
  ret = FreeVpBufLane(paddr, vaddr);
  if (ret) {
    LOGE << "free vp buffer lane failed, ret: " << ret;
    return ret;
  }
  return 0;
}

int BufferManager::AllocBuf2Lane(const uint32_t &size_y,
    const uint32_t &size_uv, std::shared_ptr<ImageFrame> &output) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  uint64_t y_paddr, c_paddr;
  void *y_vaddr = nullptr;
  void *c_vaddr = nullptr;

  if (vp_init_ == false) {
    LOGE << "vp has not init";
    return -1;
  }

  std::shared_ptr<ImageFrame> nv12_buf = std::make_shared<ImageFrame>();
  ret = AllocVpBufLane(size_y, y_paddr, y_vaddr);
  if (ret) {
    LOGE << "alloc vp y_buffer lane failed, ret: " << ret;
    return ret;
  }
  nv12_buf->src_info_.y_paddr = y_paddr;
  nv12_buf->src_info_.y_vaddr = reinterpret_cast<uint64_t>(y_vaddr);

  ret = AllocVpBufLane(size_uv, c_paddr, c_vaddr);
  if (ret) {
    LOGE << "alloc vp c_buffer lane failed, ret: " << ret;
    return ret;
  }
  nv12_buf->src_info_.c_paddr = c_paddr;
  nv12_buf->src_info_.c_vaddr = reinterpret_cast<uint64_t>(c_vaddr);

  LOGD << "alloc buf2lane success, "
    << std::hex << " y_paddr: " << "0x" << y_paddr
    << std::hex << " c_paddr: " << "0x" << c_paddr
    << " y_vaddr: " << y_vaddr
    << " c_vaddr: " << c_vaddr;

  output = nv12_buf;
  return 0;
}

int BufferManager::FreeBuf2Lane(const std::shared_ptr<ImageFrame> &input) {
  std::lock_guard<std::mutex> lg(mutex_);
  int ret = -1;
  uint64_t y_paddr, c_paddr;
  void *y_vaddr = nullptr;
  void *c_vaddr = nullptr;

  if (vp_init_ == false) {
    LOGE << "vp has not init";
    return -1;
  }

  if (input == nullptr) {
    LOGE << "free vp buf is nullptr";
    return -1;
  }

  y_paddr = input->src_info_.y_paddr;
  y_vaddr = reinterpret_cast<void*>(input->src_info_.y_vaddr);
  ret = FreeVpBufLane(y_paddr, y_vaddr);
  if (ret) {
    LOGE << "free vp y_buffer lane failed, ret: " << ret;
    return ret;
  }

  c_paddr = input->src_info_.c_paddr;
  c_vaddr = reinterpret_cast<void*>(input->src_info_.c_vaddr);
  ret = FreeVpBufLane(c_paddr, c_vaddr);
  if (ret) {
    LOGE << "free vp uv_buffer lane failed, ret: " << ret;
    return ret;
  }
  return 0;
}

int BufferManager::FreeImageBuffer(
    std::shared_ptr<ImageFrame> &input_buf_info) {
  int ret = -1;
  void* vio_buf = nullptr;
  if (input_buf_info == nullptr) {
    LOGE << "input buf info is nullptr";
    return -1;
  }

  ret = FreeBuf2Lane(input_buf_info);
  if (ret) {
    LOGE << "free buffer 2 lane failed, ret: " << ret;
    return ret;
  }

  auto* frame_ctx = reinterpret_cast<FrameContext*>(
      input_buf_info->src_context_);
  if (frame_ctx) {
    vio_buf = frame_ctx->vio_buf;
    if (vio_buf) {
      std::free(vio_buf);
      vio_buf = nullptr;
    }
    std::free(frame_ctx);
    frame_ctx = nullptr;
  }
  buf_index_--;

  return 0;
}

int BufferManager::GetImageBuffer(
    const uint32_t &input_w,
    const uint32_t &input_h,
    const uint32_t &input_s,
    std::shared_ptr<ImageFrame> &output_buf_info) {
  int ret = -1;
  uint32_t size_y  = input_s * input_h;
  uint32_t size_uv = size_y / 2;

  LOGI << "widht: " << input_w
    << " height: " << input_h
    << " stride: " << input_s;

  auto *frame_ctx = reinterpret_cast<FrameContext*>(
      std::calloc(1, sizeof(FrameContext)));
  if (frame_ctx == NULL) {
    LOGE << "frame context calloc failed";
    return -1;
  }
  hb_vio_buffer_t *vio_buf = reinterpret_cast<hb_vio_buffer_t*>(
        std::calloc(1, sizeof(hb_vio_buffer_t)));
  if (vio_buf == NULL) {
    LOGE << "vio buffer malloc failed";
    return -1;
  }
  /**
   * planeCount value: [1, 2, 3]
   * y,uv             2 plane
   * raw              1 plane
   * raw, raw         2 plane(dol2)
   * raw, raw, raw    3 plane(dol3)
   *
   * img_format value: [0, 1, 8]
   * 0: SIF_FORMAT_RAW
   * 1: SIF_FORMAT_YUV_RAW8
   * 8: SIF_FORMAT_YUV
   **/
  vio_buf->img_info.planeCount = 2;
  vio_buf->img_info.img_format = 8;
  vio_buf->img_addr.width = input_w;
  vio_buf->img_addr.height = input_h;
  vio_buf->img_addr.stride_size = input_s;

  frame_ctx->vin_buf_index = buf_index_++;
  frame_ctx->vps_ipu_chn = -1;
  frame_ctx->vio_buf = reinterpret_cast<hb_vio_buffer_t*>(vio_buf);

  std::shared_ptr<ImageFrame> nv12_buf = nullptr;
  ret = AllocBuf2Lane(size_y, size_uv, nv12_buf);
  if (ret) {
    LOGE << "alloc buffer 2 lane failed, ret: " << ret;
    return ret;
  }
  nv12_buf->src_info_.width = static_cast<uint16_t>(input_w);
  nv12_buf->src_info_.height = static_cast<uint16_t>(input_h);
  nv12_buf->src_info_.stride = static_cast<uint16_t>(input_s);
  nv12_buf->src_context_ = frame_ctx;
  output_buf_info = nv12_buf;

  return 0;
}

#ifdef X3
int BufferManager::VpInit() {
  int ret = -1;
  VP_CONFIG_S vp_config;

  init_ref_cnt_++;
  if (vp_init_ == true) {
    LOGW << "vp has already init";
    return 0;
  }

  memset(&vp_config, 0x00, sizeof(VP_CONFIG_S));
  vp_config.u32MaxPoolCnt = 32;
  ret = HB_VP_SetConfig(&vp_config);
  if (ret) {
    LOGE << "vp set config failed, ret: " << ret;
    return ret;
  }

  ret = HB_VP_Init();
  if (ret) {
    LOGE << "vp init failed, ret: " << ret;
    return ret;
  }
  vp_init_ = true;

  LOGI << "video pool init success...";
  return 0;
}

int BufferManager::VpDeInit() {
  int ret = -1;

  if (vp_init_ == false) {
    LOGW <<  "vp has not init!";
    return 0;
  }
  init_ref_cnt_--;
  if (init_ref_cnt_ == 0) {
    LOGI << "vb start attemp deinit";
  } else {
    LOGW << "vb init referent cnt: " << init_ref_cnt_;
    return 0;
  }
  ret = HB_VP_Exit();
  if (ret == 0) {
    LOGI << "vp exit ok!";
  } else {
    LOGE << "vp exit error!";
    return ret;
  }
  vp_init_ = false;

  LOGI << "video pool deinit success...";
  return 0;
}

int BufferManager::AllocVpBufLane(const uint32_t &size,
    uint64_t &paddr, void* &vaddr) {
  int ret = -1;

  if (vp_init_ == false) {
    LOGE << "vp has not init";
    return -1;
  }

  if (vb_cache_en_ == true) {
    ret = HB_SYS_AllocCached(&paddr, &vaddr, size);
    if (ret) {
      LOGE << "alloc size cached vp buffer failed, ret: " << ret;
      return ret;
    }
  } else {
    ret = HB_SYS_Alloc(&paddr, &vaddr, size);
    if (ret) {
      LOGE << "alloc size vp buffer failed, ret: " << ret;
      return ret;
    }
  }
  return 0;
}

int BufferManager::FreeVpBufLane(uint64_t paddr, void *vaddr) {
  int ret = -1;

  if (vp_init_ == false) {
    LOGE << "vp has not init";
    return -1;
  }

  if (vaddr == nullptr) {
    LOGE << "vaddr is nullptr";
    return -1;
  }

  ret = HB_SYS_Free(paddr, vaddr);
  if (ret != 0) {
    LOGE << "hb sys free buf failed, ret: " << ret;
    return ret;
  }

  return 0;
}
#endif


}  // namespace videosource
