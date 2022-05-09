/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIDEO_BUFFER_BUFFER_MANAGER_H_
#define VIDEO_SOURCE_VIDEO_BUFFER_BUFFER_MANAGER_H_
#include <string>
#include <memory>
#include <mutex>
#include "video_source/video_source_type.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#if defined(X3)
#include "vio/hb_vp_api.h"
#include "vio/hb_vio_interface.h"
#elif defined(J5)
#include "vio/hb_vpm_data_info.h"
#endif
#ifdef __cplusplus
}
#endif  /* __cplusplus */

namespace videosource {

struct FrameContext {
  int vin_buf_index;
  int vps_ipu_chn;
  hb_vio_buffer_t* vio_buf;
};

class BufferManager {
 public:
  static BufferManager &Get() {
    static BufferManager inst;
    return inst;
  }

  int Init();
  int DeInit();
  int AllocBuf2Lane(
      const uint32_t &size_y,
      const uint32_t &size_uv,
      std::shared_ptr<ImageFrame> &output);
  int FreeBuf2Lane(const std::shared_ptr<ImageFrame> &input);
  int AllocBufLane(
      const uint32_t &size,
      std::shared_ptr<ImageFrame> &output);
  int FreeBufLane(const std::shared_ptr<ImageFrame> &input);
  int GetImageBuffer(
      const uint32_t &input_w,
      const uint32_t &input_h,
      const uint32_t &input_s,
      std::shared_ptr<ImageFrame> &output_buf_info);
  int FreeImageBuffer(std::shared_ptr<ImageFrame> &input_buf_info);
  void SetVbCacheEnable(bool &value) { vb_cache_en_ = value; }
  bool GetVbCacheEnable() {
    if (vp_init_ == true)
      return vb_cache_en_;
    else
      return false;
  }

 protected:
#ifdef X3
  int VpInit();
  int VpDeInit();
  int AllocVpBufLane(const uint32_t &size, uint64_t &paddr, void* &vaddr);
  int FreeVpBufLane(uint64_t paddr, void *vaddr);
#endif

 private:
  std::mutex mutex_;
  bool vp_init_ = false;
  int init_ref_cnt_ = 0;
  int buf_index_ = 0;
  bool vb_cache_en_ = true;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIDEO_BUFFER_BUFFER_MANAGER_H_
