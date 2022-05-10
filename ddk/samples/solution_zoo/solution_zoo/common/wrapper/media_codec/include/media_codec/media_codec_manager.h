/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2020-06-29
 * @Version: v0.0.1
 * @Brief: media codec manager module
 */

#ifndef HOBOT_MEDIA_CODEC_MANAGER_H_
#define HOBOT_MEDIA_CODEC_MANAGER_H_

#include <mutex>
#include <queue>
#include <vector>
#include "hobotlog/hobotlog.hpp"
#ifdef __cplusplus
extern "C" {
#include "./hb_venc.h"
#include "./hb_comm_video.h"
#include "./hb_common.h"
#include "./hb_comm_venc.h"
}
#endif

struct VideoEncodeSourceBuffer {
  VIDEO_FRAME_PACK_S frame_info;
};

struct VideoEncodeStreamBuffer {
  VIDEO_STREAM_S stream_info;
  int src_height;
  int src_width;
  PIXEL_FORMAT_E src_pix_format;
  uint64_t src_pts;
  int chn;
};

struct VideoEncodeStreamParam {
  PAYLOAD_TYPE_E type;
  int width;
  int height;
  int frame_buf_depth;
  PIXEL_FORMAT_E pix_fmt;
  bool is_cbr;
  int bitrate;
  CODEC_ROTATION_E rotation;
  MIRROR_FLIP_E mirror;
};

namespace horizon {
namespace vision {
class MediaCodecManager {
 public:
  static MediaCodecManager &Get() {
    static MediaCodecManager inst;
    return inst;
  }
  ~MediaCodecManager() {
    if (module_init_) {
      ModuleDeInit();
    }
  }
  int ModuleInit() {
    std::lock_guard<std::mutex> lg(mutex_);
    m_ref_cnt_++;
    if (m_ref_cnt_ > 0 && !module_init_) {
      int ret = HB_VENC_Module_Init();
      HOBOT_CHECK(ret == 0) << "codec media module init failed";
      module_init_ = true;
    }
    return 0;
  }

  int ModuleDeInit() {
    std::lock_guard<std::mutex> lg(mutex_);
    m_ref_cnt_--;
    if (m_ref_cnt_ <= 0 && module_init_) {
      int ret = HB_VENC_Module_Uninit();
      HOBOT_CHECK(ret == 0) << "codec media module deinit failed";
      module_init_ = false;
    }
    return 0;
  }

  int GetEncodeChn() {
    std::lock_guard<std::mutex> lg(mutex_);
    chn_num_++;
    LOGV << "venc chn num: " << chn_num_;
    HOBOT_CHECK(chn_num_ < VENC_MAX_CHN_NUM) << "get encode chn failed";
    return chn_num_;
  }

  int EncodeChnInit(int chn, PAYLOAD_TYPE_E type, int width, int height,
      int frame_buf_depth, PIXEL_FORMAT_E pix_fmt, bool is_cbr, int bitrate);
  int EncodeChnInit(int chn, VideoEncodeStreamParam *param);
  int EncodeChnDeInit(int chn);
  int EncodeChnStart(int chn);
  int EncodeChnStop(int chn);
  int SetUserQfactorParams(int chn, int value);
  // 编码YUV，直接得到编码后的jpg字符串，是对下面的一个接口的再封装
  int EncodeYuvToJpg(int chn, VideoEncodeSourceBuffer *yuv_buf,
          std::vector<uint8_t> &jpg_result);
  int EncodeYuvToJpg(int chn, VideoEncodeSourceBuffer *buf,
          VideoEncodeStreamBuffer **stream_buf);
  int EncodeYuvToH26X(int chn, VideoEncodeSourceBuffer *buf,
          VideoEncodeStreamBuffer **stream_buf);
  int FreeStream(int chn, VideoEncodeStreamBuffer *stream_buf);
  int SetUserCbrParams(int chn, int bit_rate, int fps, int intra, int buf_size);
  int VbBufInit(int chn, int width, int height, int stride,
          int size, int vb_num, int vb_cache_en);
  int VbBufDeInit(int chn);
  int GetVbBuf(int chn, VideoEncodeSourceBuffer **buf);
  int FreeVbBuf(int chn, VideoEncodeSourceBuffer *buf);

  // int PushNV12();
  // int GetStream();
  // InitParam();

 private:
  int SetDefaultChnAttr(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E type,
          int width, int height, int frame_buf_depth, PIXEL_FORMAT_E pix_fmt,
          CODEC_ROTATION_E rotation, MIRROR_FLIP_E mirror);

  int SetDefaultRcParams(int chn, PAYLOAD_TYPE_E type, int width,
      int height, bool is_cbr, int bitrate, VENC_CHN_ATTR_S *attr);
  int AllocFrameBuffer(int chn, VideoEncodeStreamBuffer *buf);
  int AllocVbBuf2Lane(int index, void *buf, int size_y, int size_uv,
      int vb_cache_en);
  int AllocVbBuf3Lane(int index, void *buf, int size_y, int size_u,
      int size_v, int vb_cache_en);
  int AllocVbBufLane(int index, void *buf, int size, int vb_cache_en);
  int EncodeYuvToStream(int chn, VideoEncodeSourceBuffer *buf,
          VideoEncodeStreamBuffer **stream_buf);

 private:
  std::mutex mutex_;
  bool module_init_ = false;
  bool chn_init_ = false;
  int m_ref_cnt_ = 0;
  int chn_num_ = 0;
  VENC_CHN_ATTR_S venc_chn_attr_[VENC_MAX_CHN_NUM];
  VENC_JPEG_PARAM_S venc_chn_jpg_params_[VENC_MAX_CHN_NUM];
  PAYLOAD_TYPE_E chn_type_[VENC_MAX_CHN_NUM];
  int vb_num_[VENC_MAX_CHN_NUM] = { 0 };
  int use_vb_[VENC_MAX_CHN_NUM] = { 0 };
  int vb_cache_en_[VENC_MAX_CHN_NUM] = { 0 };
  std::queue<VideoEncodeSourceBuffer*> vb_src_buf_queue_[VENC_MAX_CHN_NUM];
  std::queue<VideoEncodeStreamBuffer*> vb_stream_buf_queue_[VENC_MAX_CHN_NUM];
};

}  // namespace vision
}  // namespace horizon

#endif  // HOBOT_MEDIA_CODEC_MANAGER_H_
