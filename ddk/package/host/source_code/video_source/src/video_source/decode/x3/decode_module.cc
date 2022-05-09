/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-12
 *  @Version: v0.0.1
 *  @Brief: implemenation of decoder module.
 */
#include "video_source/decode/decode_module.h"
#include <iterator>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <utility>
#include "video_source/decode/decode_manager.h"
#include "video_source/video_source.h"
#include "hobotlog/hobotlog.hpp"

namespace videosource {

DecodeModule::DecodeModule(DecodeHandle &handle) {
  handle_ = handle;
  DecChnInfo* chn_info = reinterpret_cast<DecChnInfo*>(handle);
  HOBOT_CHECK(chn_info);
  fmt_ = chn_info->format;
  chn_ = chn_info->channel_id;
}

int DecodeModule::ConvertPixelFormat(
    const HorizonVisionPixelFormat &input, PAYLOAD_TYPE_E &output) {

  switch (input) {
    case kHorizonVisionPixelFormatJPEG: {
      output = PT_JPEG;  // stream buf size need 1024 align
      stream_format_ = "JPEG";
      break;
    }
    case kHorizonVisionPixelFormatMJPEG: {
      output = PT_MJPEG;
      stream_format_ = "MJPEG";
      break;
    }
    case kHorizonVisionPixelFormatH264: {
      output = PT_H264;
      stream_format_ = "H264";
      break;
    }
    case kHorizonVisionPixelFormatH265: {
      output = PT_H265;
      stream_format_ = "H265";
      break;
    }
    default: {
      LOGE << "Unsupport pixel format: " << input;
      return -1;
    }
  }

  return 0;
}

int DecodeModule::ChnInit(const int &width,
    const int &height, const int &frame_buf_depth,
    const PAYLOAD_TYPE_E &type) {
  int ret = -1;
  int max_stream_buf_size = width * height * 1.5;
  VDEC_CHN_ATTR_S ChnAttr;
  LOGI << "deconde module chn init,"
    << " vdec_chn: " << chn_
    << " width: " << width
    << " height: " << height
    << " frame_buf_depth: " << frame_buf_depth
    << " type: " << type;

  memset(&ChnAttr, 0, sizeof(VDEC_CHN_ATTR_S));
  ChnAttr.enType = type;
  ChnAttr.enMode = VIDEO_MODE_FRAME;
  ChnAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
  ChnAttr.u32FrameBufCnt = frame_buf_depth;
  ChnAttr.u32StreamBufCnt = frame_buf_depth;
  ChnAttr.u32StreamBufSize = ALIGN(max_stream_buf_size, 1024);
  ChnAttr.bExternalBitStreamBuf  = HB_TRUE;
  if (type == PT_MJPEG) {
    ChnAttr.stAttrMjpeg.enRotation = CODEC_ROTATION_0;
    ChnAttr.stAttrMjpeg.enMirrorFlip = DIRECTION_NONE;
    ChnAttr.stAttrMjpeg.stCropCfg.bEnable = HB_FALSE;
  } else if (type == PT_H264) {
    ChnAttr.stAttrH264.bandwidth_Opt = HB_TRUE;
    ChnAttr.stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
    ChnAttr.stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
    // ChnAttr.stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DEC;
  }

  ret = HB_VDEC_CreateChn(chn_, &ChnAttr);
  if (ret) {
    LOGE << "hb vdec create chn failed, ret: " << ret;
    return ret;
  }

  ret = HB_VDEC_SetChnAttr(chn_, &ChnAttr);
  if (ret) {
    LOGE << "hb vdec set chn attr failed, ret: " << ret;
    return ret;
  }
  return 0;
}

int DecodeModule::ChnDeInit() {
  int ret = -1;

  ret = HB_VDEC_DestroyChn(chn_);
  if (ret) {
    LOGE << "hb vdec destroy chn failed, ret: " << ret
      << " vdec_chn: " << chn_;
    return ret;
  }

  return 0;
}

int DecodeModule::ChnStart() {
  int ret = -1;

  ret = HB_VDEC_StartRecvStream(chn_);
  if (ret) {
    LOGE << "decode manager start failed, ret: " << ret;
    return ret;
  }

  return 0;
}

int DecodeModule::ChnStop() {
  int ret = -1;

  ret = HB_VDEC_StopRecvStream(chn_);
  if (ret) {
    LOGE << "decode manager stop failed, ret: " << ret;
    return ret;
  }

  return 0;
}

int DecodeModule::StreamBufferInit(const int &buf_num,
    const int &buf_size) {
  int ret = -1;
  LOGI << "enter stream buffer init...";
  auto start_time = std::chrono::system_clock::now();

  sp_stream_buf_ = std::make_shared<StreamBuffer>();
  sp_stream_buf_->buf_num_ = buf_num;
  sp_stream_buf_->max_buf_size_ = buf_size;
  sp_stream_buf_->vaddr_.resize(buf_num);
  sp_stream_buf_->paddr_.resize(buf_num);

  for (int i = 0; i < buf_num; i++) {
    if (vb_cache_en_ == true) {
      ret = HB_SYS_AllocCached(&sp_stream_buf_->paddr_[i],
          reinterpret_cast<void **>(&sp_stream_buf_->vaddr_[i]),
          sp_stream_buf_->max_buf_size_);
      if (ret) {
        LOGE << "decode module alloc ion memory failed, ret: " << ret;
        return ret;
      }
    } else {
      ret = HB_SYS_Alloc(&sp_stream_buf_->paddr_[i],
          reinterpret_cast<void **>(&sp_stream_buf_->vaddr_[i]),
          sp_stream_buf_->max_buf_size_);
      if (ret) {
        LOGE << "decode module alloc ion memory failed, ret: " << ret;
        return ret;
      }
    }
  }
  auto curr_time = std::chrono::system_clock::now();
  auto cost_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(
        curr_time - start_time).count();
  LOGW << "stream buffer init success, cost_time: " << cost_time << "ms";
  return 0;
}

int DecodeModule::StreamBufferDeInit() {
  int ret = -1;
  if (sp_stream_buf_ == nullptr) {
    LOGW << "stream buffer is nullptr";
    return 0;
  }

  int buf_num = sp_stream_buf_->buf_num_;
  for (int i = 0; i < buf_num; i++) {
    ret = HB_SYS_Free(sp_stream_buf_->paddr_[i], sp_stream_buf_->vaddr_[i]);
    if (ret) {
      LOGE << "decode module free ion memory failed, ret: " << ret;
      return ret;
    }
  }

  return 0;
}

int DecodeModule::Init(const int &width, const int &height,
    const int &frame_buf_depth) {
  int ret = -1;
  int stream_buf_size = width * height * 3;  // max stream buf size

  if (init_flag_ == true) {
    LOGE << "decode module has been init, vdec_chn: " << chn_;
    return -1;
  }

  PAYLOAD_TYPE_E type;
  ret = ConvertPixelFormat(fmt_, type);
  if (ret) {
    LOGE << "convert pixel format failed, ret: " << ret;
    return ret;
  }

  ret = StreamBufferInit(frame_buf_depth, stream_buf_size);
  if (ret) {
    LOGE << "vdec Chn stream buffer init failed, ret: " << ret
      << " vdec_chn: " << chn_;
    return ret;
  }

  ret = ChnInit(width, height, frame_buf_depth, type);
  if (ret) {
    LOGE << "vdec Chn init failed, ret: " << ret
      << " vdec_chn: " << chn_;
    return ret;
  }

  ret = ChnStart();
  if (ret) {
    LOGE << "vdec Chn start failed, ret: " << ret
      << " vdec_chn: " << chn_;
    return ret;
  }

  init_flag_ = true;
  LOGI << "decode module init success,"
    << " vdec_chn: " << chn_
    << " width: " << width
    << " height: " << height
    << " frame_buf_depth: " << frame_buf_depth
    << " decode type: " << type;
  return 0;
}

int DecodeModule::DeInit() {
  int ret = -1;

  if (init_flag_ == false) {
    LOGE << "decode module has not init, vdec chn: " << chn_;
    return -1;
  }

  ret = ChnStop();
  if (ret) {
    LOGE << "decode module chn stop failed, ret: " << ret;
    return ret;
  }

  ret = ChnDeInit();
  if (ret) {
    LOGE << "decode module chn deinit failed, ret: " << ret;
    return ret;
  }

  ret = StreamBufferDeInit();
  if (ret) {
    LOGE << "decode module stream buffer deinit failed, ret: " << ret;
    return ret;
  }

  DecodeManager &manager = DecodeManager::Get();
  ret = manager.FreeDecodeHandle(handle_);
  if (ret) {
    LOGE << "decode manager free chn failed, ret: " << ret;
    return ret;
  }
  init_flag_ = false;

  return 0;
}

int DecodeModule::Input(const uint64_t &frame_id,
    const void* input_buf,
    const int &input_size) {
  int ret = -1;

  if (init_flag_ == false) {
    LOGE << "decode module has not init, vdec chn: " << chn_;
    return -1;
  }
  if (sp_stream_buf_ == nullptr) {
    LOGE << "stream buffer is nullptr";
    return -1;
  }
  int max_buf_size = sp_stream_buf_->max_buf_size_;
  if (input_size > max_buf_size) {
    LOGE << "input_size: " << input_size
      << " is exceeds max_buf_size: " <<  max_buf_size;
    return -1;
  }

  // debug encode dump
  DumpEncodeData(input_buf, input_size);
  // copy source data to stream buf
  int buf_num = sp_stream_buf_->buf_num_;
  int buf_index = sp_stream_buf_->buf_index_++ % buf_num;
  memcpy(sp_stream_buf_->vaddr_[buf_index], input_buf, input_size);

  VIDEO_STREAM_S pstStream = { 0 };
  sp_stream_buf_->stream_count_ = frame_id;
  pstStream.pstPack.phy_ptr = sp_stream_buf_->paddr_[buf_index];
  pstStream.pstPack.vir_ptr = sp_stream_buf_->vaddr_[buf_index];
  pstStream.pstPack.pts = frame_id;
  pstStream.pstPack.src_idx = buf_index;
  pstStream.pstPack.size = input_size;
  pstStream.pstPack.stream_end = HB_FALSE;
  // recoder decode send frame time
  {
    std::lock_guard<std::mutex> lk(id_map_mutex_);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    auto curr_tv_ms = ((uint64_t)tv.tv_sec * 1000) +
      ((uint64_t)tv.tv_usec / 1000);
    id_map_ts_.insert(std::pair<uint64_t, uint64_t>(
          pstStream.pstPack.pts, curr_tv_ms));
  }
  LOGD << "send frame_id: " << frame_id << " stream_size: "<< input_size;
  ret = HB_VDEC_SendStream(chn_, &pstStream, 2000);
  if (ret) {
    LOGE << "send stream to decoder fail";
    return ret;
  }

  LOGD << "send frame_id: " << frame_id << " to vdec success";
  return 0;
}

void DecodeModule::DumpEncodeData(const void* input_buf,
    const int &input_size) {
  std::string file_name;
  static int count = 0;
  bool dump_en = false;
  if (0 == access("./vin_encode_dump.txt",  F_OK)) dump_en = true;
  if (dump_en == true) {
    if (fmt_ == kHorizonVisionPixelFormatJPEG
        || fmt_ == kHorizonVisionPixelFormatMJPEG) {
      file_name = "encode_dump_" + std::to_string(count++) + ".jpg";
    } else if (fmt_ == kHorizonVisionPixelFormatH264) {
      file_name = "encode_dump_" + std::to_string(count++) + ".h264";
    } else if (fmt_ == kHorizonVisionPixelFormatH265) {
      file_name = "encode_dump_" + std::to_string(count++) + ".h265";
    } else {
      LOGE << "UnSupport image format: " << fmt_;
    }
    LOGW << "dump fname: " << file_name;
    std::ofstream ofs(file_name);
    ofs.write((const char*)input_buf, input_size);
    ofs.close();
  }
}

void DecodeModule::DumpDecodeData(const void* y_buf, const void* c_buf,
    const int &y_size, const int &c_size) {
  std::string file_name;
  static int count = 0;
  bool dump_en = false;
  if (0 == access("./vin_decode_dump.txt",  F_OK)) dump_en = true;
  if (dump_en == true) {
    file_name = "decode_dump_" + std::to_string(count++) + ".yuv";
    LOGW << "dump fname: " << file_name;
    std::ofstream ofs(file_name);
    ofs.write((const char*)y_buf, y_size);
    ofs.write((const char*)c_buf, c_size);
    ofs.close();
  }
}

int DecodeModule::Output(std::shared_ptr<ImageFrame> &output_image) {
  int ret = -1;
  auto *pstFrame = reinterpret_cast<VIDEO_FRAME_S *>(
      std::calloc(1, sizeof(VIDEO_FRAME_S)));
  if (nullptr == pstFrame) {
    LOGE << "std::calloc video frame buffer failed";
    return -1;
  }

  ret = HB_VDEC_GetFrame(chn_, pstFrame, 5000);
  if (ret) {
    LOGE << "vdec module get frame failed, ret: " << ret
      << " decode_chn: " << chn_;
    return ret;
  }
  void *y_buf = reinterpret_cast<void*>(pstFrame->stVFrame.vir_ptr[0]);
  void *c_buf = reinterpret_cast<void*>(pstFrame->stVFrame.vir_ptr[1]);
  int image_width = pstFrame->stVFrame.width;
  int image_height = pstFrame->stVFrame.height;
  int image_stride = pstFrame->stVFrame.stride;
  int y_size = image_stride * image_height;
  int c_size = pstFrame->stVFrame.size - y_size;
  uint64_t frame_id = pstFrame->stVFrame.pts;

  {
    std::lock_guard<std::mutex> lk(id_map_mutex_);
    uint64_t curr_tv_ms = 0;
    for (auto iter = id_map_ts_.begin(); iter != id_map_ts_.end(); iter++) {
#if 0
      LOGD << "decode iter index: "
        << std::distance(std::begin(id_map_ts_), iter)
        << " frame_id: " << iter->first
        << " timestamp_ms: " << iter->second;
#endif
      if (iter->first == frame_id) {
        auto last_tv_ms = iter->second;
        struct timeval tv;
        gettimeofday(&tv, NULL);
        curr_tv_ms = ((uint64_t)tv.tv_sec * 1000) +
          ((uint64_t)tv.tv_usec / 1000);
        auto tv_ms_diff = curr_tv_ms - last_tv_ms;
        std::string time_cost = "./time_cost.txt";
        if (0 == access(time_cost.c_str(), F_OK)) {
          LOGW << "decode frame_id: " << iter->first
            << " image_width: " << image_width
            << " image_height: " << image_height
            << " stream_format: " << stream_format_
            << " cost_time:  " << tv_ms_diff << "ms";
        }
        iter = id_map_ts_.erase(iter);
        break;
      }
    }
  }
  LOGD << "decode image info, "
    << " frame_id: " << frame_id
    << " width: " << image_width
    << " height: " << image_height
    << " stride: " << image_stride
    << " y_size: " << y_size
    << " c_size: " << c_size
    << " total_size: " << pstFrame->stVFrame.size
    << " y_buf_addr: " << y_buf
    << " c_buf_addr: " << c_buf;
  DumpDecodeData(y_buf, c_buf, y_size, c_size);

  output_image = std::make_shared<ImageFrame>();
  // adapt the align in decoder
  // e.g. input jpeg is 1920*1080, output nv12 maybe 1920*1088
  output_image->frame_id_ = frame_id;
  output_image->pixel_format_ = kHorizonVisionPixelFormatNV12;
  output_image->src_info_.width = pstFrame->stVFrame.width;
  output_image->src_info_.height = pstFrame->stVFrame.height;
  output_image->src_info_.stride = pstFrame->stVFrame.stride;
  output_image->src_info_.y_paddr = pstFrame->stVFrame.phy_ptr[0];
  output_image->src_info_.c_paddr = pstFrame->stVFrame.phy_ptr[1];
  output_image->src_info_.y_vaddr = reinterpret_cast<uint64_t>(y_buf);
  output_image->src_info_.c_vaddr = reinterpret_cast<uint64_t>(c_buf);
  output_image->src_context_ = reinterpret_cast<void*>(pstFrame);

  return 0;
}

int DecodeModule::Free(std::shared_ptr<ImageFrame> &input_image) {
  int ret = -1;
  VIDEO_FRAME_S *pstFrame = nullptr;

  if (input_image == nullptr) {
    LOGE << "decode module free failed, input image is nullptr";
    return -1;
  }

  pstFrame = reinterpret_cast<VIDEO_FRAME_S*>(input_image->src_context_);
  ret = HB_VDEC_ReleaseFrame(chn_, pstFrame);
  if (ret) {
    LOGE << "hb vdec release frame failed, ret: " << ret;
    return ret;
  }
  LOGD << "decode release frame success..."
    << " pts: " << pstFrame->stVFrame.pts
    << " frame_id: " << input_image->frame_id_;
  if (pstFrame) {
    std::free(pstFrame);
    pstFrame = nullptr;
  }
  return 0;
}

}  // namespace videosource
