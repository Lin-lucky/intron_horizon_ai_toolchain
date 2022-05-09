/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_DECODE_DECODE_MANAGER_H_
#define VIDEO_SOURCE_DECODE_DECODE_MANAGER_H_
#include "video_source/decode/decode_module.h"
#include "video_source/video_source_type.h"
#include <string>
#include <memory>
#include <mutex>

#define IN
#define OUT
#define INOUT
#define VPU_FIFO_NAME  "vpu_id_fifo"
#define JPU_FIFO_NAME  "jpu_id_fifo"
#define VPU_FIFO_NUM    (32)
#define JPU_FIFO_NUM    (64)

namespace videosource {

struct DecChnInfo {
  enum HorizonVisionPixelFormat format;
  int channel_id;
};

struct VpuIdStatus {
  int status[VPU_FIFO_NUM];
};

struct JpuIdStatus {
  int status[JPU_FIFO_NUM];
};

class DecodeManager {
 public:
  static DecodeManager &Get() {
    static DecodeManager inst;
    return inst;
  }
  ~DecodeManager() {}
  DecodeHandle CreateDecodeHandle(IN const HorizonVisionPixelFormat &input_fmt);
  int FreeDecodeHandle(IN DecodeHandle handle);

 protected:
  int ModuleInit(const HorizonVisionPixelFormat &input_fmt);
  int ModuleDeInit(const HorizonVisionPixelFormat &input_fmt);
  int CreateSharedFifo(const std::string &input_fifo_name,
      const int &input_max_num, int &output_fifo_fd);
  int DestroySharedFifo(const std::string &input_fifo_name,
      const int &input_fifo_fd);
  int CreateDecodeIdFifo(const HorizonVisionPixelFormat &input_fmt);
  int DestroyDecodeIdFifo(const HorizonVisionPixelFormat &input_fmt);
  int GetDecodeIdFromFifo(const HorizonVisionPixelFormat &input_fmt,
      int &output_decode_id);
  int FreeDecodeIdToFifo(const HorizonVisionPixelFormat &input_fmt,
      int &input_decode_id);
  int TryTestFifo(const std::string &input_fifo_name);

 private:
  std::mutex mutex_;
  int m_ref_cnt_ = 0;
  bool init_flag_ = false;
  bool vpu_is_main_process_ = false;
  bool jpu_is_main_process_ = false;
  int vpu_fifo_fd_ = -1;
  int jpu_fifo_fd_ = -1;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_DECODE_DECODE_MANAGER_H_
