/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_DECODE_DECODE_MODULE_H_
#define VIDEO_SOURCE_DECODE_DECODE_MODULE_H_
#include "video_source/video_source_type.h"
#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <map>
#include <mutex>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#ifdef X3
#include "vio/hb_comm_vdec.h"
#include "vio/hb_vdec.h"
#include "vio/hb_vp_api.h"
#endif
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#define IN
#define OUT
#define INOUT

namespace videosource {

#define ALIGN(x, a)                            (((x) + (a) - 1) & ~(a - 1))
typedef void* DecodeHandle;

struct StreamBuffer {
  StreamBuffer() {}
  ~StreamBuffer() {}
  std::vector<char *> vaddr_;
  std::vector<uint64_t> paddr_;
  int max_buf_size_ = 0;
  int stream_count_ = 0;
  std::atomic_int buf_index_{0};
  int buf_num_ = 0;
};

class DecodeModule {
 public:
  DecodeModule() = delete;
  explicit DecodeModule(DecodeHandle &handle);
  explicit DecodeModule(int chn, const HorizonVisionPixelFormat &fmt)
    : chn_(chn), fmt_(fmt) {}
  ~DecodeModule() {}
  int Init(const int &width, const int &height, const int &frame_buf_depth);
  int DeInit();
  int Input(IN const uint64_t &frame_id,
      IN const void* input_buf,
      IN const int &input_size);
  int Output(OUT std::shared_ptr<ImageFrame> &output_image);
  int Free(IN std::shared_ptr<ImageFrame> &input_image);
  int GetDecodeChn() { return chn_; }
  HorizonVisionPixelFormat GetDecodeFormat() { return fmt_; }

 protected:
#ifdef X3
  int ConvertPixelFormat(const HorizonVisionPixelFormat &input,
      PAYLOAD_TYPE_E &output);
  int ChnInit(const int &width, const int &height,
      const int &frame_buf_depth, const PAYLOAD_TYPE_E &type);
  int ChnDeInit();
  int ChnStart();
  int ChnStop();
  int StreamBufferInit(const int &buf_num, const int &buf_size);
  int StreamBufferDeInit();
#endif

 private:
  void DumpEncodeData(const void* input_buf, const int &input_size);
  void DumpDecodeData(const void* y_buf, const void* c_buf,
    const int &y_size, const int &c_size);


 private:
  int chn_;  // decode channel
  HorizonVisionPixelFormat fmt_;  // decode frame format
  DecodeHandle handle_;
  bool init_flag_ = false;
  std::shared_ptr<StreamBuffer> sp_stream_buf_ = nullptr;
  bool vb_cache_en_ = true;
  std::string stream_format_;
  std::map<uint64_t, uint64_t> id_map_ts_;
  std::mutex id_map_mutex_;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_DECODE_DECODE_MODULE_H_
