#ifndef INCLUDE_UVC_SERVER_GTEST_TRANSFER_FORMAT_H_
#define INCLUDE_UVC_SERVER_GTEST_TRANSFER_FORMAT_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>
#ifdef __cplusplus
extern "C" {
#endif
#include "libswresample/swresample.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/avutil.h"
#include "libavutil/imgutils.h"
#include "libavutil/hwcontext.h"
#ifdef __cplusplus
}
#endif

#include "uvc_server/uvc_server.h"

namespace uvc_gtest {
using uvccomponent::UvcEvent;
using uvccomponent::UvcEventCallback;
using uvccomponent::UvcServer;
class TestTransfer : public UvcEventCallback {
 public:
  TestTransfer();
  ~TestTransfer() = default;

  int Init();
  int DeInit();
  int Start();
  int Stop();

  int SendNv12Data(void* nv12_data, int len, int nv12_width, int nv12_height);
  int SendH264Data(void* h264_data, int len);
  inline bool IsUvcStreamOn() {
    std::lock_guard<std::mutex> lg(mutex_);
    return uvc_stream_on_ > 0;
  }

  inline void SetUvcStreamOn(int on) {
    std::lock_guard<std::mutex> lg(mutex_);
    uvc_stream_on_ = on;
  }
  inline void SetNv12IsOn(bool is_on) {
    std::lock_guard<std::mutex> lg(mutex_);
    nv12_is_on_ = is_on;
  }

  inline bool IsNv12On() {
    std::lock_guard<std::mutex> lg(mutex_);
    return nv12_is_on_;
  }

    inline void SetH264IsOn(bool is_on) {
    std::lock_guard<std::mutex> lg(mutex_);
    h264_is_on_ = is_on;
  }

  inline bool IsH264On() {
    std::lock_guard<std::mutex> lg(mutex_);
    return h264_is_on_;
  }

  inline void SetH265IsOn(bool is_on) {
    std::lock_guard<std::mutex> lg(mutex_);
    h265_is_on_ = is_on;
  }

  inline bool IsH265On() {
    std::lock_guard<std::mutex> lg(mutex_);
    return h265_is_on_;
  }

  inline void SetMjpegIsOn(bool is_on) {
    std::lock_guard<std::mutex> lg(mutex_);
    mjpeg_is_on_ = is_on;
  }

  inline bool IsMjpegOn() {
    std::lock_guard<std::mutex> lg(mutex_);
    return mjpeg_is_on_;
  }

 private:
  void OnUvcEvent(UvcEvent event_type, void* data, int data_len) override;

 private:
  int uvc_stream_on_;
  bool nv12_is_on_;
  bool h264_is_on_;
  bool h265_is_on_;
  bool mjpeg_is_on_;
  bool is_inited_;
  std::mutex mutex_;
  int request_width_;
  int request_height_;
  std::shared_ptr<UvcServer> uvc_server_;
};
}  // namespace uvc_gtest
#endif  // INCLUDE_UVC_SERVER_GTEST_TRANSFER_FORMAT_H_

