/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     send_jepg.h
 * \Author   xudong.du
 * \Version  1.0.0.0
 * \Date     2020/08/16
 * \Brief    implement of api header
 */
#ifndef INCLUDE_UVC_SERVER_SAMPLE_TRANSFER_NV12_H_
#define INCLUDE_UVC_SERVER_SAMPLE_TRANSFER_NV12_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "uvc_server/uvc_server.h"

namespace uvc_sample {
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

 private:
  void OnUvcEvent(UvcEvent event_type, void* data, int data_len) override;

 private:
  int uvc_stream_on_;
  bool nv12_is_on_;
  bool is_inited_;
  std::mutex mutex_;
  int request_width_;
  int request_height_;
  std::shared_ptr<UvcServer> uvc_server_;
};
}  // namespace uvc_sample
#endif  // INCLUDE_UVC_SERVER_SAMPLE_TRANSFER_NV12_H_

