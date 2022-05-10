#include "transfer_nv12.h"

#include <iostream>

namespace uvc_sample {
TestTransfer::TestTransfer()
    : uvc_stream_on_(0),
      nv12_is_on_(false),
      uvc_server_(nullptr),
      is_inited_(false),
      request_width_(0),
      request_height_(0) {}

int TestTransfer::Init() {
  uvc_server_ = UvcServer::GetInstance();
  if (nullptr == uvc_server_) {
    return -1;
  }
  if (0 != uvc_server_->Init(this)) {
    std::cout << "[ERROR] TestTransfer Init uvc server failed";
    return -1;
  }
  is_inited_ = true;
  return 0;
}

int TestTransfer::DeInit() {
  if (false == is_inited_) {
    return -1;
  }
  SetUvcStreamOn(0);
  SetNv12IsOn(false);
  if (uvc_server_) {
    uvc_server_->DeInit();
  }
  uvc_server_ = nullptr;
  is_inited_ = false;
  return 0;
}

int TestTransfer::Start() {
  if (false == is_inited_) {
    return -1;
  }
  if (uvc_server_) {
    uvc_server_->Start();
  }
  return 0;
}

int TestTransfer::Stop() {
  if (false == is_inited_) {
    return -1;
  }
  if (uvc_server_) {
    uvc_server_->Stop();
  }
  return 0;
}

int TestTransfer::SendNv12Data(void *nv12_data, int len, int nv12_width,
                               int nv12_height) {
  if (nv12_data == nullptr || len <= 0 || !IsUvcStreamOn() || !IsNv12On() ||
      nv12_height != request_height_ || nv12_width != request_width_) {
    std::cout << "SendNv12Data input invalid or stream off." << std::endl;
    return -1;
  }
  if (uvc_server_) {
    uvccomponent::UvcVideoData video_data;
    memset(&video_data, 0, sizeof(video_data));
    video_data.v_buffer_ = static_cast<char *>(nv12_data);
    video_data.v_size_ = len;
    uvc_server_->SendFrame(video_data, false);  // uvc_server should copy data
    return 0;
  }
  return -1;
}

void TestTransfer::OnUvcEvent(UvcEvent event_type, void *data, int data_len) {
  std::cout << "TestTransfer::OnUvcEvent event type: " << event_type
            << std::endl;
  switch (event_type) {
    case uvccomponent::UVC_STREAM_OFF:
      /* code */
      SetUvcStreamOn(0);
      break;
    case uvccomponent::UVC_STREAM_ON: {
      uvccomponent::StreamParams *event_data =
          reinterpret_cast<uvccomponent::StreamParams *>(data);
      if (!event_data) {
        break;
      }
      std::cout << "StreaOn param: width = " << event_data->width_
                << ", height = " << event_data->height_
                << ", video_type = " << event_data->video_type_ << std::endl;
      request_width_ = event_data->width_;
      request_height_ = event_data->height_;
      SetUvcStreamOn(0);
      if (event_data->video_type_ == uvccomponent::CODEC_NV12) {
        SetNv12IsOn(true);
        SetUvcStreamOn(1);
      } else {
        SetNv12IsOn(false);
        SetUvcStreamOn(1);
      }
      break;
    }
    case uvccomponent::UVC_ADD:
      if (uvc_server_) {
        uvc_server_->Init(this);
        uvc_server_->Start();
      }
      break;
    case uvccomponent::UVC_REMOVE:
      if (uvc_server_) {
        uvc_server_->DeInit();
      }
      break;
    default:
      break;
  }
}
}  // namespace uvc_sample

