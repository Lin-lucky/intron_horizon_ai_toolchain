#include "uvc_server/uvc_server.h"

#include "hobotlog/hobotlog.hpp"
#include "ring_queue.h"
#include "uevent_helper.h"

namespace uvccomponent {

#define UEVENT_MSG_LEN 4096

RingQueue<UvcVideoData> UvcServer::video_queue_;
RingQueue<UvcEventData> UvcServer::uvc_event_queue_;
int UvcServer::uvc_stream_on_ = 0;
std::mutex UvcServer::mutex_;
uvc_context *UvcServer::uvc_ctx_ = nullptr;
std::mutex UvcServer::uvc_instance_mutex_;
std::shared_ptr<UvcServer> UvcServer::uvc_instance_ = nullptr;

UvcServer::UvcServer() {
  uvc_stream_on_ = 0;
  width_ = 1920;
  height_ = 1080;
  vtype_ = VENC_MJPEG;
  uvc_instance_ = nullptr;
}
UvcServer::~UvcServer() { DeInit(); }

void UvcServer::UvcEventProc() {
  while (!stop_) {
    UvcEventData ud;
    auto ret = uvc_event_queue_.Pop(ud);
    if (!ret) {
      break;
    }
    if (ud.event_type_ >= UVC_EVENT_MAX || ud.event_type_ <= UVC_EVENT_NONE) {
      continue;
    }
    if (uvc_event_call_back_) {
      uvc_event_call_back_->OnUvcEvent(ud.event_type_, &ud.params_,
                                       sizeof(StreamParams));
    }
  }
}

int UvcServer::Init(UvcEventCallback *call_back) {
  if (call_back == NULL) {
    return -1;
  }
  uvc_event_call_back_ = call_back;
  stop_ = false;
  video_queue_.Init(8, [](UvcVideoData &elem) {
    if (elem.v_buffer_) {
      free(elem.v_buffer_);
      elem.v_buffer_ = nullptr;
    }
  });
  uvc_event_queue_.Init(8, nullptr);
  if (event_thread_ == nullptr) {
    event_thread_ = std::make_shared<std::thread>(
        std::bind(&UvcServer::UvcEventProc, this));
  }
  if (dwc3_thread_ == nullptr) {
    dwc3_thread_ =
        std::make_shared<std::thread>(&UvcServer::Dwc3UeventMonitor, this);
  }
  uvc_init_with_params(vtype_, 0, width_, height_);  // default
  LOGI << "UvcServer::Init";
  return 0;
}

int UvcServer::DeInit() {
  stop_ = true;
  uvc_event_queue_.DeInit();
  video_queue_.DeInit();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  if (uvc_ctx_) {
    Stop();
    uvc_gadget_stop(uvc_ctx_);
    uvc_gadget_deinit(uvc_ctx_);
    uvc_ctx_ = nullptr;
  }
  if (event_thread_) {
    event_thread_->join();
    event_thread_ = nullptr;
  }
  if (dwc3_thread_) {
    dwc3_thread_->join();
    dwc3_thread_ = nullptr;
  }
  return 0;
}

int UvcServer::Start() { return 0; }

int UvcServer::Stop() { return 0; }

int UvcServer::SendFrame(UvcVideoData &data, bool uvc_manager_buf) {
  if (uvc_manager_buf) {
    video_queue_.Push(data);
  } else {
    UvcVideoData *uvc_data =
        reinterpret_cast<UvcVideoData *>(calloc(1, sizeof(UvcVideoData)));
    *uvc_data = data;
    uvc_data->v_buffer_ = reinterpret_cast<char *>(calloc(1, data.v_size_));
    memcpy(uvc_data->v_buffer_, data.v_buffer_, data.v_size_);
    video_queue_.Push(*uvc_data);
  }
  return 0;
}

std::shared_ptr<UvcServer> UvcServer::GetInstance() {
  if (uvc_instance_ == nullptr) {
    std::lock_guard<std::mutex> lg(uvc_instance_mutex_);
    uvc_instance_ = std::make_shared<UvcServer>();
    return uvc_instance_;
  }
  return uvc_instance_;
}

int UvcServer::uvc_init_with_params(venc_type type, int vechn, int width,
                                    int height) {
  struct uvc_params params;
  char *uvc_devname = NULL;  /* uvc Null, lib will find one */
  char *v4l2_devname = NULL; /* v4l2 Null is Null... */
  int ret;

  LOGD << "uvc gadget int with params in";

  /* init uvc user params, just use default params and overlay
   * some specify params.
   */
  uvc_gadget_user_params_init(&params);
  params.width = width;
  params.height = height;
  params.bulk_mode = 1; /* isoc mode still hung, use bulk mode instead */
  params.h264_quirk = 0;
  switch (type) {
    case CODEC_MJPEG:
      params.format = UVC_FORMAT_MJPEG;
      break;
    case CODEC_H264:
      params.format = UVC_FORMAT_H264;
      break;
    case CODEC_H265:
      params.format = UVC_FORMAT_H265;
      break;
    default:
      params.format = UVC_FORMAT_MJPEG;
      break;
  }

  ret = uvc_gadget_init(&uvc_ctx_, uvc_devname, v4l2_devname, &params);
  if (ret < 0) {
    LOGF << "uvc_gadget_init error!";
    return ret;
  }

  uvc_set_streamon_handler(uvc_ctx_, uvc_streamon_callback, NULL);
  /* prepare/release buffer with video queue */
  uvc_set_prepare_data_handler(uvc_ctx_, uvc_prepare_frame_callback, NULL);
  uvc_set_release_data_handler(uvc_ctx_, uvc_release_frame_callback, NULL);

  ret = uvc_gadget_start(uvc_ctx_);
  if (ret < 0) {
    LOGF << "uvc_gadget_start error!";
  }

  LOGD << "uvc gadget int with params out ret(" << ret << ")";

  return ret;
}

codec_type UvcServer::fcc_to_video_format(unsigned int fcc) {
  codec_type format = CODEC_MJPEG;
  switch (fcc) {
    case V4L2_PIX_FMT_NV12:
      format = CODEC_NV12;
      break;
    case V4L2_PIX_FMT_MJPEG:
      format = CODEC_MJPEG;
      break;
    case V4L2_PIX_FMT_H265:
      format = CODEC_H265;
      break;
    case V4L2_PIX_FMT_H264:
      format = CODEC_H264;
      break;
    default:
      break;
  }

  return format;
}

void UvcServer::uvc_streamon_callback(struct uvc_context *ctx, int is_on,
                                      void *userdata) {
  struct uvc_device *dev;
  unsigned int width, height, fcc = 0;

  if (!ctx || !ctx->udev) return;

  dev = ctx->udev;
  width = dev->width;
  height = dev->height;
  fcc = dev->fcc;

  UvcEventData event_data;

  event_data.params_.width_ = width;
  event_data.params_.height_ = height;
  event_data.params_.video_type_ = fcc_to_video_format(fcc);

  if (is_on) {
    usleep(10000);
    event_data.event_type_ = UVC_STREAM_ON;
    SetUvcStreamOn(1);
  } else {
    SetUvcStreamOn(0);
    usleep(10000);
    event_data.event_type_ = UVC_STREAM_OFF;
    video_queue_.Clear();
  }
  uvc_event_queue_.Push(event_data);
}

int UvcServer::uvc_prepare_frame_callback(struct uvc_context *ctx,
                                          void **buf_to, int *buf_len,
                                          void **entity, void *userdata) {
  if (!ctx || !ctx->udev) {
    LOGE << "uvc_get_frame: input params is null";
    return -EINVAL;
  }
  if (!IsUvcStreamOn()) {
    return -EFAULT;
  }
  if (video_queue_.Empty()) {
    return -EINVAL;
  }
  UvcVideoData *one_video =
      reinterpret_cast<UvcVideoData *>(calloc(1, sizeof(UvcVideoData)));
  auto bResult = video_queue_.Pop(*one_video);
  if (!bResult) {
    return -EINVAL;
  }
  *buf_to = one_video->v_buffer_;
  *buf_len = one_video->v_size_;
  *entity = one_video;
#ifdef UVC_DEBUG
  interval_cost_trace_out();
#endif

  return 0;
}

void UvcServer::uvc_release_frame_callback(struct uvc_context *ctx,
                                           void **entity, void *userdata) {
  if (!ctx || !entity || !(*entity)) return;

  auto video_buffer = static_cast<UvcVideoData *>(*entity);
  if (video_buffer->v_buffer_) free(video_buffer->v_buffer_);
  free(video_buffer);
  *entity = nullptr;
  return;
}

void UvcServer::Dwc3UeventMonitor() {
  struct uevent uevent;
  struct timeval timeout = {1, 0};  // 1s timeout
  char msg[UEVENT_MSG_LEN + 2];
  uint64_t event;
  int sock, n;

  sock = open_uevent_socket();
  if (sock < 0) return;

  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout,
                 sizeof(timeout)) < 0) {
    fprintf(stderr, "setsockopt for socket receive timeout failed\n");
    return;
  }

  do {
    n = recv(sock, msg, UEVENT_MSG_LEN, 0);

    if (n < 0 || n > UEVENT_MSG_LEN) continue;

    msg[n] = '\0';
    msg[n + 1] = '\0';

    parse_uevent(msg, &uevent);
    UvcEventData event_data;
    event_data.params_.width_ = 0;
    event_data.params_.height_ = 0;
    event_data.params_.video_type_ = CODEC_MJPEG;
    if (strncmp(uevent.subsystem, "video4linux", 11) == 0 &&
        strncmp(uevent.devname, "video", 5) == 0 &&
        strstr(uevent.path, "gadget")) {
      if (strncmp(uevent.action, "add", 3) == 0) {
        event_data.event_type_ = UVC_ADD;
      } else if (strncmp(uevent.action, "remove", 5) == 0) {
        event_data.event_type_ = UVC_REMOVE;
      }
      uvc_event_queue_.Push(event_data);
    }
  } while (!stop_); /* re-use efd as thread quit flag */

  close_uevent_socket(sock);

  return;
}

}  // namespace uvccomponent

