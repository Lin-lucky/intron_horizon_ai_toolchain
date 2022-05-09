#ifndef COMPONENT_UVC_SERVER_H_
#define COMPONENT_UVC_SERVER_H_

#include <stdint.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "guvc/uvc_gadget.h"
#include "guvc/uvc_gadget_api.h"
// undef problematic defines sometimes defined by system headers (uvc_gadget.h)
#undef max

namespace uvccomponent {

template <typename Dtype>
class RingQueue;

typedef enum {
  CODEC_NV12 = 0,
  CODEC_MJPEG = 1,
  CODEC_H265 = 2,
  CODEC_H264 = 3,
} codec_type;

enum UvcEvent {
  UVC_EVENT_NONE = 0,
  UVC_STREAM_OFF = 1,  //  视频流关闭

  UVC_STREAM_ON,  //  视频流开启，传入width/height/video_type

  UVC_ADD,  // uvc设备接入

  UVC_REMOVE,  // uvc设备
  UVC_EVENT_MAX
};

class UvcEventCallback {
 public:
  virtual void OnUvcEvent(UvcEvent event_type, void *data, int data_len) = 0;
};

struct UvcVideoData {
  int width_;  // 视频宽度

  int height_;  // 视频高度

  codec_type video_type_;  // 视频编码类型

  char *v_buffer_;  // 图像buffer地址

  int v_size_;  // 大小
};

struct StreamParams {
  int width_;  // 视频宽度, stream on/off时会左右参数导出

  int height_;  // 视频高度

  codec_type video_type_;  // 视频编码类型
};

struct UvcEventData {
  UvcEvent event_type_;
  StreamParams params_;
};

typedef enum {
  VENC_INVALID = 0,
  VENC_H264,
  VENC_H265,
  VENC_MJPEG,
} venc_type;

class UvcServer {
 public:
  UvcServer();
  ~UvcServer();

  int Init(UvcEventCallback *call_back);
  /**
   * @brief distory resource
   * @return 0: ok, -1: fail
   */
  int DeInit();
  /**
   * @brief start
   * @return 0: ok, -1: fail
   */
  int Start();
  /**
   * @brief stop
   * @return 0: ok, -1: fail
   */
  int Stop();
  /**
   * @brief send video data to uvc
   * @UvcVideoData video data
   * @bool uvc manager video data buffer,The caller does not need to be
   * concerned with memory release issues, default true.
   * @return 0: ok, -1: fail
   */
  int SendFrame(UvcVideoData &data, bool uvc_manager_buf = true);
  /**
   * @brief return UvcServer Instance
   * @return
   */
  static std::shared_ptr<UvcServer> GetInstance();

 private:
  static bool IsUvcStreamOn() {
    std::lock_guard<std::mutex> lg(mutex_);
    return uvc_stream_on_ == 1;
  }

  static void SetUvcStreamOn(int on) {
    std::lock_guard<std::mutex> lg(mutex_);
    uvc_stream_on_ = on;
  }

  static void uvc_streamon_callback(struct uvc_context *ctx, int is_on,
                                    void *userdata);

  static int uvc_prepare_frame_callback(struct uvc_context *ctx, void **buf_to,
                                        int *buf_len, void **entity,
                                        void *userdata);

  static void uvc_release_frame_callback(struct uvc_context *ctx, void **entity,
                                         void *userdata);
  static codec_type fcc_to_video_format(unsigned int fcc);
  int uvc_init_with_params(venc_type type, int vechn, int width, int height);
  void UvcEventProc();
  void Dwc3UeventMonitor();

 private:
  static int uvc_stream_on_;
  static std::mutex mutex_;
  static struct uvc_context *uvc_ctx_;
  static RingQueue<UvcVideoData> video_queue_;
  static RingQueue<UvcEventData> uvc_event_queue_;
  static std::mutex uvc_instance_mutex_;
  static std::shared_ptr<UvcServer> uvc_instance_;

 private:
  UvcEventCallback *uvc_event_call_back_;  // 用于传出event信息
  int width_;
  int height_;
  venc_type vtype_;
  std::shared_ptr<std::thread> event_thread_;
  std::shared_ptr<std::thread> dwc3_thread_;
  bool stop_;
};
}  // namespace uvccomponent

#endif  //  COMPONENT_UVC_SERVER_H_

