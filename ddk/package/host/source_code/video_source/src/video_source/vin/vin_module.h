/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_VIN_VIN_MODULE_H_
#define VIDEO_SOURCE_VIN_VIN_MODULE_H_
#include <atomic>
#include <semaphore.h>
#include <mutex>
#include <condition_variable>
#include <utility>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <functional>
#include "video_source/video_source_type.h"
#include "video_source/decode/decode_module.h"
#include "utils/json_cfg_wrapper.h"
#include "utils/blocking_queue.h"
#include "hobotlog/hobotlog.hpp"
#ifdef __cplusplus
extern "C" {
#endif
#include "libmm/libavutil/imgutils.h"
#include "libmm/libavutil/samplefmt.h"
#include "libmm/libavutil/timestamp.h"
#include "libmm/libavformat/avformat.h"
#include "libmm/libavutil/opt.h"
#include "libmm/libavutil/error.h"
#ifdef __cplusplus
}
#endif

#define MAX_QUEUE_SIZE (20)

namespace videosource {


#define MAX_BUFFER_PLANES                      (3)
#define TIME_MICRO_SECOND                      (1000000)
#define ALIGN(x, a)                            (((x) + (a) - 1) & ~(a - 1))
#define kHorizonVisionVinModuleOffset          (1200)

using DataHandleCb =
  std::function<int(const std::shared_ptr<ImageFrame> &nv12_image)>;
using ManagerHandleCb = std::function<int(std::atomic_bool &status)>;

struct FFMPEGStreamParams {
  FFMPEGStreamParams() {}
  ~FFMPEGStreamParams() {}
  AVCodecContext *pVideoDecCtx = nullptr;
  AVFormatContext *pFormatCtx = nullptr;
  AVFrame *pFrame = nullptr;
  AVStream *pVideoStream = nullptr;
  AVBSFContext *absCtx = nullptr;
  enum AVPixelFormat PixFmt;
  bool is_annexb = false;
  int video_stream_idx = -1;
};

struct VinBufferConfig {
  int max_width;
  int max_height;
  int max_queue_len;
  bool decode_en;
  bool use_vb;  // alloc vb buffer
  // raw format pixel length, 0:8 、1:10 、2:12、3:14 、4:16 5:20
  int raw_pixel_len;
  HorizonVisionPixelFormat format;
};
/* info :
 * y,uv             2 plane
 * raw              1 plane
 * raw, raw         2 plane(dol2)
 * raw, raw, raw    3 plane(dol3)
 **/
struct VinFrame {
  VinBufferConfig vin_buf;
  uint64_t frame_id;
  bool is_key_frame;
  uint64_t time_stamp;  // HW time stamp
  uint64_t system_time_stamp;  // system time stamp
  HorizonVisionPixelFormat  format;   // need update set
  int plane_count;  // need update set
  void *vaddr[MAX_BUFFER_PLANES];  // need update set
  uint64_t paddr[MAX_BUFFER_PLANES];  // need update set if paddr has value
  uint32_t plane_size[MAX_BUFFER_PLANES];
  int width;   // need update set
  int height;  // need update set
  int stride;  // need update set
  uint32_t size;  // need update set
};

class VinModule {
 public:
  VinModule() = delete;
  explicit VinModule(const std::string &module_name,
      const int &channel_id, const int &group_id)
    : module_name_(module_name), channel_id_(channel_id), group_id_(group_id) {}
  ~VinModule() {}

  // derived class implementation
  virtual int Init(const std::string &input_config_file) { return 0; }
  virtual int Init(
      const std::shared_ptr<JsonConfigWrapper> &json_cfg) { return 0; }
  virtual int DeInit() { return 0; }
  virtual int Start() { return 0; }
  virtual int Stop() { return 0; }
  virtual int SetVinBindVps(const int &group_id) { return 0; }
  virtual int ReadImage(char* data, const uint32_t &len,
      const uint32_t &width, const uint32_t &height,
      const HorizonVisionPixelFormat &pixel_format) { return 0; }

 public:
  // base class implementation
  virtual void SetDataHandleCb(
      const DataHandleCb &cb_func) { cb_func_ = cb_func; }
  virtual void SetManagerHandleCb(
      const ManagerHandleCb &cb_func) { manager_cb_func_ = cb_func; }
  virtual void SetOutputEnable(const bool &value) { vin_out_en_ = value; }
  virtual void SetVinFrameDepth(const int &value) { vin_frame_depth_ = value; }
  virtual int GetFrame(std::shared_ptr<ImageFrame> &output_image);
  virtual int FreeFrame(const std::shared_ptr<ImageFrame> &input_image);
  virtual void GetVinOutputSize(int &output_width, int &output_height) {
    output_width = vin_out_image_width_;
    output_height = vin_out_image_height_;
  }
  virtual void SetVinOutputSize(const int &input_width,
      const int &input_height) {
    vin_image_width_ = input_width;
    vin_image_height_ = input_height;
    vin_out_image_width_ = input_width;
    vin_out_image_height_ = input_height;
  }
  virtual bool GetVinIsStart() {
    std::chrono::milliseconds timeout(20000);
    if (VinStartTimeoutWait(timeout) == false) {
      LOGE << "vin start timeout";
      return false;
    }
    return vin_start_flag_;
  }
  virtual bool GetVinSyncStatus() { return vin_sync_state_; }
  virtual void SetVinSyncStatus(const bool &value) { vin_sync_state_ = value; }
  virtual int SetFrameFromWorkedToCompleted(const uint64_t &frame_id);
  virtual bool VinStartTimeoutWait(const std::chrono::milliseconds &timeout);
  virtual void SetVinFpsOutput(const bool &value) { vin_fps_out_en_ = value; }
  virtual int GetBindChannelId() { return bind_chn_id_; }

 protected:
  int VinInit();
  int VinDeInit();
  int VinStart();
  int VinStop();
  int InputData(VinFrame &frame);
  virtual int LoadConfig(const std::string &config_file) { return 0; }
  virtual int LoadConfigFile(const std::string &input_config_file,
      std::shared_ptr<JsonConfigWrapper> &output_json_cfg);
  virtual void GetVinBuf(VinBufferConfig &output) {
    output = vin_buf_cfg_;
  }
  virtual void SetVinBuf(VinBufferConfig &input, bool flag) {
    vin_buf_cfg_ = input;
    update_flag_ = flag;
  }
  virtual void SetVinCachedBufCount(const size_t &value) {
    max_cached_queue_len_ = value; }
  int TimeoutWait(sem_t &sem, const uint64_t &timeout_ms);

 private:
  int BufferQueueInit();
  int BufferQueueDeInit();
  int ParseRawData(VinFrame &frame);
  int InputRawData(VinFrame frame);
  int InputRawDataWithoutVb(VinFrame frame);
  int InputEncodeData(VinFrame frame);
  int DecodeInit();
  int DecodeDeInit();
  int DecodeDataToNv12Thread();
  int RawDataToNv12Thread();
  bool CheckPixelFormat(HorizonVisionPixelFormat &fmt);
  bool CheckBufferInfo(uint32_t width, uint32_t height, uint32_t stride);
  int CheckFirstKeyFrame(const VinFrame &frame);
  int convert_yuy2_to_nv12(void *in_frame, void *y_out_frame,
      void *c_out_frame, int width, int height);
  void DumpVinInputData(const VinFrame &frame);
  void DumpVinOutputData(std::shared_ptr<ImageFrame> &nv12_frame);
  void GetDumpNum(const std::string &input, bool &dump_en, int &dump_num);
  void VinReset();
  int SendStreamToVdec();
  void UpdateVinModule(VinFrame &frame);

 protected:
  std::string module_name_;
  int channel_id_;
  int group_id_;
  DataHandleCb cb_func_;
  ManagerHandleCb manager_cb_func_;
  VinBufferConfig vin_buf_cfg_ = { 0 };
  bool vin_out_en_ = false;
  std::atomic_int vin_image_width_{0};
  std::atomic_int vin_image_height_{0};
  std::atomic_int vin_out_image_width_{0};
  std::atomic_int vin_out_image_height_{0};
  int vin_frame_depth_ = 8;
  size_t max_vin_queue_len_ = 8;
  size_t max_cached_queue_len_ = 8;
  uint64_t frame_id_ = 0;
  std::atomic_int send_stream_num_{0};
  std::atomic_bool update_flag_{false};
  bool vin_sync_state_ = false;
  int bind_chn_id_ = -1;
  std::atomic_bool vin_start_flag_{false};
  std::atomic_bool is_source_stop_{false};
  // bool is_source_stop_ = false;

 private:
  BlockingQueue<std::pair<VinFrame,
    std::shared_ptr<unsigned char>>> encode_queue_;
  BlockingQueue<std::pair<VinFrame,
    std::shared_ptr<unsigned char>>> raw_queue_;
  std::shared_ptr<BlockingQueue<std::shared_ptr<ImageFrame>>> \
    nv12_total_queue_ = nullptr;
  std::shared_ptr<BlockingQueue<std::shared_ptr<ImageFrame>>> \
    nv12_free_queue_ = nullptr;
  std::shared_ptr<BlockingQueue<std::shared_ptr<ImageFrame>>> \
    nv12_work_queue_ = nullptr;
  std::shared_ptr<BlockingQueue<std::shared_ptr<ImageFrame>>> \
    nv12_done_queue_ = nullptr;
  std::shared_ptr<BlockingQueue<std::shared_ptr<ImageFrame>>> \
    nv12_user_queue_ = nullptr;

  std::shared_ptr<DecodeModule> decode_module_ = nullptr;
  std::shared_ptr<std::thread> sp_feed_decoder_task_ = nullptr;
  std::shared_ptr<std::thread> sp_get_decoder_task_ = nullptr;
  std::shared_ptr<std::thread> sp_parse_raw_task_ = nullptr;
  std::shared_ptr<std::thread> sp_update_vinmodule_task_ = nullptr;
  bool decode_is_running_ = false;
  bool parse_raw_is_running_ = false;
  bool vin_init_flag_  = false;
  int buf_lane_num_ = 0;
  sem_t send_sync_sem_;
  sem_t nv12_free_queue_sem_;
  bool check_first_idr_ = false;
  int input_dump_count_ = 0;
  int output_dump_count_ = 0;
  std::condition_variable vin_condition_;
  std::mutex vin_mutex_;
  bool vin_fps_out_en_ = false;
  std::chrono::system_clock::time_point start_time_;
  uint64_t vin_fps_ = 0;
};

}  // namespace videosource
#endif  // VIDEO_SOURCE_VIN_VIN_MODULE_H_
