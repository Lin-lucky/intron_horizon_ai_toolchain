/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_MEDIAPIPELINE_H_
#define INCLUDE_MEDIAPIPELINE_H_

#include <future>
#include <memory>

#include "media_pipe_manager/vdec_module.h"
#include "media_pipe_manager/vps_module.h"

namespace solution {
namespace video_box {
class MediaPipeline {
 public:
  MediaPipeline(uint32_t gp_id0, uint32_t gp_id1);
  virtual int Init();
  bool InitFlag();
  virtual int Start();
  bool StartFlag();
  virtual int Stop();
  virtual int Input(void *data);
  virtual int Output(void **data);
  virtual int OutputBufferFree(void *data);
  virtual int DeInit();

  uint32_t GetGrpId();
  int CheckStat();

  void SetDecodeType(const int deocde_type) { decode_type_ = deocde_type; }
  const int GetDecodeType() { return decode_type_; }

  void SetDecodeResolution(const int width, const int height) {
    width_ = width;
    height_ = height;
  }

  void GetDecodeResolution(int &width, int &height) {
    width = width_;
    height = height_;
  }

  void UpdateTime();
  uint64_t GetlastReadDataTime() { return last_recv_data_time_; }

  void SetFrameDropFlag(const bool frame_drop) { frame_drop_ = frame_drop;}
  bool GetFrameDropFlag() { return frame_drop_;}

  void SetFrameDropInterval(const int frame_drop_interval) {
    frame_drop_interval_ = frame_drop_interval;
  }

 private:
  MediaPipeline() = delete;
  MediaPipeline(const MediaPipeline &) = delete;
  MediaPipeline &operator=(const MediaPipeline &) = delete;

  uint32_t vdec_group_id_;
  uint32_t vps_group_id_;

  std::shared_ptr<VdecModule> vdec_module_;
  std::shared_ptr<VpsModule> vps_module_;

  int decode_type_;
  int width_ = 0;
  int height_ = 0;

  bool init_ = false;
  bool running_ = false;
  std::promise<int> promise_;
  bool set_prom_ = false;

  uint64_t last_recv_data_time_;
  bool frame_drop_ = false;
  int frame_drop_interval_ = 0;
};

}  // namespace vision
}  // namespace horizon

#endif  // INCLUDE_MediaPipeline_H_