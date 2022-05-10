/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#include "hb_comm_vdec.h"
#include "hb_vps_api.h"
#include "hobotlog/hobotlog.hpp"
#include "media_pipe_manager/media_pipeline.h"

namespace solution {
namespace video_box {
MediaPipeline::MediaPipeline(uint32_t gp_id0, uint32_t gp_id1)
    : vdec_group_id_(gp_id0),
      vps_group_id_(gp_id1),
      decode_type_(RTSP_Payload_NONE) {
  vps_module_ = std::make_shared<VpsModule>();
  vdec_module_ = std::make_shared<VdecModule>();

  struct timeval tv;
  gettimeofday(&tv, NULL);
  last_recv_data_time_ = (uint64_t)tv.tv_sec;
}

int MediaPipeline::Init() {
  if (init_) return 0;

  PipeModuleInfo module_info;
  module_info.input_height = height_;
  module_info.input_width = width_;
  LOGD << "media pipeline input video width:" << width_
       << " height:" << height_;
  module_info.output_height = 1080;
  module_info.output_width = 1920;
  module_info.frame_depth = 2;
  module_info.input_encode_type = decode_type_;
  LOGD << "vdec frame_depth:" << module_info.frame_depth;
  int ret = vdec_module_->Init(vdec_group_id_, &module_info);
  if (ret != 0) {
    LOGE << "vdec module init fail, group:" << vdec_group_id_;
    return ret;
  }
  module_info.frame_depth = 4;
  LOGD << "vps frame_depth:" << module_info.frame_depth;
  ret = vps_module_->Init(vps_group_id_, &module_info);
  if (ret != 0) {
    LOGE << "vps module init fail, group:" << vps_group_id_;
    return ret;
  }

  init_ = true;

  return 0;
}

bool MediaPipeline::InitFlag() { return init_; }

int MediaPipeline::Start() {
  if (running_) return 0;
  if (!init_) return -1;

  running_ = true;
  vdec_module_->Start();
  vps_module_->Start();

  struct timeval tv;
  gettimeofday(&tv, NULL);
  last_recv_data_time_ = (uint64_t)tv.tv_sec;
  return 0;
}

bool MediaPipeline::StartFlag() { return running_; }

int MediaPipeline::Input(void *data) {
  if (!running_) {
    LOGE << "channel:" << vdec_group_id_ << " has not running, input error";
    return -1;
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  last_recv_data_time_ = (uint64_t)tv.tv_sec;

  auto ret = vdec_module_->Input(data);
  LOGI << "vdec in grp:" << GetGrpId() << "  ret:" << ret;
  if (set_prom_) {
    set_prom_ = false;
    promise_.set_value(ret);
    LOGI << "promise_ set_value";
  }

  return 0;
}

int MediaPipeline::Output(void **data) {
  if (!running_) {
    return -5;
  }
  static int decode_frame = 0;
  int ret = 0;
  hb_vio_buffer_t hb_vio_buf;
  void *data_temp = nullptr;
  ret = vdec_module_->Output(&data_temp);
  LOGI << "vdec out  grp:" << GetGrpId() << "  ret:" << ret;
  if (ret != 0) {
    *data = nullptr;
    return -1;
  }
  if (data_temp == nullptr) {
    LOGE << "data_temp is nullptr";
    return -2;
  }

  if (frame_drop_) {
    decode_frame++;
    if (decode_frame > frame_drop_interval_) {
      vdec_module_->OutputBufferFree(data_temp);
      decode_frame = 0;
      return -2;
    }
  }

  VIDEO_FRAME_S *video_frame = (VIDEO_FRAME_S *)(data_temp);
  memset(&hb_vio_buf, 0, sizeof(hb_vio_buffer_t));
  hb_vio_buf.img_addr.addr[0] = video_frame->stVFrame.vir_ptr[0];
  hb_vio_buf.img_addr.paddr[0] = video_frame->stVFrame.phy_ptr[0];
  hb_vio_buf.img_addr.addr[1] = video_frame->stVFrame.vir_ptr[1];
  hb_vio_buf.img_addr.paddr[1] = video_frame->stVFrame.phy_ptr[1];
  hb_vio_buf.img_addr.width = video_frame->stVFrame.width;
  hb_vio_buf.img_addr.height = video_frame->stVFrame.height;
  hb_vio_buf.img_addr.stride_size = video_frame->stVFrame.width;
  hb_vio_buf.img_info.planeCount = 2;
  hb_vio_buf.img_info.img_format = 8;
  hb_vio_buf.img_info.fd[0] = video_frame->stVFrame.fd[0];
  hb_vio_buf.img_info.fd[1] = video_frame->stVFrame.fd[1];
  ret = vps_module_->Input(&hb_vio_buf);
  LOGI << "vps in  grp:" << GetGrpId() << "  ret:" << ret;
  if (ret != 0) {
    LOGW << "vps input timeout.";
    vdec_module_->OutputBufferFree(data_temp);
    *data = nullptr;
    return -2;
  }
  ret = vps_module_->Output(data);
  LOGI << "vps out  grp:" << GetGrpId() << "  ret:" << ret;
  vdec_module_->OutputBufferFree(data_temp);
  if (ret != 0) {
    LOGW << "vps output timeout.";
    *data = nullptr;
    return -3;
  }

  return 0;
}

int MediaPipeline::OutputBufferFree(void *data) {
  vps_module_->OutputBufferFree(data);
  return 0;
}

int MediaPipeline::Stop() {
  if (!running_) return 0;

  LOGI << "call MediaPipeline stop,"
       << "channel:" << vdec_group_id_;
  running_ = false;
  vps_module_->Stop();
  vdec_module_->Stop();
  return 0;
}

int MediaPipeline::DeInit() {
  decode_type_ = RTSP_Payload_NONE;
  if (!init_) return 0;

  LOGI << "call MediaPipeline DeInit, "
       << "channel:" << vdec_group_id_;
  vps_module_->DeInit();
  vdec_module_->DeInit();
  init_ = false;
  return 0;
}

uint32_t MediaPipeline::GetGrpId() { return vdec_group_id_; }

int MediaPipeline::CheckStat() {
  set_prom_ = true;
  auto fut = promise_.get_future();
  auto fut_stat = fut.wait_for(std::chrono::seconds(3));
  if (std::future_status::ready == fut_stat) {
    auto ret = fut.get();
    LOGI << "grp " << vdec_group_id_ << " state is " << ret;
    return 0;
  } else if (std::future_status::timeout == fut_stat) {
    LOGE << "error!!! grp " << vdec_group_id_ << " start timeout";
    return -2;
  }

  return 0;
}

void MediaPipeline::UpdateTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  last_recv_data_time_ = (uint64_t)tv.tv_sec;
}
}  // namespace vision
}  // namespace horizon
