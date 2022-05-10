/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "media_pipe_manager/vdec_module.h"

#include "hb_vdec.h"
#include "hb_vio_interface.h"
#include "hobotlog/hobotlog.hpp"

namespace solution {
namespace video_box {
std::once_flag VdecModule::flag_;

VdecModule::VdecModule() : group_id_(-1), timeout_(100), frameDepth_(-1) {
  std::call_once(flag_, []() {
    int ret = HB_VDEC_Module_Init();
    if (ret != 0) {
      LOGF << "HB_VDEC_Module_Init Failed. ret = " << ret;
    }
  });
}

VdecModule::~VdecModule() {}

int VdecModule::Init(uint32_t group_id, const PipeModuleInfo *module_info) {
  int ret = 0;
  group_id_ = group_id;
  frameDepth_ = module_info->frame_depth;
  buffers_.resize(frameDepth_);
  VDEC_CHN_ATTR_S vdec_attr;
  memset(&vdec_attr, 0, sizeof(VDEC_CHN_ATTR_S));
  if (module_info->input_encode_type == RTSP_Payload_H264) {
    vdec_attr.enType = PT_H264;
  } else if (module_info->input_encode_type == RTSP_Payload_H265) {
    vdec_attr.enType = PT_H265;
    LOGI << "vdec init decoder type is 265";
  } else {
    LOGE << "VdecModule::Init recv unknow decoder type:"
         << module_info->input_encode_type << ", channel:" << group_id;
    return -1;
  }

  LOGI << "vdec module recv module_info->input_width:"
       << module_info->input_width
       << " height:" << module_info->input_height;

  vdec_attr.enMode = VIDEO_MODE_FRAME;
  // vdec_attr.enMode = VIDEO_MODE_STREAM;
  vdec_attr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
  vdec_attr.u32FrameBufCnt = frameDepth_;
  vdec_attr.u32StreamBufCnt = frameDepth_;  // 5;
  LOGI << "vdec_attr.u32StreamBufCnt:" << vdec_attr.u32StreamBufCnt;
  vdec_attr.u32StreamBufSize = module_info->input_width *
                               module_info->input_height * 3 /
                               2;  // 768 * 1024;  // 1920 * 1088 * 3 / 2;
  LOGI << "vdec_attr.u32StreamBufSize:" << vdec_attr.u32StreamBufSize;
  vdec_attr.bExternalBitStreamBuf = HB_TRUE;
  if (module_info->input_encode_type == RTSP_Payload_H264) {
    vdec_attr.stAttrH264.bandwidth_Opt = HB_TRUE;
    vdec_attr.stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
    vdec_attr.stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
  } else {
    vdec_attr.stAttrH265.bandwidth_Opt = HB_TRUE;
    vdec_attr.stAttrH265.enDecMode = VIDEO_DEC_MODE_NORMAL;
    vdec_attr.stAttrH265.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
  }
  ret = HB_VDEC_CreateChn(group_id_, &vdec_attr);
  if (ret != 0) {
    LOGE << "HB_VDEC_CreateChn Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  }
  ret = HB_VDEC_SetChnAttr(group_id_, &vdec_attr);
  if (ret != 0) {
    LOGE << "HB_VDEC_SetChnAttr Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  }
  return ret;
}

int VdecModule::Start() {
  int ret = 0;
  ret = HB_VDEC_StartRecvStream(group_id_);
  if (ret != 0) {
    LOGE << "HB_VDEC_StartRecvStream Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  }
  return ret;
}

int VdecModule::Input(void *data) {
  int ret = 0;
  VIDEO_STREAM_S *pstStream = (VIDEO_STREAM_S *)data;
  ret = HB_VDEC_SendStream(group_id_, pstStream, timeout_);
  if (ret != 0) {
    LOGE << "HB_VDEC_SendStream Failed. group:" << group_id_
         << " ret = " << ret;
  } else {
    in_fps_++;
    std::chrono::duration<double, std::milli> interval_ms =
            std::chrono::high_resolution_clock::now() - in_start_tp_;
    if (interval_ms.count() >= 1000) {
      LOGI << "vdec in fps " << in_fps_;
      in_fps_ = 0;
      in_start_tp_ = std::chrono::high_resolution_clock::now();
    }
  }
  return ret;
}

int VdecModule::Output(void **data) {
  int ret = 0;
  uint32_t index = buffer_index_ % frameDepth_;
  ret = HB_VDEC_GetFrame(group_id_, &buffers_[index], timeout_);
  if (ret != 0) {
    data = nullptr;
    LOGE << "HB_VDEC_GetFrame Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  } else {
    out_fps_++;
    std::chrono::duration<double, std::milli> interval_ms =
            std::chrono::high_resolution_clock::now() - out_start_tp_;
    if (interval_ms.count() >= 1000) {
      LOGI << "vdec out fps " << out_fps_;
      out_fps_ = 0;
      out_start_tp_ = std::chrono::high_resolution_clock::now();
    }
  }
  LOGD << "HB_VDEC_GetFrame frame width: " << buffers_[index].stVFrame.width
       << " frame height: " << buffers_[index].stVFrame.height
       << " frame size: " << buffers_[index].stVFrame.size;

  *data = &buffers_[index];
  buffer_index_++;

  return ret;
}

int VdecModule::OutputBufferFree(void *data) {
  int ret = 0;
  if (data == nullptr) return -1;
  ret = HB_VDEC_ReleaseFrame(group_id_, (VIDEO_FRAME_S *)data);
  if (ret != 0) {
    LOGE << "HB_VDEC_ReleaseFrame Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  }
  return ret;
}

int VdecModule::Stop() {
  int ret = 0;
  LOGI << "Vdec Stop id: " << group_id_;
  ret = HB_VDEC_StopRecvStream(group_id_);
  if (ret != 0) {
    LOGE << "HB_VDEC_StopRecvStream Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  }
  ret = HB_VDEC_DestroyChn(group_id_);
  if (ret != 0) {
    LOGE << "HB_VENC_DestroyChn Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  }
  return ret;
}

int VdecModule::DeInit() {
  int ret = 0;
  ret = HB_VDEC_DestroyChn(group_id_);
  if (ret != 0) {
    LOGE << "HB_VDEC_DestroyChn Failed. group:" << group_id_
         << " ret = " << ret;
    return ret;
  }
  return ret;
}

}  // namespace vision
}  // namespace horizon