/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#include "venc_module.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iostream>

#include "hobotlog/hobotlog.hpp"
#include "smart_plugin/display_info.h"
#include "visual_plugin/horizon_server_api.h"

extern "C" {
#include "vio/hb_mipi_api.h"
#include "vio/hb_sys.h"
#include "vio/hb_venc.h"
#include "vio/hb_vin_api.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_vp_api.h"
#include "vio/hb_vps_api.h"
}

namespace solution {
namespace video_box {

std::once_flag VencModule::flag_;

VencModule::VencModule() : chn_id_(-1), timeout_(50), pipe_fd_(-1) {
  std::call_once(flag_, []() {
    int ret = HB_VENC_Module_Init();
    if (ret != 0) {
      LOGF << "HB_VENC_Module_Init Failed. ret = " << ret;
    }
  });
}

VencModule::~VencModule() {
  HB_VENC_Module_Uninit();
  if (buffer_1080p_) {
    free(buffer_1080p_);
  }

  if (gdc_buf) {
    free(gdc_buf);
  }
}

int VencModule::VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr,
                                PAYLOAD_TYPE_E p_enType, int p_Width,
                                int p_Height, PIXEL_FORMAT_E pixFmt) {
  int streambuf = 2 * 1024 * 1024;

  memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
  pVencChnAttr->stVencAttr.enType = p_enType;

  pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
  pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

  pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
  pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
  pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
  if (p_Width * p_Height > 2688 * 1522) {
    streambuf = 2 * 1024 * 1024;
  } else if (p_Width * p_Height > 1920 * 1080) {
    streambuf = 1024 * 1024;
  } else if (p_Width * p_Height > 1280 * 720) {
    streambuf = 512 * 1024;
  } else {
    streambuf = 256 * 1024;
  }
  if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
    pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
    pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
    pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
    pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
    pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
    pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 0;
    pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
    pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
  } else {
    pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
    pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 3;
    pVencChnAttr->stVencAttr.u32FrameBufferCount = 3;
    pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
    pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
  }

  if (p_enType == PT_H265) {
    pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
    pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
    pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
    pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 50;
    pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = 25;
  }
  if (p_enType == PT_H264) {
    pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
    pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
    pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
    pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 50;
    pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = 25;
    pVencChnAttr->stVencAttr.stAttrH264.h264_profile = HB_H264_PROFILE_MP;
    pVencChnAttr->stVencAttr.stAttrH264.h264_level = HB_H264_LEVEL1;
  }

  pVencChnAttr->stGopAttr.u32GopPresetIdx = 3;
  pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

  return 0;
}

int VencModule::Init(uint32_t chn_id, const VencModuleInfo *module_info,
                     const int channel_num, const int display_mode) {
  channel_num_ = channel_num;
  display_mode_ = display_mode;

  VENC_CHN_ATTR_S vencChnAttr;
  VENC_RC_ATTR_S *pstRcParam;

  int width = module_info->width;
  int height = module_info->height;
  if (width == 1280 && height == 720) {
    encode_720p_ = true;
    LOGI << "video box init 720p vencmodule";
  } else {
    LOGI << "video box init 1080p vencmodule";
  }

  PAYLOAD_TYPE_E ptype = PT_H264;
  VencChnAttrInit(&vencChnAttr, ptype, width, height, HB_PIXEL_FORMAT_NV12);

  int s32Ret = HB_VENC_CreateChn(chn_id, &vencChnAttr);
  if (s32Ret != 0) {
    printf("HB_VENC_CreateChn %d failed, %d.\n", chn_id, s32Ret);
    return -1;
  }

  if (ptype == PT_H264) {
    pstRcParam = &(vencChnAttr.stRcAttr);
    vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    s32Ret = HB_VENC_GetRcParam(chn_id, pstRcParam);
    if (s32Ret != 0) {
      printf("HB_VENC_GetRcParam failed.\n");
      return -1;
    }

    printf(" vencChnAttr.stRcAttr.enRcMode = %d mmmmmmmmmmmmmmmmmm   \n",
           vencChnAttr.stRcAttr.enRcMode);
    printf(" u32VbvBufferSize = %d mmmmmmmmmmmmmmmmmm   \n",
           vencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize);

    pstRcParam->stH264Cbr.u32BitRate = module_info->bits;
    pstRcParam->stH264Cbr.u32FrameRate = 25;
    pstRcParam->stH264Cbr.u32IntraPeriod = 50;
    pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
  } else if (ptype == PT_H265) {
    pstRcParam = &(vencChnAttr.stRcAttr);
    vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
    s32Ret = HB_VENC_GetRcParam(chn_id, pstRcParam);
    if (s32Ret != 0) {
      printf("HB_VENC_GetRcParam failed.\n");
      return -1;
    }
    printf(" m_VencChnAttr.stRcAttr.enRcMode = %d mmmmmmmmmmmmmmmmmm   \n",
           vencChnAttr.stRcAttr.enRcMode);
    printf(" u32VbvBufferSize = %d mmmmmmmmmmmmmmmmmm   \n",
           vencChnAttr.stRcAttr.stH265Cbr.u32VbvBufferSize);

    pstRcParam->stH265Cbr.u32BitRate = module_info->bits;
    pstRcParam->stH265Cbr.u32FrameRate = 25;
    pstRcParam->stH265Cbr.u32IntraPeriod = 50;
    pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
  } else if (ptype == PT_MJPEG) {
    pstRcParam = &(vencChnAttr.stRcAttr);
    vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
    s32Ret = HB_VENC_GetRcParam(chn_id, pstRcParam);
    if (s32Ret != 0) {
      printf("HB_VENC_GetRcParam failed.\n");
      return -1;
    }
  }

  s32Ret = HB_VENC_SetChnAttr(chn_id, &vencChnAttr);  // config
  if (s32Ret != 0) {
    printf("HB_VENC_SetChnAttr failed\n");
    return -1;
  }

  chn_id_ = chn_id;
  venc_info_.width = module_info->width;
  venc_info_.height = module_info->height;
  venc_info_.type = module_info->type;
  venc_info_.bits = module_info->bits;

  s32Ret = HB_VP_Init();
  if (s32Ret != 0) {
    printf("vp_init fail s32Ret = %d !\n", s32Ret);
  }

  buffers_.mmz_size = venc_info_.width * venc_info_.height * 3 / 2;
  s32Ret = HB_SYS_Alloc(&buffers_.mmz_paddr,
                        reinterpret_cast<void **>(&buffers_.mmz_vaddr),
                        buffers_.mmz_size);
  if (s32Ret == 0) {
    printf("mmzAlloc paddr = 0x%lx, vaddr = 0x%p \n", buffers_.mmz_paddr,
           buffers_.mmz_vaddr);
  }

  if (encode_720p_ &&
      ((display_mode_ == 0 && channel_num_ > 4) || display_mode_ == 1)) {
    InitVPS();
  }

  // char stream_name[100] = {0};
  // sprintf(stream_name, "%s%d%s", "./video_box/output_stream_", chn_id_,
  //         ".h264");
  // outfile_ = fopen(stream_name, "wb");

  return s32Ret;
}

int VencModule::Start() {
  int ret = 0;
  VENC_RECV_PIC_PARAM_S pstRecvParam;
  pstRecvParam.s32RecvPicNum = 0;  // unchangable
  ret = HB_VENC_StartRecvFrame(chn_id_, &pstRecvParam);
  if (ret != 0) {
    LOGE << "HB_VENC_StartRecvStream Failed. ret = " << ret;
    return ret;
  }

  if (buffer_1080p_) {
    ret = HB_VPS_StartGrp(channel_num_);
    if (ret) {
      LOGE << "box vencmodule HB_VPS_StartGrp Failed. ret = " << ret;
      return ret;
    } else {
      LOGI << "box vencmodule HB_VPS_StartGrp：" << channel_num_ << " success!";
    }
  }

  process_running_ = true;
  process_thread_ = std::make_shared<std::thread>(&VencModule::Process, this);
  encode_thread_ = std::make_shared<std::thread>(&VencModule::HandleData, this);

  if (encode_720p_) {
    LOGI << "video box start 720p vencmodule success!";
  } else {
    LOGI << "video box start 1080p vencmodule success!";
  }

  return ret;
}

int VencModule::Stop() {
  int ret = 0;

  process_running_ = false;
  if (process_thread_->joinable()) {
    process_thread_->join();
  }
  if (encode_thread_->joinable()) {
    encode_thread_->join();
  }
  in_queue_.clear();

  LOGE << "VENC Stop id: " << chn_id_;
  ret = HB_VENC_StopRecvFrame(chn_id_);
  if (ret != 0) {
    LOGE << "HB_VENC_StopRecvStream Failed. ret = " << ret;
    return ret;
  }

  if (buffer_1080p_) {
    ret = HB_VPS_StopGrp(channel_num_);
    if (ret) {
      LOGE << "box vencmodule HB_VPS_StopGrp Failed. ret = " << ret;
      return ret;
    }
  }

  return ret;
}

int VencModule::DeInit() {
  int ret = 0;
  ret = HB_VENC_DestroyChn(chn_id_);
  if (ret != 0) {
    LOGE << "HB_VENC_DestroyChn Failed. ret = " << ret;
    return ret;
  }

  if (buffer_1080p_) {
    DeInitVPS();
  }

  return ret;
}

int VencModule::Process() {
  const char *fifo_name = nullptr;

  if (0 == chn_id_) {
    fifo_name = "/tmp/h264_fifo";
  } else {
    fifo_name = "/tmp/h264_fifo1";
  }

  if (access(fifo_name, F_OK) == -1) {
    int res = mkfifo(fifo_name, 0777);
    if (res != 0) {
      LOGE << "mkdir fifo failed!!!!";
      return -1;
    }
  }

  // 会阻塞在这里，直到读取进程打开该fifo
  pipe_fd_ = open(fifo_name, O_WRONLY);
  if (pipe_fd_ == -1) {
    LOGE << "open fifo fail";
    return -1;
  }

  VIDEO_STREAM_S pstStream;
  while (process_running_) {
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
    int ret = HB_VENC_GetStream(chn_id_, &pstStream, timeout_);
    if (ret < 0) {
      printf("HB_VENC_GetStream error!!!\n");
    } else {
      // fwrite(pstStream.pstPack.vir_ptr, 1, pstStream.pstPack.size, outfile_);
      write(pipe_fd_, (unsigned char *)pstStream.pstPack.vir_ptr,
            pstStream.pstPack.size);
      LOGD << "Venc chn " << chn_id_ << " get stream pack size "
           << pstStream.pstPack.size;
      HB_VENC_ReleaseStream(chn_id_, &pstStream);
    }
  }

  return 0;
}

int VencModule::Input(std::shared_ptr<VideoData> video_data, const bool copy) {
  if (!copy) {
    in_queue_.push(video_data);
    return 0;
  }

  auto video_data_new = std::make_shared<solution::video_box::VideoData>();
  HorizonVisionAllocSmartFrame(&video_data_new->smart_frame);
  HorizonVisionCopySmartFrame(video_data->smart_frame,
                              video_data_new->smart_frame);
  video_data_new->channel = video_data->channel;
  video_data_new->width = video_data->width;
  video_data_new->height = video_data->height;
  video_data_new->data_len = video_data->data_len;
  video_data_new->has_plot = video_data->has_plot;
  video_data_new->nv12 = video_data->nv12;
  video_data_new->timestamp = video_data->timestamp;
  video_data_new->buffer = static_cast<char *>(malloc(video_data->data_len));
  memcpy(video_data_new->buffer, video_data->buffer, video_data->data_len);
  if (in_queue_.size() >= in_queue_len_max_) {
    in_queue_.pop();
    if (encode_720p_) {
      LOGE << "VencModule 720p queue is full";
    } else {
      LOGE << "VencModule 1080p queue is full";
    }
  }
  in_queue_.push(video_data_new);
  return 0;
}

int VencModule::HandleData() {
  while (process_running_) {
    std::shared_ptr<VideoData> video_data;
    {
      // std::lock_guard<std::mutex> lk(cache_mtx_);
      auto is_getitem =
          in_queue_.try_pop(&video_data, std::chrono::microseconds(100));
      if (!is_getitem) {
        continue;
      }
    }
    EncodeData(video_data);
  }
  return 0;
}

int VencModule::EncodeData(std::shared_ptr<VideoData> video_data) {
  int pad_x = 0;
  int pad_y = 0;
  uint32_t image_width = 0;
  uint32_t image_height = 0;
  DisplayInfo::computeXYPossition(display_mode_, channel_num_,
                                  video_data->channel, pad_x, pad_y,
                                  encode_720p_);
  DisplayInfo::computeResolution(display_mode_, channel_num_,
                                 video_data->channel, image_width, image_height,
                                 encode_720p_);
  if (image_height != video_data->height || image_width != video_data->width) {
    LOGE << "vencmodule recv dest resolution != src resolution!!!";
    return -1;
  }

  bool resize = false;
  char *img_i420 = video_data->buffer;
  char *des_buffer = buffers_.mmz_vaddr;
  int des_width = 1920;
  int des_height = 1080;
  if (encode_720p_) {
    if (display_mode_ == 0 && channel_num_ <= 4) {
      des_width = 1280;
      des_height = 720;
    } else {
      resize = true;
    }
  }

  if (resize) {  // need to resize
    des_buffer = buffer_1080p_->img_addr.addr[0];
    for (uint32_t i = 0; i < image_height; ++i) {
      memcpy(des_buffer + (i + pad_y) * des_width + pad_x,
             img_i420 + i * image_width, image_width);
    }
    des_buffer = buffer_1080p_->img_addr.addr[1];
    for (uint32_t i = 0; i < image_height / 2; ++i) {
      memcpy(des_buffer + (i + pad_y / 2) * des_width + pad_x,
             img_i420 + image_width * image_height + i * image_width,
             image_width);
    }
    ResizePic();
  } else {
    for (uint32_t i = 0; i < image_height; ++i) {
      memcpy(des_buffer + (i + pad_y) * des_width + pad_x,
             img_i420 + i * image_width, image_width);
    }
    for (uint32_t i = 0; i < image_height / 2; ++i) {
      memcpy(des_buffer + (i + des_height + pad_y / 2) * des_width + pad_x,
             img_i420 + image_width * image_height + i * image_width,
             image_width);
    }
  }

  // 送给编码器编码
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));

  pstFrame.stVFrame.width = venc_info_.width;
  pstFrame.stVFrame.height = venc_info_.height;
  pstFrame.stVFrame.size = buffers_.mmz_size;
  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;

  pstFrame.stVFrame.phy_ptr[0] = buffers_.mmz_paddr;
  pstFrame.stVFrame.phy_ptr[1] =
      buffers_.mmz_paddr + venc_info_.width * venc_info_.height;

  pstFrame.stVFrame.vir_ptr[0] = buffers_.mmz_vaddr;
  pstFrame.stVFrame.vir_ptr[1] =
      buffers_.mmz_vaddr + venc_info_.width * venc_info_.height;
  pstFrame.stVFrame.pts = 0;

  int ret = HB_VENC_SendFrame(chn_id_, &pstFrame, timeout_);
  if (ret != 0) {
    LOGE << "HB_VENC_SendStream Failed. ret = " << ret;
  }

  return 0;
}

int VencModule::InitVPS() {
  VPS_GRP_ATTR_S grp_attr;
  memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
  grp_attr.maxW = 1920;
  grp_attr.maxH = 1080;
  grp_attr.frameDepth = 1;
  int ret = HB_VPS_CreateGrp(channel_num_, &grp_attr);
  if (ret) {
    LOGE << "box vencmodule HB_VPS_CreateGrp error!!!";
    return ret;
  }

  VPS_CHN_ATTR_S chn_attr;
  chn_attr.enScale = 1;
  chn_attr.width = 1280;
  chn_attr.height = 720;
  chn_attr.frameDepth = 1;
  ret = HB_VPS_SetChnAttr(channel_num_, 2, &chn_attr);
  if (ret) {
    LOGE << "box vencmodule HB_VPS_SetChnAttr error!!!";
    return ret;
  }

  HB_VPS_EnableChn(channel_num_, 2);

  buffer_1080p_ =
      reinterpret_cast<hb_vio_buffer_t *>(malloc(sizeof(hb_vio_buffer_t)));
  memset(buffer_1080p_, 0, sizeof(hb_vio_buffer_t));
  buffer_1080p_->img_addr.width = 1920;
  buffer_1080p_->img_addr.height = 1080;
  buffer_1080p_->img_addr.stride_size = 1920;
  buffer_1080p_->img_info.planeCount = 2;
  buffer_1080p_->img_info.img_format = 8;
  ret = HB_SYS_Alloc(
      &buffer_1080p_->img_addr.paddr[0],
      reinterpret_cast<void **>(&buffer_1080p_->img_addr.addr[0]), 1920 * 1080);
  if (ret == 0) {
    printf("mmzAlloc venc 1080p paddr0 = 0x%lx, vaddr = 0x%p \n",
           buffer_1080p_->img_addr.paddr[0], buffer_1080p_->img_addr.addr[0]);
  }
  ret =
      HB_SYS_Alloc(&buffer_1080p_->img_addr.paddr[1],
                   reinterpret_cast<void **>(&buffer_1080p_->img_addr.addr[1]),
                   1920 * 1080 / 2);
  if (ret == 0) {
    printf("mmzAlloc venc 1080p paddr1 = 0x%lx, vaddr = 0x%p \n",
           buffer_1080p_->img_addr.paddr[1], buffer_1080p_->img_addr.addr[1]);
  }
  memset(buffer_1080p_->img_addr.addr[0], 0, 1920 * 1080);
  memset(buffer_1080p_->img_addr.addr[1], 0, 1920 * 1080 / 2);

  gdc_buf =
      reinterpret_cast<hb_vio_buffer_t *>(malloc(sizeof(hb_vio_buffer_t)));
  memset(gdc_buf, 0, sizeof(hb_vio_buffer_t));
  return 0;
}

int VencModule::ResizePic() {
  int timeout = 40;
  int ret = HB_VPS_SendFrame(channel_num_, buffer_1080p_, timeout_);
  if (ret != 0) {
    LOGE << "HB_VPS_SendFrame Failed. ret = " << ret;
    return ret;
  }

  memset(gdc_buf, 0, sizeof(hb_vio_buffer_t));
  ret = HB_VPS_GetChnFrame(channel_num_, 2, gdc_buf, timeout);
  if (ret != 0) {
    LOGE << "Get Chn GDC frame failed!, ret:" << ret;
    return ret;
  }

  int size = gdc_buf->img_addr.width * gdc_buf->img_addr.height;
  int size1 = gdc_buf->img_addr.width * gdc_buf->img_addr.height / 2;
  int width = gdc_buf->img_addr.width;
  int height = gdc_buf->img_addr.height;
  char *buffer = buffers_.mmz_vaddr;
  char *src_buf = gdc_buf->img_addr.addr[0];
  char *src_buf1 = gdc_buf->img_addr.addr[1];
  int stride = gdc_buf->img_addr.stride_size;
  int i = 0;

  if (width == stride) {
    memcpy(buffer, src_buf, size);
    memcpy(buffer + size, src_buf1, size1);
  } else {
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy(buffer + i * width, src_buf + i * stride, width);
    }

    // jump over stride - width UV
    for (i = 0; i < height / 2; i++) {
      memcpy(buffer + size + i * width, src_buf1 + i * stride, width);
    }
  }

  HB_VPS_ReleaseChnFrame(channel_num_, 2, gdc_buf);
  return 0;
}

int VencModule::DeInitVPS() {
  int ret = HB_VPS_DestroyGrp(channel_num_);
  if (ret) {
    LOGE << "box vencmodule HB_VPS_DestroyGrp Failed. ret = " << ret;
  }
  return ret;
}

}  // namespace video_box
}  // namespace solution
