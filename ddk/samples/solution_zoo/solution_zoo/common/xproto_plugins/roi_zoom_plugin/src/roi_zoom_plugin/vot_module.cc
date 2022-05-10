/*
 * @Description:
 * @Author: xx@horizon.ai
 * @Date: 2020-12-19 16:17:25
 * @Copyright Horizon Robotics, Inc.
 */

#include "roi_zoom_plugin/vot_module.h"

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <fstream>
#include <vector>

#include "hobotlog/hobotlog.hpp"

namespace xproto {
VotModule::VotModule() : stop_(true) {}

VotModule::~VotModule() {}

int VotModule::Init() {
  VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
  VOT_CHN_ATTR_S stChnAttr;
  VOT_CROP_INFO_S cropAttrs;
  VOT_PUB_ATTR_S devAttr;

  devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
  devAttr.u32BgColor = 0x108080;
  devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;
  int ret = HB_VOT_SetPubAttr(0, &devAttr);
  if (ret) {
    LOGE << "HB_VOT_SetPubAttr failed";
    return -1;
  }
  ret = HB_VOT_Enable(0);
  if (ret) LOGE << "HB_VOT_Enable failed";

  ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
  if (ret) {
    LOGE << "HB_VOT_GetVideoLayerAttr failed.";
  }

  stLayerAttr.stImageSize.u32Width = 1920;
  stLayerAttr.stImageSize.u32Height = 1080;
  stLayerAttr.panel_type = 0;
  stLayerAttr.rotate = 0;
  stLayerAttr.dithering_flag = 0;
  stLayerAttr.dithering_en = 0;
  stLayerAttr.gamma_en = 0;
  stLayerAttr.hue_en = 0;
  stLayerAttr.sat_en = 0;
  stLayerAttr.con_en = 0;
  stLayerAttr.bright_en = 0;
  stLayerAttr.theta_sign = 0;
  stLayerAttr.contrast = 0;
  stLayerAttr.theta_abs = 0;
  stLayerAttr.saturation = 0;
  stLayerAttr.off_contrast = 0;
  stLayerAttr.off_bright = 0;
  stLayerAttr.user_control_disp = 0;
  stLayerAttr.big_endian = 0;
  stLayerAttr.display_addr_type = 2;
  stLayerAttr.display_addr_type_layer1 = 2;
  ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
  if (ret) LOGE << "HB_VOT_SetVideoLayerAttr failed";

  ret = HB_VOT_EnableVideoLayer(0);
  if (ret) LOGE << "HB_VOT_EnableVideoLayer failed";

  stChnAttr.u32Priority = 2;
  stChnAttr.s32X = 0;
  stChnAttr.s32Y = 0;
  stChnAttr.u32SrcWidth = 1920;
  stChnAttr.u32SrcHeight = 1080;
  stChnAttr.u32DstWidth = 1920;
  stChnAttr.u32DstHeight = 1080;
  buffer_ = static_cast<char*>(malloc(1920 * 1080 * 3 / 2));

  ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
  LOGI << "HB_VOT_SetChnAttr 0:" << ret;
  cropAttrs.u32Width = stChnAttr.u32DstWidth;
  cropAttrs.u32Height = stChnAttr.u32DstHeight;
  ret = HB_VOT_SetChnCrop(0, 0, &cropAttrs);
  init_ = true;
  return ret;
}

int VotModule::Start() {
  if (!stop_) return 0;
  stop_ = false;
  plot_task_ = std::thread(&VotModule::HandleData, this);
  LOGI << "usb roi VotModule Start success!!!";
  return 0;
}

int VotModule::Stop() {
  if (stop_) return 0;
  LOGI << "usb roi VotModule Stop begin";
  stop_ = true;
  // condition_.notify_all();
  if (plot_task_.joinable()) {
    plot_task_.join();
  }
  in_queue_.clear();
  LOGI << "usb roi VotModule Stop end !!!";
  return 0;
}

int VotModule::DeInit() {
  if (!init_) return 0;
  LOGI << "usb roi VotModule Deinit begin";
  int ret = 0;
  ret = HB_VOT_DisableChn(0, 0);
  if (ret) printf("HB_VOT_DisableChn failed.\n");

  ret = HB_VOT_DisableVideoLayer(0);
  if (ret) printf("HB_VOT_DisableVideoLayer failed.\n");

  ret = HB_VOT_Disable(0);
  if (ret) printf("HB_VOT_Disable failed.\n");
  if (buffer_) free(buffer_);
  init_ = false;
  LOGI << "usb roi VotModule Deinit end !!!";
  return ret;
}

int VotModule::HandleData() {
  while (!stop_) {
    std::shared_ptr<VideoRoiData> video_data;
    {
      // std::lock_guard<std::mutex> lk(cache_mtx_);
      auto is_getitem =
          in_queue_.try_pop(&video_data, std::chrono::microseconds(100));
      if (!is_getitem) {
        continue;
      }
    }

    PlotImage(video_data);
    VOT_FRAME_INFO_S stFrame = {};
    stFrame.addr = buffer_;
    stFrame.size = 1920 * 1080 * 3 / 2;
    HB_VOT_SendFrame(0, 0, &stFrame, -1);
  }
  return 0;
#if 0
  while (!stop_) {
    std::map<uint64_t, std::shared_ptr<VideoRoiData>>::iterator front;
    std::shared_ptr<VideoRoiData> video_data = nullptr;

    std::unique_lock<std::mutex> lg(mut_cache_);
    condition_.wait(lg,
                    [this]() { return !is_running_ || !video_list_.empty(); });

    if (!is_running_) {
      video_list_.clear();
      break;
    }

    if (video_list_.empty()) {
      continue;
    }
    front = video_list_.begin();
    {
      video_data = front->second;
      video_list_.erase(front);
      lg.unlock();
      PlotImage(video_data);
      VOT_FRAME_INFO_S stFrame = {};
      stFrame.addr = buffer_;
      stFrame.size = 1920 * 1080 * 3 / 2;
      HB_VOT_SendFrame(0, 0, &stFrame, -1);
      continue;
    }

    LOGV << "cache_roi_data_ size:" << video_list_.size();

    if (video_list_.size() >= cache_len_limit_) {
      // exception occurred
      video_data = front->second;
      video_list_.erase(front);
      lg.unlock();
      PlotImage(video_data);
      VOT_FRAME_INFO_S stFrame = {};
      stFrame.addr = buffer_;
      stFrame.size = 1920 * 1080 * 3 / 2;
      HB_VOT_SendFrame(0, 0, &stFrame, -1);
      continue;
    }
  }

  return 0;
#endif
}

void VotModule::PlotImage(const std::shared_ptr<VideoRoiData>& vot_data) {
  int x_offset = 0, y_offset = 0;
  int channel_id = vot_data->channel;
  if (channel_id == 0) {
    x_offset = 0;
    y_offset = 0;
  } else if (channel_id == 1) {  // 智能取景
    x_offset = 960;
    y_offset = 0;
  } else if (channel_id == 2) {  // 单人发言
    x_offset = 0;
    y_offset = 540;
  } else if (channel_id == 3) {  // 双人发言
    x_offset = 960;
    y_offset = 540;
  } else if (channel_id == 4) {  // 双人发言
    x_offset = 1440;
    y_offset = 540;
  } else {
    LOGE << "display mode 0 recv channle id:" << channel_id << " error!";
    x_offset = 0;
    y_offset = 0;
  }

  uint32_t image_width = vot_data->width;
  uint32_t image_height = vot_data->height;
  // printf("_____________vo plot data, channel:%d, width:%d, height:%d\n",
  // channel_id, image_width, image_height);
  for (uint32_t i = 0; i < image_height; ++i) {
    memcpy(buffer_ + (i + y_offset) * 1920 + x_offset,
           vot_data->y_virtual_addr + i * image_width, image_width);
  }
  for (uint32_t i = 0; i < image_height / 2; ++i) {
    memcpy(buffer_ + (i + 1080 + y_offset / 2) * 1920 + x_offset,
           vot_data->uv_virtual_addr + i * image_width, image_width);
  }
}

int VotModule::Input(const std::shared_ptr<VideoRoiData>& vot_data) {
  if (stop_) return 0;
  HOBOT_CHECK(vot_data);
  if (vot_data->y_virtual_addr == NULL || vot_data->uv_virtual_addr == NULL) {
    return 0;
  }

  if (in_queue_.size() >= in_queue_len_max_) {
    in_queue_.pop();
    LOGE << "vot queue is full, drop frame_id " << vot_data->frame_id;
  }

  in_queue_.push(std::move(vot_data));
  return 0;
#if 0
  std::lock_guard<std::mutex> lg(mut_cache_);
  uint64_t frame_id = video_data->frame_id;
  video_list_[frame_id] = video_data;
  condition_.notify_one();
  LOGV << "cache_vio_smart_ size:" << video_list_.size();
  return 0;
#endif
}

}  // namespace xproto
