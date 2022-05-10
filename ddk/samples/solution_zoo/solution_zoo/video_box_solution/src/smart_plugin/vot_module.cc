/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @file votmodule.cpp
 * @brief vot module
 * @author kairui.wang
 * @email kairui.wang@horizon.ai
 * @date 2020/07/22
 */
#include "vot_module.h"

#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "libyuv/convert.h"
#include "opencv2/opencv.hpp"
#include "plot_smart_data.h"
#include "smart_plugin/display_info.h"
#include "vio/hb_vot.h"

// #define PRINTF_USE_TIME

namespace solution {
namespace video_box {

uint64_t VotModule::frame_id_ = 0;
std::mutex VotModule::frame_id_mtx_;

VotModule::VotModule()
    : group_id_(-1),
      timeout_(40),
      display_mode_(0),
      channel_num_(0),
      frame_interval_(400),
      frame_transition_(200),
      display_chn_hide_(false) {}

VotModule::~VotModule() {}

int VotModule::Init(uint32_t group_id, const smart_vo_cfg_t &smart_vo_cfg) {
  vo_plot_cfg_ = smart_vo_cfg;
  LOGW << "vo_plot_cfg  box_face_thr: " << vo_plot_cfg_.box_face_thr
       << "  box_head_thr:" << vo_plot_cfg_.box_head_thr
       << "  box_body_thr:" << vo_plot_cfg_.box_body_thr
       << "  lmk_thr:" << vo_plot_cfg_.lmk_thr
       << "  kps_thr:" << vo_plot_cfg_.kps_thr
       << "  box_veh_thr:" << vo_plot_cfg_.box_veh_thr
       << "  plot_fps:" << vo_plot_cfg_.plot_fps;

  int ret = 0;
  image_height_ = 540;
  image_width_ = 960;

  VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
  VOT_CHN_ATTR_S stChnAttr;
  VOT_CROP_INFO_S cropAttrs;
  VOT_PUB_ATTR_S devAttr;

  devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
  devAttr.u32BgColor = 0x108080;
  devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;
  ret = HB_VOT_SetPubAttr(0, &devAttr);
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
  buffer_ = static_cast<char *>(malloc(1920 * 1080 * 3 / 2));

  ret = HB_VOT_SetChnAttr(0, 0, &stChnAttr);
  LOGI << "HB_VOT_SetChnAttr 0:" << ret;
  cropAttrs.u32Width = stChnAttr.u32DstWidth;
  cropAttrs.u32Height = stChnAttr.u32DstHeight;
  ret = HB_VOT_SetChnCrop(0, 0, &cropAttrs);

  if (vo_plot_cfg_.transition_support) {
    stChnAttr.u32Priority = 3;
    stChnAttr.s32X = 0;
    stChnAttr.s32Y = 0;
    stChnAttr.u32SrcWidth = 1920;
    stChnAttr.u32SrcHeight = 1080;
    stChnAttr.u32DstWidth = 1920;
    stChnAttr.u32DstHeight = 1080;
    buffer1_ = static_cast<char *>(malloc(1920 * 1080 * 3 / 2));

    ret = HB_VOT_SetChnAttr(0, 1, &stChnAttr);
    printf("HB_VOT_SetChnAttr 1: %d\n", ret);
    cropAttrs.u32Width = stChnAttr.u32DstWidth;
    cropAttrs.u32Height = stChnAttr.u32DstHeight;
    ret = HB_VOT_SetChnCrop(0, 1, &cropAttrs);
  }

  ParseLogoImg("./video_box/configs/jisuanhe-top@2x.png",
               "./video_box/configs/aionhorizon@2x.png");
  ParseBottomLogoImg("./video_box/configs/aionhorizon-left@1x.png",
                     "./video_box/configs/aionhorizon-right@1x.png");
  init_ = true;
  return ret;
}

int VotModule::Input(std::shared_ptr<VideoData> video_data,
                     const bool transition) {
  if (transition && vo_plot_cfg_.transition_support) {
    return Transition(video_data);
  }

  if (in_queue_.size() >= in_queue_len_max_) {
    in_queue_.pop();
    LOGE << "VotModule data queue is full";
  }
  in_queue_.push(video_data);
  return 0;
}

int VotModule::Start() {
  if (start_) return 0;

  HB_VOT_EnableChn(0, 0);
  if (vo_plot_cfg_.transition_support) HB_VOT_EnableChn(0, 1);

  running_ = true;
  plot_task_ = std::thread(&VotModule::HandleData, this);
  start_ = true;
  return 0;
}

int VotModule::Stop() {
  running_ = false;
  start_ = false;
  if (plot_task_.joinable()) {
    plot_task_.join();
  }
  in_queue_.clear();

  int ret = 0;
  ret = HB_VOT_DisableChn(0, 0);
  if (ret) LOGE << "HB_VOT_DisableChn failed.";

  if (vo_plot_cfg_.transition_support) {
    ret = HB_VOT_DisableChn(0, 1);
    if (ret) LOGE << "HB_VOT_DisableChn 1 failed.";
  }

  ret = HB_VOT_DisableVideoLayer(0);
  if (ret) LOGE << "HB_VOT_DisableVideoLayer failed.";

  ret = HB_VOT_Disable(0);
  if (ret) LOGE << "HB_VOT_Disable failed.";
  return ret;
}

int VotModule::DeInit() {
  if (buffer_) free(buffer_);
  if (buffer1_) free(buffer1_);
  return 0;
}

int VotModule::HandleData() {
  while (running_) {
    std::shared_ptr<VideoData> video_data;
    {
      // std::lock_guard<std::mutex> lk(cache_mtx_);
      auto is_getitem =
          in_queue_.try_pop(&video_data, std::chrono::microseconds(100));
      if (!is_getitem) {
        continue;
      }
    }
    if (!video_data->has_plot && video_data->smart_frame) {
      PlotSmartData objplot;
      objplot.PlotData(vo_plot_cfg_, video_data);
    }

    auto image_data_size_ = video_data->width * video_data->height * 3;
    uint8_t *bgr_buf = new uint8_t[image_data_size_ / 2 + image_data_size_ * 2];
    cv::Mat bgr(video_data->height, video_data->width, CV_8UC3, bgr_buf);
    cv::cvtColor(cv::Mat(video_data->height * 3 / 2, video_data->width, CV_8UC1,
                         video_data->buffer),
                 bgr, CV_YUV2BGR_NV12);
    DrawLogo(&bgr, video_data->channel);
    bool resize = false;
    DataToBuffer(video_data->width, video_data->height, bgr, buffer_,
                 video_data->channel, resize);
    delete[] bgr_buf;

    VOT_FRAME_INFO_S stFrame = {};
    stFrame.addr = buffer_;
    stFrame.size = 1920 * 1080 * 3 / 2;
    HB_VOT_SendFrame(0, 0, &stFrame, -1);
  }
  return 0;
}

void VotModule::bgr_to_nv12(uint8_t *bgr, uint8_t *buf, const uint32_t width,
                            const uint32_t height) {
  int uv_height = height / 2;
  int uv_width = width / 2;
  int uv_size = uv_height * uv_width;
  uint8_t *uv_data = buf + (uv_size << 2);
  uint8_t *uv_data_store = bgr + (uv_size << 2) * 3;
  libyuv::RGB24ToI420(bgr, uv_width * 6, buf, uv_width * 2, uv_data_store,
                      uv_width, uv_data_store + uv_size, uv_width, uv_width * 2,
                      uv_height * 2);
  // copy uv data
  for (int i = 0; i < uv_size; ++i) {
    *(uv_data++) = *(uv_data_store + i);
    *(uv_data++) = *(uv_data_store + uv_size + i);
  }
}

void VotModule::DataToBuffer(const uint32_t src_width,
                             const uint32_t src_height, cv::Mat &bgr_mat,
                             char *buf, int channel, bool &resize) {
  int pad_x = 0;
  int pad_y = 0;
  uint32_t image_width = 0;
  uint32_t image_height = 0;
  DisplayInfo::computeXYPossition(display_mode_, channel_num_, channel, pad_x,
                                  pad_y);
  DisplayInfo::computeResolution(display_mode_, channel_num_, channel,
                                 image_width, image_height);
  uint8_t *img_i420 =
      reinterpret_cast<uint8_t *>(malloc(image_width * image_height * 3 / 2));
  if (src_width != image_width || src_height != image_height) {
    LOGW << "display mode:" << display_mode_
         << " src resolution and dst resolution not equal, need to resize!!!";
    cv::resize(bgr_mat, bgr_mat, cv::Size(image_width, image_height));
    resize = true;
  }

  bgr_to_nv12(bgr_mat.ptr<uint8_t>(), img_i420, image_width, image_height);
  for (uint32_t i = 0; i < image_height; ++i) {
    memcpy(buf + (i + pad_y) * 1920 + pad_x, img_i420 + i * image_width,
           image_width);
  }
  for (uint32_t i = 0; i < image_height / 2; ++i) {
    memcpy(buf + (i + 1080 + pad_y / 2) * 1920 + pad_x,
           img_i420 + image_width * image_height + i * image_width,
           image_width);
  }

  free(img_i420);
  return;
}

void VotModule::padding_logo(char *data, int pad_width, int pad_height) {
  // padding yuv420_mat to 1080P data
  // the start padding position is (pad_x, pad_y)
  // yuv420_mat size is mat_width and mat_height
  auto padding = [this, &data, &pad_width, &pad_height](
                     const cv::Mat &yuv420_mat, const int &pad_x,
                     const int &pad_y, const int &mat_width,
                     const int &mat_height) {
    uint32_t in_offset = 0;
    uint32_t out_offset = pad_y * pad_width + pad_x;
    // padding Y
    for (auto idx = 0; idx < mat_height; idx++) {
      memcpy(&data[out_offset], &yuv420_mat.data[in_offset], mat_width);
      in_offset += mat_width;
      out_offset += pad_width;
    }
    // padding UV
    // has UV data
    int uv_height = mat_height / 2;
    int uv_width = mat_width / 2;
    int uv_stride = uv_height * uv_width;
    out_offset = pad_width * pad_height + pad_y / 2 * pad_width + pad_x;
    uint8_t *uv_ptr = yuv420_mat.data + in_offset;
    for (int i = 0; i < uv_height; i++) {
      for (int j = 0; j < uv_width; j++) {
        data[out_offset++] = *(uv_ptr + i * uv_width + j);
        data[out_offset++] = *(uv_ptr + uv_stride + i * uv_width + j);
      }
      out_offset = pad_width * pad_height + (pad_y / 2 + i) * pad_width + pad_x;
    }
  };

  // top
  if (!logo_img_cache_.top_yuv_mat_.empty()) {
    padding(logo_img_cache_.top_yuv_mat_, 0, 0,
            logo_img_cache_.top_image_width_,
            logo_img_cache_.top_image_height_);
  } else {
    LOGI << "no top logo";
  }

  // bottom
  if (!logo_img_cache_.bottom_yuv_mat_.empty()) {
    padding(logo_img_cache_.bottom_yuv_mat_,
            (pad_width - logo_img_cache_.bottom_image_width_) / 2,
            pad_height - logo_img_cache_.bottom_image_height_,
            logo_img_cache_.bottom_image_width_,
            logo_img_cache_.bottom_image_height_);
  } else {
    LOGI << "no bottom logo";
  }
  return;
}

int VotModule::ParseBottomLogoImg(const std::string &file_name_bottom_left,
                                  const std::string &file_name_bottom_rigth) {
  logo_img_cache_.bottom_bgr_mat_left_ =
      cv::imread(file_name_bottom_left, CV_LOAD_IMAGE_UNCHANGED);
  if (!logo_img_cache_.bottom_bgr_mat_left_.data) {
    LOGE << "Failed to call imread for " << file_name_bottom_left;
    return -1;
  }
  logo_img_cache_.bottom_bgr_mat_right_ =
      cv::imread(file_name_bottom_rigth, CV_LOAD_IMAGE_UNCHANGED);
  if (!logo_img_cache_.bottom_bgr_mat_right_.data) {
    LOGE << "Failed to call imread for " << file_name_bottom_rigth;
    return -1;
  }

  return 0;
}

int VotModule::ParseLogoImg(const std::string &file_name_top,
                            const std::string &file_name_bottom, int pad_width,
                            int pad_height) {
  auto bgr_mat_top = cv::imread(file_name_top);
  if (!bgr_mat_top.data) {
    LOGE << "Failed to call imread for " << file_name_top;
    return -1;
  }
  auto bgr_mat_bottom = cv::imread(file_name_bottom);
  if (!bgr_mat_bottom.data) {
    LOGE << "Failed to call imread for " << file_name_bottom;
    return -1;
  }

  auto ori_width = bgr_mat_top.cols;
  auto ori_height = bgr_mat_top.rows;
  if (ori_width > pad_width || ori_height > pad_height) {
    auto aspect_ratio = ori_width / ori_height;
    auto dst_ratio = static_cast<float>(pad_width) / ori_height;
    uint32_t resized_width = -1;
    uint32_t resized_height = -1;
    // 等比缩放
    if (aspect_ratio >= dst_ratio) {
      resized_width = pad_width;
      resized_height =
          static_cast<uint32_t>(ori_height * pad_width / ori_width);
    } else {
      resized_width =
          static_cast<uint32_t>(ori_width * pad_height / ori_height);
      resized_height = pad_height;
    }
    // mat should allign with 2
    cv::resize(bgr_mat_top, bgr_mat_top,
               cv::Size(resized_width / 2 * 2, resized_height / 2 * 2));
  }
  logo_img_cache_.top_bgr_mat_ = bgr_mat_top;
  if (display_mode_ == 0) {
    if (channel_num_ == 1) {
      logo_img_cache_.top_bgr_mat_left_ = bgr_mat_top;
    } else if (channel_num_ > 1 && channel_num_ <= 4) {
      logo_img_cache_.top_bgr_mat_left_ =
          bgr_mat_top(cv::Rect(0, 0, 960, bgr_mat_top.rows));
      logo_img_cache_.top_bgr_mat_right_ =
          bgr_mat_top(cv::Rect(960, 0, 960, bgr_mat_top.rows));
    } else {
      logo_img_cache_.top_bgr_mat_left_ =
          bgr_mat_top(cv::Rect(0, 0, 640, bgr_mat_top.rows));
      logo_img_cache_.top_bgr_mat_mid_ =
          bgr_mat_top(cv::Rect(640, 0, 640, bgr_mat_top.rows));
      logo_img_cache_.top_bgr_mat_right_ =
          bgr_mat_top(cv::Rect(1280, 0, 640, bgr_mat_top.rows));
    }
  } else {  // 9窗口
    logo_img_cache_.top_bgr_mat_left_ =
        bgr_mat_top(cv::Rect(0, 0, 640, bgr_mat_top.rows));
    logo_img_cache_.top_bgr_mat_mid_ =
        bgr_mat_top(cv::Rect(640, 0, 640, bgr_mat_top.rows));
    logo_img_cache_.top_bgr_mat_right_ =
        bgr_mat_top(cv::Rect(1280, 0, 640, bgr_mat_top.rows));
  }

  logo_img_cache_.top_image_width_ = bgr_mat_top.cols;
  logo_img_cache_.top_image_height_ = bgr_mat_top.rows;
  cv::cvtColor(bgr_mat_top, logo_img_cache_.top_yuv_mat_,
               cv::COLOR_BGR2YUV_I420);

  if (display_mode_ == 0) {
    if (channel_num_ == 1) {
      logo_img_cache_.top_bgr_mat_left_ = bgr_mat_top;
    } else if (channel_num_ > 1 && channel_num_ <= 4) {
      logo_img_cache_.top_bgr_mat_left_ =
          bgr_mat_top(cv::Rect(0, 0, 960, bgr_mat_top.rows));
      logo_img_cache_.top_bgr_mat_right_ =
          bgr_mat_top(cv::Rect(960, 0, 960, bgr_mat_top.rows));
    } else {
      logo_img_cache_.top_bgr_mat_left_ =
          bgr_mat_top(cv::Rect(0, 0, 640, bgr_mat_top.rows));
      logo_img_cache_.top_bgr_mat_mid_ =
          bgr_mat_top(cv::Rect(640, 0, 640, bgr_mat_top.rows));
      logo_img_cache_.top_bgr_mat_right_ =
          bgr_mat_top(cv::Rect(1280, 0, 640, bgr_mat_top.rows));
    }
  } else {  // 9窗口
    logo_img_cache_.top_bgr_mat_left_ =
        bgr_mat_top(cv::Rect(0, 0, 640, bgr_mat_top.rows));
    logo_img_cache_.top_bgr_mat_mid_ =
        bgr_mat_top(cv::Rect(640, 0, 640, bgr_mat_top.rows));
    logo_img_cache_.top_bgr_mat_right_ =
        bgr_mat_top(cv::Rect(1280, 0, 640, bgr_mat_top.rows));
  }

  // crop bottom logo from trapezium to rectangle
  int w_offset = 64;
  cv::Rect rect(w_offset, 0, bgr_mat_bottom.cols - w_offset * 2,
                bgr_mat_bottom.rows);
  cv::Mat new_bottom = bgr_mat_bottom(rect);
  logo_img_cache_.bottom_image_width_ = new_bottom.cols;
  logo_img_cache_.bottom_image_height_ = new_bottom.rows;
  cv::cvtColor(new_bottom, logo_img_cache_.bottom_yuv_mat_,
               cv::COLOR_BGR2YUV_I420);
  cv::resize(new_bottom, logo_img_cache_.bottom_bgr_mat_,
             cv::Size(new_bottom.cols / 8 * 2, new_bottom.rows / 8 * 2));

  return 0;
}

int VotModule::PlotFont(char *y_mat, const char *font_buf, int x0, int y0,
                        int bg_width, int bg_height) {
  if (!y_mat || !font_buf) {
    LOGE << "plot font fail! mat/font invalid";
    return -1;
  }
  static int32_t tagY = 226;
  //操作屏幕
  for (int index = 0; index < w_font_ * h_font_ / 8; index++) {
    for (int i = 7; i >= 0; i--) {
      if (font_buf[index] & (1 << i)) {
        int x = 8 * (index % (w_font_ / 8)) + 7 - i + x0;
        int y = index / (w_font_ / 8) + y0;
        if (x >= bg_width || y >= bg_height) {
          continue;
        }

        *(y_mat + bg_width * y + x) = tagY;
        *(y_mat + y * bg_width + x + 1) = tagY;
        *(y_mat + (y + 1) * bg_width + x) = tagY;
        *(y_mat + (y + 1) * bg_width + x + 1) = tagY;
      }
    }
  }
  return 0;
}

void VotModule::DrawLogo(cv::Mat *bgr, const int channel) {
  if (display_mode_ == 0 && channel_num_ == 1) {
    if (logo_img_cache_.top_bgr_mat_.data) {
      Drawlogo(logo_img_cache_.top_bgr_mat_, bgr, 0);
    }
    // bottom
    if (logo_img_cache_.bottom_bgr_mat_left_.data) {
      Drawlogo(logo_img_cache_.top_bgr_mat_left_, bgr, 1, 0);
    }
  } else if (display_mode_ == 0 && (channel_num_ > 1 && channel_num_ <= 4)) {
    if (logo_img_cache_.top_bgr_mat_left_.data &&
        logo_img_cache_.top_bgr_mat_right_.data) {
      if (0 == channel) {
        Drawlogo(logo_img_cache_.top_bgr_mat_left_, bgr, 0);
      } else if (1 == channel) {
        Drawlogo(logo_img_cache_.top_bgr_mat_right_, bgr, 0);
      }
    }

    // bottom
    if (logo_img_cache_.bottom_bgr_mat_left_.data &&
        logo_img_cache_.bottom_bgr_mat_right_.data) {
      if (2 == channel) {
        Drawlogo(logo_img_cache_.bottom_bgr_mat_left_, bgr, 1, 0);
      } else if (3 == channel) {
        Drawlogo(logo_img_cache_.bottom_bgr_mat_right_, bgr, 1, 1);
      }
    }
  } else {
    if (logo_img_cache_.top_bgr_mat_left_.data &&
        logo_img_cache_.top_bgr_mat_mid_.data &&
        logo_img_cache_.top_bgr_mat_right_.data) {
      if (0 == channel) {
        Drawlogo(logo_img_cache_.top_bgr_mat_left_, bgr, 0);
      } else if (1 == channel) {
        Drawlogo(logo_img_cache_.top_bgr_mat_mid_, bgr, 0);
      } else if (2 == channel) {
        Drawlogo(logo_img_cache_.top_bgr_mat_right_, bgr, 0);
      }
    }

    // bottom
    if (logo_img_cache_.bottom_bgr_mat_left_.data &&
        logo_img_cache_.bottom_bgr_mat_right_.data) {
      if (6 == channel) {
        Drawlogo(logo_img_cache_.bottom_bgr_mat_left_, bgr, 1, 0);
      } else if (7 == channel) {
        Drawlogo(logo_img_cache_.bottom_bgr_mat_mid_, bgr, 1, 1);
      } else if (8 == channel) {
        Drawlogo(logo_img_cache_.bottom_bgr_mat_right_, bgr, 1, 1);
      }
    }
  }
}

int VotModule::Drawlogo(const cv::Mat &logo, cv::Mat *bgr, int position,
                        int left_right) {
  if (logo.data == nullptr || !bgr) {
    return -1;
  }

  cv::Point point_x;
  cv::Point point_y;
  if (position == 0) {  //  top
    point_x.x = 0;
    point_x.y = 0;
  } else if (position == 1) {  //  bottom_middle
    if (0 == left_right) {
      if (display_mode_ == 0 && channel_num_ == 1) {
        point_x.x = 1920 - logo.cols;
        point_x.y = 1080 - logo.rows;
      } else if (display_mode_ == 0 && channel_num_ > 1 && channel_num_ <= 4) {
        point_x.x = 960 - logo.cols;
        point_x.y = 540 - logo.rows;
      } else {
        point_x.x = 640 - logo.cols;
        point_x.y = 360 - logo.rows;
      }
    } else if (1 == left_right || 2 == left_right) {
      point_x.x = 0;
      if (display_mode_ == 0 && channel_num_ == 1) {
        point_x.y = 1080 - logo.rows;
      } else if (display_mode_ == 0 && channel_num_ > 1 && channel_num_ <= 4) {
        point_x.y = 540 - logo.rows;
      } else {
        point_x.y = 360 - logo.rows;
      }
    }
  }

  point_y.x = logo.cols;
  point_y.y = logo.rows;
  point_y += point_x;
  auto bgr_roi = (*bgr)(cv::Rect(point_x, point_y));
  std::vector<cv::Mat> logo_channels;
  std::vector<cv::Mat> bgr_channels;
  cv::split(logo, logo_channels);
  cv::split(bgr_roi, bgr_channels);
  if (logo_channels.size() == 4) {
    for (int i = 0; i < 3; i++) {
      bgr_channels[i] = bgr_channels[i].mul(255.0 - logo_channels[3], 0.003921);
      bgr_channels[i] += logo_channels[i].mul(logo_channels[3], 0.003921);
    }
    merge(bgr_channels, bgr_roi);
  } else {
    logo.copyTo(bgr_roi);
  }
  return 0;
}

int VotModule::Transition(std::shared_ptr<VideoData> video_data) {
  uint32_t frame_index = 0;
  {
    std::lock_guard<std::mutex> lg(frame_id_mtx_);
    frame_index = frame_id_++ % (channel_num_ * frame_interval_);
  }

  auto index_chn = frame_index % frame_interval_;
  auto chn_display = frame_index / frame_interval_;
  auto chn_display_pre = (chn_display + channel_num_ - 1) % channel_num_;

  if (index_chn <= frame_transition_ && frame_id_ >= frame_interval_) {
    if (video_data->channel == chn_display_pre) {
      memcpy(buffer1_, video_data->y_virtual_addr, 1920 * 1080);
      memcpy(buffer1_ + 1920 * 1080, video_data->uv_virtual_addr,
             1920 * 1080 / 2);

      VOT_FRAME_INFO_S stFrame = {0};
      stFrame.addr = buffer1_;
      stFrame.size = 1920 * 1080 * 3 / 2;
      HB_VOT_SendFrame(0, 1, &stFrame, -1);
    } else if (video_data->channel == chn_display) {
      memcpy(buffer_, video_data->y_virtual_addr, 1920 * 1080);
      memcpy(buffer_ + 1920 * 1080, video_data->uv_virtual_addr,
             1920 * 1080 / 2);

      VOT_FRAME_INFO_S stFrame = {0};
      stFrame.addr = buffer_;
      stFrame.size = 1920 * 1080 * 3 / 2;
      HB_VOT_SendFrame(0, 0, &stFrame, -1);
      if (display_chn_hide_) {
        HB_VOT_ShowChn(0, 0);
        display_chn_hide_ = false;
      }
    }

    // 右下->左上
    POINT_S display_point = {0};
    display_point.s32X = 1920 - index_chn * 1920 / frame_transition_;
    display_point.s32Y = 1080 - index_chn * 1080 / frame_transition_;
    HB_VOT_SetChnDisplayPosition(0, 0, &display_point);

    // 左下->右上
    // POINT_S display_point = {0};
    // display_point.s32X = 0;
    // display_point.s32Y = 1080 - index_chn * 1080 / frame_transition_;
    // ret = HB_VOT_SetChnDisplayPosition(0, 0, &display_point);

    // VOT_CROP_INFO_S crop_info = {0};
    // crop_info.u32Width = index_chn * 1920 / frame_transition_;
    // crop_info.u32Height = index_chn * 1080 / frame_transition_;
    // HB_VOT_SetChnCrop(0, 0, &crop_info);

    // 左上->右下
    // VOT_CROP_INFO_S crop_info = {0};
    // crop_info.u32Width = index_chn * 1920 / frame_transition_;
    // crop_info.u32Height = index_chn * 1080 / frame_transition_;
    // HB_VOT_SetChnCrop(0, 0, &crop_info);

    // 右上->左下
    // POINT_S display_point = {0};
    // display_point.s32X = 1920 - index_chn * 1920 / frame_transition_;
    // display_point.s32Y = 0;
    // ret = HB_VOT_SetChnDisplayPosition(0, 0, &display_point);

    // VOT_CROP_INFO_S crop_info = {0};
    // crop_info.u32Width = index_chn * 1920 / frame_transition_;
    // crop_info.u32Height = index_chn * 1080 / frame_transition_;
    // HB_VOT_SetChnCrop(0, 0, &crop_info);
  } else {
    if (video_data->channel == chn_display) {
      memcpy(buffer_, video_data->y_virtual_addr, 1920 * 1080);
      memcpy(buffer_ + 1920 * 1080, video_data->uv_virtual_addr,
             1920 * 1080 / 2);

      VOT_FRAME_INFO_S stFrame = {0};
      stFrame.addr = buffer_;
      stFrame.size = 1920 * 1080 * 3 / 2;
      HB_VOT_SendFrame(0, 1, &stFrame, -1);
      if (!display_chn_hide_) {
        HB_VOT_HideChn(0, 0);
        display_chn_hide_ = true;
      }
    }
  }
  return 0;
}

}  // namespace video_box
}  // namespace solution
