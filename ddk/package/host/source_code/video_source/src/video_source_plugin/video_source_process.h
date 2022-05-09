/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_PROCESS_H_
#define VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_PROCESS_H_

#include <memory>
#include <vector>
#include "xstream/vision_type.h"
#include "video_source/video_source_type.h"

namespace xproto {

typedef struct MessageContext_ {
  void *pym_context;
  void *src_context;
} MessageContext;

inline void ConvertPym2Msg(
    std::shared_ptr<videosource::PyramidFrame> &input,
    std::shared_ptr<xstream::PyramidImageFrame> &output) {
    auto pym_msg_img = std::make_shared<xstream::PyramidImageFrame>();
    pym_msg_img->channel_id_ = input->channel_id_;
    if (input->time_stamp_) {
      pym_msg_img->time_stamp_ = input->time_stamp_ / 24000;  // unit is ms
      // pym_msg_img->time_stamp_ = input->system_time_stamp_;  // unit is ms
    } else {
      pym_msg_img->time_stamp_ = input->system_time_stamp_;  // unit is ms
    }
    pym_msg_img->frame_id_ = input->frame_id_;
    auto *msg_ctx = reinterpret_cast<MessageContext*>(
        std::calloc(1, sizeof(MessageContext)));
    HOBOT_CHECK(msg_ctx);
    msg_ctx->pym_context = input->pym_context_;
    msg_ctx->src_context = input->src_context_;
    pym_msg_img->context_ = reinterpret_cast<void*>(msg_ctx);
    int ds_layer_num = static_cast<int>(input->bl_ds_.size());
    int us_layer_num = static_cast<int>(input->roi_us_.size());
    pym_msg_img->img_.ds_pym_layer = ds_layer_num;
    pym_msg_img->img_.us_pym_layer = us_layer_num;
    pym_msg_img->img_.frame_id = pym_msg_img->frame_id_;
    pym_msg_img->img_.timestamp = pym_msg_img->time_stamp_;
    pym_msg_img->img_.cam_id = pym_msg_img->channel_id_;
    {
      pym_msg_img->img_.src_img.width = input->src_info_.width;
      pym_msg_img->img_.src_img.height = input->src_info_.height;
      pym_msg_img->img_.src_img.step = input->src_info_.stride;
      pym_msg_img->img_.src_img.y_paddr = input->src_info_.y_paddr;
      pym_msg_img->img_.src_img.c_paddr = input->src_info_.c_paddr;
      pym_msg_img->img_.src_img.y_vaddr = input->src_info_.y_vaddr;
      pym_msg_img->img_.src_img.c_vaddr = input->src_info_.c_vaddr;
    }
    for (int i = 0; i < ds_layer_num; ++i) {
      pym_msg_img->img_.down_scale[i].width =
        input->bl_ds_[i].width;
      pym_msg_img->img_.down_scale[i].height =
        input->bl_ds_[i].height;
      pym_msg_img->img_.down_scale[i].step =
        input->bl_ds_[i].stride;
      pym_msg_img->img_.down_scale[i].y_paddr =
        input->bl_ds_[i].y_paddr;
      pym_msg_img->img_.down_scale[i].c_paddr =
        input->bl_ds_[i].c_paddr;
      pym_msg_img->img_.down_scale[i].y_vaddr =
        input->bl_ds_[i].y_vaddr;
      pym_msg_img->img_.down_scale[i].c_vaddr =
        input->bl_ds_[i].c_vaddr;
    }
    for (int i = 0; i < us_layer_num; ++i) {
      pym_msg_img->img_.up_scale[i].width = input->roi_us_[i].width;
      pym_msg_img->img_.up_scale[i].height = input->roi_us_[i].height;
      pym_msg_img->img_.up_scale[i].step = input->roi_us_[i].stride;
      pym_msg_img->img_.up_scale[i].y_paddr = input->roi_us_[i].y_paddr;
      pym_msg_img->img_.up_scale[i].c_paddr = input->roi_us_[i].c_paddr;
      pym_msg_img->img_.up_scale[i].y_vaddr = input->roi_us_[i].y_vaddr;
      pym_msg_img->img_.up_scale[i].c_vaddr = input->roi_us_[i].c_vaddr;
    }
    int ds_main_layer_num = ds_layer_num / 4;
    for (int i = 0, j = 0; i < ds_main_layer_num; i++) {
      j = i * 4;
      pym_msg_img->img_.down_scale_main[i].width =
        input->bl_ds_[j].width;
      pym_msg_img->img_.down_scale_main[i].height =
        input->bl_ds_[j].height;
      pym_msg_img->img_.down_scale_main[i].step =
        input->bl_ds_[j].stride;
      pym_msg_img->img_.down_scale_main[i].y_paddr =
        input->bl_ds_[j].y_paddr;
      pym_msg_img->img_.down_scale_main[i].c_paddr =
        input->bl_ds_[j].c_paddr;
      pym_msg_img->img_.down_scale_main[i].y_vaddr =
        input->bl_ds_[j].y_vaddr;
      pym_msg_img->img_.down_scale_main[i].c_vaddr =
        input->bl_ds_[j].c_vaddr;
    }
    output = pym_msg_img;
}

inline void ConvertMsg2Pym(
    std::shared_ptr<xstream::PyramidImageFrame> &input,
    std::shared_ptr<videosource::PyramidFrame> &output) {
    auto pym_img = std::make_shared<videosource::PyramidFrame>();
    pym_img->channel_id_ = input->channel_id_;
    pym_img->time_stamp_ = input->time_stamp_;
    pym_img->frame_id_ = input->frame_id_;
    auto msg_ctx = reinterpret_cast<MessageContext*>(input->context_);
    HOBOT_CHECK(msg_ctx);
    pym_img->pym_context_ = msg_ctx->pym_context;
    pym_img->src_context_ = msg_ctx->src_context;
    if (msg_ctx) std::free(msg_ctx);

    int ds_layer_num = input->img_.ds_pym_layer;
    int us_layer_num = input->img_.us_pym_layer;
    for (int i = 0; i < ds_layer_num; ++i) {
      videosource::ImageLevelInfo ds_info = { 0 };
      ds_info.width = input->img_.down_scale[i].width;
      ds_info.height = input->img_.down_scale[i].height;
      ds_info.stride = input->img_.down_scale[i].step;
      ds_info.y_paddr = input->img_.down_scale[i].y_paddr;
      ds_info.c_paddr = input->img_.down_scale[i].c_paddr;
      ds_info.y_vaddr = input->img_.down_scale[i].y_vaddr;
      ds_info.c_vaddr = input->img_.down_scale[i].c_vaddr;
      pym_img->bl_ds_.push_back(ds_info);
    }
    for (int i = 0; i < us_layer_num; ++i) {
      videosource::ImageLevelInfo us_info = { 0 };
      us_info.width = input->img_.up_scale[i].width;
      us_info.height = input->img_.up_scale[i].height;
      us_info.stride = input->img_.up_scale[i].step;
      us_info.y_paddr = input->img_.up_scale[i].y_paddr;
      us_info.c_paddr = input->img_.up_scale[i].c_paddr;
      us_info.y_vaddr = input->img_.up_scale[i].y_vaddr;
      us_info.c_vaddr = input->img_.up_scale[i].c_vaddr;
      pym_img->roi_us_.push_back(us_info);
    }
    output = pym_img;
}

}  // namespace xproto
#endif  // VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_PROCESS_H_
