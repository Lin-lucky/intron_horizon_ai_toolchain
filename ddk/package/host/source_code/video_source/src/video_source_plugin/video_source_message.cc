/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source_plugin/video_source_message.h"
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "video_source_plugin/video_source_produce.h"
#include "video_source_plugin/video_source_process.h"
#include "hobotlog/hobotlog.hpp"

namespace xproto {

ImageVioMessage::ImageVioMessage(
    const std::shared_ptr<VideoSource> &video_source,
    const std::shared_ptr<PyramidImageFrame> &image_frame,
    uint32_t img_num, bool is_valid) {
  type_ = TYPE_IMAGE_MESSAGE;
  num_ = img_num;
  is_valid_uri_ = is_valid;
  video_source_ = video_source;
  if (image_frame) {
    time_stamp_ = image_frame->time_stamp_;
    sequence_id_ = image_frame->frame_id_;
    channel_ = image_frame->channel_id_;
    image_.resize(img_num);
    image_[0] = image_frame;
  } else {
    LOGE << "image frame is nullptr";
  }
}

ImageVioMessage::~ImageVioMessage() {
  LOGI << "call ~ImageVioMessage";
  int ret = -1;
  std::shared_ptr<videosource::PyramidFrame> pym_image;
  ConvertMsg2Pym(image_[0], pym_image);
  ret = video_source_->FreePyramidFrame(pym_image);
  if (ret) {
    std::cout << "free pyramid frame failed, ret: " << ret << std::endl;
  }
}

DropImageVioMessage::DropImageVioMessage(
    const std::shared_ptr<VideoSource> &video_source,
    const std::shared_ptr<PyramidImageFrame> &image_frame,
    uint32_t img_num, bool is_valid) {
  type_ = TYPE_DROP_MESSAGE;
  num_ = img_num;
  is_valid_uri_ = is_valid;
  video_source_ = video_source;
  if (image_frame) {
    time_stamp_ = image_frame->time_stamp_;
    sequence_id_ = image_frame->frame_id_;
    channel_ = image_frame->channel_id_;
    image_.resize(img_num);
    image_[0] = image_frame;
  } else {
    LOGE << "image frame is nullptr";
  }
  HOBOT_CHECK(image_frame);
}

DropImageVioMessage::~DropImageVioMessage() {
  LOGI << "call ~DropImageVioMessage";
  int ret = -1;
  std::shared_ptr<videosource::PyramidFrame> pym_image;
  ConvertMsg2Pym(image_[0], pym_image);
  ret = video_source_->FreePyramidFrame(pym_image);
  if (ret) {
    std::cout << "free pyramid frame failed, ret: " << ret << std::endl;
  }
}

}  // namespace xproto
