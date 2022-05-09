/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_MESSAGE_H_
#define VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_MESSAGE_H_
#include <memory>
#include <vector>
#include <string>
#include "xproto/msg_type/vio_message.h"
#include "xproto/message/msg_registry.h"
#include "xstream/vision_type.h"
#include "video_source/video_source.h"
#include "video_source/video_source_type.h"

namespace xproto {

using xstream::PyramidImageFrame;
using xproto::message::VioMessage;
using xproto::message::MultiVioMessage;
using videosource::VideoSource;
using videosource::ImageFrame;
using videosource::PyramidFrame;

struct ImageVioMessage : VioMessage {
 public:
  ImageVioMessage() = delete;
  explicit ImageVioMessage(
      const std::shared_ptr<VideoSource> &video_source,
      const std::shared_ptr<PyramidImageFrame> &image_frame,
      uint32_t img_num, bool is_valid = true);

  ~ImageVioMessage();

  // serialize proto
  std::string Serialize() { return "No need serialize"; }

  void FreeImage() {}

 private:
  std::shared_ptr<VideoSource> video_source_;
};

struct DropImageVioMessage : VioMessage {
 public:
  DropImageVioMessage() = delete;
  explicit DropImageVioMessage(
      const std::shared_ptr<VideoSource> &video_source,
      const std::shared_ptr<PyramidImageFrame> &image_frame,
      uint32_t img_num, bool is_valid = true);
  ~DropImageVioMessage();

  // serialize proto
  std::string Serialize() { return "No need serialize"; }

 private:
  std::shared_ptr<VideoSource> video_source_;
};

#if 0
struct MultiVioMessage : VioMessage {
 public:
  MultiVioMessage() {
    LOGI << "MultiVioMessage()";
    type_ = TYPE_MULTI_IMAGE_MESSAGE;}
  std::vector<std::shared_ptr<VioMessage>> multi_vio_img_;
  ~MultiVioMessage() {
    LOGI << "~MultiVioMessage";
    multi_vio_img_.clear();}
};
#endif

}  // namespace xproto

#endif  // VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_MESSAGE_H_
