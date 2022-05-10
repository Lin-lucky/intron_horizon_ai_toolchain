/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_RTSPMESSAGE_RTSPMESSAGE_H_
#define INCLUDE_RTSPMESSAGE_RTSPMESSAGE_H_

#include <memory>
#include <vector>

#include "blocking_queue.hpp"
#include "media_pipe_manager/media_pipeline.h"
#include "xproto/msg_type/vio_message.h"

namespace solution {
namespace video_box {
using xstream::PyramidImageFrame;
using xproto::message::VioMessage;

struct ImageVioMessage : VioMessage {
public:
  ImageVioMessage() = delete;
  explicit ImageVioMessage(
      std::vector<std::shared_ptr<PyramidImageFrame>> &image_frame,
      uint32_t img_num, bool is_valid = true, int channel = -1,
      std::shared_ptr<MediaPipeline> pipeline = nullptr, void *data = nullptr);
  ~ImageVioMessage();

  // serialize proto
  std::string Serialize() { return "No need serialize"; };

  void FreeImage();
  void FreeImage(int tmp); // 用于释放x3临时回灌功能的接口

  std::shared_ptr<MediaPipeline> pipeline_;
  void *slot_data_;
};

struct DropVioMessage : VioMessage {
public:
  DropVioMessage() = delete;
  explicit DropVioMessage(uint64_t timestamp, uint64_t seq_id);
  ~DropVioMessage(){};

  // serialize proto
  std::string Serialize() override;
};

}  // namespace video_box
}  // namespace solution

#endif
