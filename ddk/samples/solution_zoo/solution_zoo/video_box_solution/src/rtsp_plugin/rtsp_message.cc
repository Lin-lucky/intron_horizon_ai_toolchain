/*
 * @Description: implement of vioplugin
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-08-26 16:17:25
 * @Author: songshan.gong@horizon.ai
 * @Date: 2019-09-26 16:17:25
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 15:34:58
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#include "rtsp_plugin/rtsp_message.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "vision/vision_type_util.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xstream/xstream_world.h"

namespace solution {
namespace video_box {

ImageVioMessage::ImageVioMessage(
    std::vector<std::shared_ptr<PyramidImageFrame>> &image_frame,
    uint32_t img_num, bool is_valid, int channel,
    std::shared_ptr<MediaPipeline> pipeline,
    void *data) {
  type_ = TYPE_IMAGE_MESSAGE;
  num_ = img_num;
  channel_ = channel;
  if (image_frame.size() > 0) {
    time_stamp_ = image_frame[0]->time_stamp_;
    sequence_id_ = image_frame[0]->frame_id_;
    image_.resize(image_frame.size());
  }
  is_valid_uri_ = is_valid;
  for (uint32_t i = 0; i < image_frame.size(); ++i) {
    image_[i] = image_frame[i];
  }
  pipeline_ = pipeline;
  slot_data_ = data;
}

ImageVioMessage::~ImageVioMessage() { LOGI << "call ~ImageVioMessage"; }

DropVioMessage::DropVioMessage(uint64_t timestamp, uint64_t seq_id) {
  type_ = TYPE_DROP_MESSAGE;
  time_stamp_ = timestamp;
  sequence_id_ = seq_id;
}

std::string DropVioMessage::Serialize() {
  std::string smart_str;
  x3::SmartLegibleMessage smart_frame_message;
  LOGI << "Drop Serialize";
  smart_frame_message.set_timestamp_(time_stamp_);
  smart_frame_message.set_error_code_(1);
  smart_frame_message.SerializeToString(&smart_str);

  return smart_str;
}

}  // namespace rtspplugin
}  // namespace xproto
