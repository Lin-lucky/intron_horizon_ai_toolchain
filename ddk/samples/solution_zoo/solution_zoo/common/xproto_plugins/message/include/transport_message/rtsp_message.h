/*
 * @Description: implement of  vio data header
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-10-14 16:35:21
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:11:58
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef XPROTO_MESSAGE_TRANSPORT_RTSP_MESSAGE_H_
#define XPROTO_MESSAGE_TRANSPORT_RTSP_MESSAGE_H_

#include <memory>
#include <vector>
#include <string>
#include "xstream/vision_type.h"
#include "xproto/message/flowmsg.h"

namespace xproto {
namespace message {

#define TYPE_DECODE_IMAGE_MESSAGE "XPLUGIN_DECODE_IMAGE_MESSAGE"
#define TYPE_DECODE_DROP_MESSAGE "XPLUGIN_DECODE_DROP_MESSAGE"

struct RtspMessage : public XProtoMessage {
 public:
  RtspMessage() { type_ = TYPE_DECODE_IMAGE_MESSAGE; }
  virtual ~RtspMessage() = default;

  // channel number
  uint32_t chanel = 0;
  // image frames number
  uint32_t num_ = 0;
  // sequence id, would increment automatically
  uint64_t sequence_id_ = 0;
  // time stamp
  uint64_t time_stamp_ = 0;
  // is valid uri
  bool is_valid_uri_ = true;
  // free source image
  void FreeImage();
  // serialize proto
  std::string Serialize() override { return "Default vio message"; };
  std::vector<std::shared_ptr<xstream::PyramidImageFrame>> image_;
};

}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MESSAGE_TRANSPORT_RTSP_MESSAGE_H_
