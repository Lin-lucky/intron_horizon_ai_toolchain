/*
 * @Description: implement of  vio data header
 * @Author: fei.cheng@horizon.ai
 * @Date: 2019-10-14 16:35:21
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:11:58
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#ifndef XPROTO_MESSAGE_IMAGE_VIO_MESSAGE_H_
#define XPROTO_MESSAGE_IMAGE_VIO_MESSAGE_H_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xproto/version.h"
#include "hobotlog/hobotlog.hpp"
#include "xproto/message/flowmsg.h"
#include "xstream/vision_type.h"

namespace xproto {
namespace message {

#define TYPE_IMAGE_MESSAGE "XPLUGIN_IMAGE_MESSAGE"
#define TYPE_DROP_MESSAGE "XPLUGIN_DROP_MESSAGE"
#define TYPE_DROP_IMAGE_MESSAGE "XPLUGIN_DROP_IMAGE_MESSAGE"
#define TYPE_MULTI_IMAGE_MESSAGE "XPLUGIN_MULTI_IMAGE_MESSAGE"
#define TYPE_FACE_PIC_IMAGE_MESSAGE "XPLUGIN_FACE_PIC_IMAGE_MESSAGE"
#define TYPE_INFO_IMAGE_MESSAGE "XPLUGIN_INFO_IMAGE_MESSAGE"

#define IMAGE_CHANNEL_FROM_AP (1003)  //  meaning this channel image is from ap

struct XPROTO_EXPORT VioMessage : public xproto::XProtoMessage {
 public:
  VioMessage() { type_ = TYPE_IMAGE_MESSAGE; }
  virtual ~VioMessage() = default;

  int channel_ = -1;
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
  std::string Serialize() override { return "Default vio message"; }
  bool DeSerialize(const std::string &data) override { return false; }
  // pyramid image data
  std::vector<std::shared_ptr<xstream::PyramidImageFrame>> image_;
  // src image data
  std::vector<std::shared_ptr<xstream::OriginalPyramidImageFrame>> src_image_;
};

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

}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MESSAGE_IMAGE_VIO_MESSAGE_H_
