/*
 * @Description: implement of monitor_control_message.h
 * @Author: yingmin.li@horizon.ai
 * @Date: 2019-08-24 11:29:24
 * @LastEditors: hao.tian@horizon.ai
 * @LastEditTime: 2019-10-16 16:11:08
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */
#ifndef XPROTO_MESSAGE_TRANSPORT_MONITOR_CONTROL_MESSAGE_H_
#define XPROTO_MESSAGE_TRANSPORT_MONITOR_CONTROL_MESSAGE_H_

#include <string.h>
#include <string>
#include <utility>
#include <memory>
#include "hobotlog/hobotlog.hpp"
#include "xproto/message/flowmsg.h"
#include "xstream/vision_type.h"

namespace xproto {
namespace message {
#define TYPE_APIMAGE_MESSAGE "XPLUGIN_APIMAGE_MESSAGE"
#define TYPE_MC_UPSTREAM_MESSAGE "XPLUGIN_MC_UPSTREAM_MESSAGE"
#define TYPE_MC_DOWNSTREAM_MESSAGE "XPLUGIN_MC_DOWMSTREAM_MESSAGE"

struct APImageMessage : XProtoMessage {
  explicit APImageMessage(const std::string& buff_str,
                          const std::string& img_type, uint32_t width,
                          uint32_t height, uint64_t seq_id)
      : sequence_id_(seq_id) {
    img_ = static_cast<xstream::RawDataImageFrame*>(
        std::calloc(1, sizeof(xstream::RawDataImageFrame)));
    if (img_type == "JPEG") {
      img_->pixel_format_ = xstream::kHorizonVisionPixelFormatImageContainer;
    } else if (img_type == "NV12") {
      img_->pixel_format_ = xstream::kHorizonVisionPixelFormatRawNV12;
    } else if (img_type == "BGR") {
      img_->pixel_format_ = xstream::kHorizonVisionPixelFormatRawBGR;
    } else {
      HOBOT_CHECK(false) << "unsupport AP image type";
    }
    // img_->data_size = buff_str.size();
    img_->width_ = width;
    img_->height_ = height;
    if (img_->DataSize() != buff_str.size()) {
      LOGF << "input img size is wrong!";
    }
    img_->data_ = static_cast<uint8_t*>(std::malloc(buff_str.size()));
    memcpy(img_->data_, buff_str.c_str(), buff_str.size());
    type_ = TYPE_APIMAGE_MESSAGE;
  }

  std::string Serialize() override { return "APImageMessage"; }
  virtual ~APImageMessage() {
    if (img_ && img_->data_) {
      std::free(img_->data_);
    }
    if (img_) {
      std::free(img_);
    }
    img_ = nullptr;
  }

  xstream::RawDataImageFrame* img_ = nullptr;
  uint64_t sequence_id_ = 0;
  std::string img_name;
};

struct APRespMessage : public XProtoMessage {
  explicit APRespMessage(uint64_t seq_id) : sequence_id_(seq_id) {
    type_ = TYPE_MC_UPSTREAM_MESSAGE;
  }
  std::string Serialize() { return proto_; }
  uint64_t sequence_id_ = 0;
  std::string proto_;
};

/* global data/cmd config type */
typedef enum {
  GET_DATA_MESSAGE = 0,
  GET_CP_STATUS,
  GET_CP_CAP,
  GET_CP_PARAM,
  GET_CP_VERSION,
  GET_CP_LOG_LEVEL,
  GET_AP_SYS_TIME,
  SET_DATA_MESSAGE = 40,
  SET_CP_CAP,
  SET_CP_PARAM,
  SET_CP_LOG_LEVEL,
  SET_CP_IMAGE,
  SET_PIC_RECOG,
  SET_CMD_MESSAGE = 80,
  SET_APP_START,
  SET_APP_STOP,
  SET_COMMON_RSP = 120,
} ConfigMessageType;

/* global plugin message mask */
typedef enum {
  MESSAGE_TO_NONE = 0,
  MESSAGE_TO_PRODUCER = 1 << 0,
  MESSAGE_TO_SMARTER = 1 << 1,
  MESSAGE_TO_ADAPTER = 1 << 2,
  MESSAGE_TO_MC = 1 << 3,
  MESSAGE_TO_HBIPC = 1 << 4,
  MESSAGE_TO_VIO = 1 << 5,
  MESSAGE_TO_WARE = 1 << 6,
  MESSAGE_MASK_ALL = 0xFFFFFFFF,
} ConfigMessageMask;

/* global config base data structure */
class BaseConfigData {
 public:
  BaseConfigData() = default;
  explicit BaseConfigData(std::string name, std::string type) {
    name_ = name;
    type_ = type;
    is_json_format_ = false;
  }
  virtual ~BaseConfigData() = default;
  virtual std::string Serialize() = 0;

 public:
  bool is_json_format_;
  std::string name_;
  std::string type_;
};

struct MonitorControlMessage : public XProtoMessage {
 public:
  MonitorControlMessage() = delete;
  explicit MonitorControlMessage(ConfigMessageType type,
                     ConfigMessageMask mask,
                     uint64_t msg_id = 0) {
    type_ = TYPE_MC_DOWNSTREAM_MESSAGE;
    message_mask_ = mask;
    message_type_ = type;
    msg_id_ = msg_id;
  }
  explicit MonitorControlMessage(ConfigMessageType type, ConfigMessageMask mask,
                     std::shared_ptr<BaseConfigData> data,
                     std::string direction, uint64_t msg_id = 0) {
    if (direction == "up") {
      type_ = TYPE_MC_UPSTREAM_MESSAGE;
    } else {
      type_ = TYPE_MC_DOWNSTREAM_MESSAGE;
    }

    message_mask_ = mask;
    message_type_ = type;
    message_data_ = data;
    msg_id_ = msg_id;
  }
  ~MonitorControlMessage() = default;

  bool IsMessageValid(ConfigMessageMask mask);
  ConfigMessageType GetMessageType() { return message_type_; }
  std::shared_ptr<BaseConfigData> GetMessageData() { return message_data_; }
  std::string Serialize() override;

 private:
  ConfigMessageType message_type_;
  ConfigMessageMask message_mask_;
  std::shared_ptr<BaseConfigData> message_data_;
  uint64_t msg_id_;
};

}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MESSAGE_TRANSPORT_MONITOR_CONTROL_MESSAGE_H_
