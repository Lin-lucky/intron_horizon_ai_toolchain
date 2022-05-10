/*!
 * Copyright (c) 2016-present, Horizon Robotics, Inc.
 * All rights reserved.
 * \File     audio_message.h
 * \Author   ronghui.zhang
 * \Version  1.0.0.0
 * \Date     2021.1.7
 */
#ifndef XPROTO_MSGTYPE_AUDIO_MESSAGE_H_
#define XPROTO_MSGTYPE_AUDIO_MESSAGE_H_

#include <string>
#include "xstream/vision_type.h"
#include "xproto/message/flowmsg.h"

namespace xproto {
namespace message {

#define TYPE_AUDIO_MESSAGE "XPLUGIN_AUDIO_MESSAGE"

struct AudioMessage : public XProtoMessage {
 public:
  AudioMessage() { type_ = TYPE_AUDIO_MESSAGE; }
  explicit AudioMessage(char* buffer, int size, int num) {
    type_ = TYPE_AUDIO_MESSAGE;
    buffer_ = buffer;
    size_ = size;
    num_ = num;
  }
  virtual ~AudioMessage() = default;

  // audio frames number
  uint32_t num_ = 0;
  // sequence id, would increment automatically
  uint64_t sequence_id_ = 0;
  // time stamp
  uint64_t time_stamp_ = 0;
  // is valid uri
  bool is_valid_uri_ = true;

  // serialize proto
  std::string Serialize() override { return "Default audio message"; };
  // audio data
 public:
  // free source image
  void FreeAudio() {
    if (buffer_) {
      free(buffer_);
    }
  }
  char* buffer_;
  int size_;
};

}  // namespace message
}  // namespace xproto

#endif  // XPROTO_MSGTYPE_AUDIO_MESSAGE_H_
