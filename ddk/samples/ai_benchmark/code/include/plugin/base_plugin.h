// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _PLUGIN_BASE_PLUGIN_H_
#define _PLUGIN_BASE_PLUGIN_H_

#define TYPE_INPUT_MESSAGE "XPLUGIN_INPUT_MESSAGE"
#define TYPE_RELEASE_MESSAGE "XPLUGIN_RELEASE_MESSAGE"
#define TYPE_OUTPUT_MESSAGE "XPLUGIN_OUTPUT_MESSAGE"

#include <memory>
#include <string>

#include "base/perception_common.h"
#include "input/input_data.h"
#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"
#include "xstream/xstream_data.h"
#include "xstream/xstream_sdk.h"

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;

typedef std::shared_ptr<ImageTensor> ImageTensorPtr;
typedef std::shared_ptr<Perception> PerceptionPtr;

struct InputMessage : XProtoMessage {
  explicit InputMessage(ImageTensorPtr &image_tensor)
      : image_tensor(image_tensor) {
    type_ = TYPE_INPUT_MESSAGE;
  }

  std::string Serialize() override { return type_; }

  ImageTensorPtr image_tensor;
};

struct OutputMessage : XProtoMessage {
  explicit OutputMessage(ImageTensorPtr &image_tensor,
                         PerceptionPtr &perception)
      : image_tensor(image_tensor), perception(perception) {
    type_ = TYPE_OUTPUT_MESSAGE;
  }

  std::string Serialize() override { return type_; }

  ImageTensorPtr image_tensor;
  PerceptionPtr perception;
};

struct ReleaseMessage : XProtoMessage {
  explicit ReleaseMessage(ImageTensorPtr &image_tensor)
      : image_tensor(image_tensor) {
    type_ = TYPE_RELEASE_MESSAGE;
  }

  std::string Serialize() override { return type_; }

  ImageTensorPtr image_tensor;
};

class BasePlugin : public XPluginAsync {
 public:
  BasePlugin() = default;
  virtual int Init(std::string config_file, std::string config_string);
  virtual ~BasePlugin() {}

 protected:
  virtual int LoadConfig(std::string &config_string) { return 0; }

 private:
  int LoadConfigFile(std::string &config_file);
};

#endif  // _PLUGIN_BASE_PLUGIN_H_
