/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 03:42:16
 * @Version: v0.0.1
 * @Brief:  convert to xstream inputdata from input VioMessage
 * @Note:  extracted from xperson repo.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 02:57:24
 */
#ifndef INCLUDE_SMARTPLUGIN_CONVERT_H_
#define INCLUDE_SMARTPLUGIN_CONVERT_H_

#include <string>

#include "smart_plugin/traffic_info.h"
#include "xproto/msg_type/vio_message.h"
#include "xstream/xstream_world.h"

namespace xproto {
using xproto::message::VioMessage;

class Convertor {
 public:
  /**
   * @brief convert input VioMessage to xstream inputdata.
   * @param input input VioMessage
   * @param input image input name in xstream workflow config
   * @return xstream::InputDataPtr xstream input
   */
  static xstream::InputDataPtr ConvertInput(const VioMessage *input,
                                            std::string input_name = "image");

 public:
  static int image_compress_quality;
};

}  // namespace xproto

#endif  //  INCLUDE_SMARTPLUGIN_CONVERT_H_
