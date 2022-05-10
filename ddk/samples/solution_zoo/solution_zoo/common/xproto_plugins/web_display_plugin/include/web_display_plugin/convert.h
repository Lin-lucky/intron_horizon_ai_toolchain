/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-02 03:42:16
 * @Version: v0.0.1
 * @Brief:  convert to xroc inputdata from input VioMessage
 * @Note:  extracted from xperson repo.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-08-02 05:22:15
 */

#ifndef XPROTO_WEB_DISPLAY_PLUGIN_INCLUDE_CONVERT_H_
#define XPROTO_WEB_DISPLAY_PLUGIN_INCLUDE_CONVERT_H_

#include <string>
#include <vector>

#include "media_codec/media_codec_manager.h"
#include "opencv2/opencv.hpp"
#include "smart_message/smart_message.h"
#include "web_display_plugin/convert.h"
#include "xproto/msg_type/smart_legible_message.h"
#include "xproto/msg_type/vio_message.h"
#include "xproto/xproto_world.h"

namespace xproto {

using xproto::XProtoMessagePtr;
using xproto::message::SmartLegibleMessage;
using xproto::message::SmartLegibleMessagePtr;
using xproto::message::SmartMessage;
using xproto::message::VioMessage;

class Convertor {
 public:
  /**
   * @brief get yuv data from VioMessage (only for mono)
   * @param out yuv_img - cv::Mat with yuv type
   * @param in vio_msg
   * @param in layer - number
   * @return error code
   */
  static int GetYUV(cv::Mat &yuv_img, VioMessage *vio_msg, int level);

  /**
   * @brief get yuv data from VioMessage (only for mono)
   * @param out frame_buf - VideoEncodeSourceBuffer with yuv type
   * @param in vio_msg
   * @param in layer - number
   * @param in use_vb - if use vb memory or not
   * @return error code
   */
  static int GetYUV(VideoEncodeSourceBuffer *frame_buf, VioMessage *vio_msg,
          int level, int use_vb);
  /**
   * @brief convert cv::Mat(format of yuv) to JPG data
   * @param out img_buf
   * @param in yuv_img
   * @param in quality
   * @return bool
   */
  static bool YUV2JPG(std::vector<uchar> &img_buf,
      cv::Mat &yuv_img, int quality);

  /**
   * @brief package smart messge to proto
   * @param out data - protobuf serialized string
   * @param in smart_msg
   * @return error code
   */
  static int PackSmartMsg(std::string &data, XProtoMessagePtr smart_msg);
};

}  // namespace xproto

#endif  //  XPROTO_WEB_DISPLAY_PLUGIN_INCLUDE_CONVERT_H_
