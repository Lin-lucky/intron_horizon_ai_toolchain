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
#ifndef INCLUDE_ROI_ZOOM_PLUGIN_CONVERT_H_
#define INCLUDE_ROI_ZOOM_PLUGIN_CONVERT_H_

#include <string>
#include <vector>

#include "xstream/xstream_world.h"
#include "utils/roi_vision_msg.h"
namespace xproto {
class Convertor {
 public:
  static HorizonVisionSmartFrame *ConvertOutputToSmartFrame(
      const xstream::OutputDataPtr &xroc_output);

  static void ConvertOutputToSmartFrame(
      const xstream::OutputDataPtr &xroc_output,
      HorizonVisionSmartFrame *smart_frame);

 public:
  static int image_compress_quality;
};

}  // namespace xproto

#endif  //  INCLUDE_ROI_ZOOM_PLUGIN_CONVERT_H_
