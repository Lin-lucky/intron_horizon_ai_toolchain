/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief:
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#ifndef INCLUDE_SMARTPLUGIN_TCLCONVERTOR_H_
#define INCLUDE_SMARTPLUGIN_TCLCONVERTOR_H_

#include <future>
#include <list>
#include <map>
#include <memory>
#include <string>

#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"
#include "smart_message/smart_message.h"

namespace xproto {
using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

class KeyPointConvertor {
 public:
  KeyPointConvertor() = default;
  ~KeyPointConvertor() = default;

 public:
  static void ConverKeyPoint(xstream::OutputDataPtr xstream_out);

 private:
  static void ConvertFaceLmk(xstream::BaseDataVector *face_lmks);
  static void ConvertKps(xstream::BaseDataVector *kps_list,
                             xstream::BaseDataVector *body_box_list,
                             xstream::BaseDataVector *head_box_list,
                             xstream::BaseDataVector *face_lmks,
                             xstream::BaseDataVector *face_box_list);
};

}  // namespace xproto
#endif  // INCLUDE_SMARTPLUGIN_TCLCONVERTOR_H_
