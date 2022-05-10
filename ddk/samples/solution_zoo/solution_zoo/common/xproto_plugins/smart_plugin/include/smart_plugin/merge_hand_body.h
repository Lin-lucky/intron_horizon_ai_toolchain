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

#ifndef INCLUDE_SMARTPLUGIN_MERGEHANDBODY_H_
#define INCLUDE_SMARTPLUGIN_MERGEHANDBODY_H_

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
using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;
using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

class MergeHandBody {
 public:
  static std::shared_ptr<MergeHandBody> &Instance() {
    static std::shared_ptr<MergeHandBody> processor;
    static std::once_flag init_flag;
    std::call_once(init_flag, []() {
      processor = std::shared_ptr<MergeHandBody>(new MergeHandBody());
    });
    return processor;
  }

  ~MergeHandBody() = default;

 public:
  void UpdateHandTrackID(xstream::OutputDataPtr xstream_out);

 private:
  void UpdateHandTrackID(xstream::BaseDataVector *body_list,
                         xstream::BaseDataVector *kps_list,
                         xstream::BaseDataVector *hand_box_list,
                         xstream::BaseDataVector *hand_lmk_list,
                         xstream::BaseDataVector *face_boxe_list);
  bool UpdateHandID(
      xstream::BaseDataVector *body_list, xstream::BaseDataVector *kps_list,
      std::shared_ptr<xstream::Landmarks> hand_lmk,
      int32_t &new_track_id);
  bool UpdateHandID(
      xstream::BaseDataVector *face_boxes_list,
      std::shared_ptr<xstream::Landmarks> hand_lmk,
      int32_t &new_track_id);
  bool UpdateDistance(
      std::shared_ptr<xstream::Landmarks> hand_lmk,
      std::shared_ptr<xstream::Landmarks> kps,
      std::shared_ptr<xstream::BBox> body_box,
      float &min_distance, int32_t &new_track_id);
  bool IsHandOnOneLine(
      std::shared_ptr<xstream::Landmarks> hand_lmk,
      std::shared_ptr<xstream::Landmarks> lmk,
      int32_t body_trackid, const bool left_hand = true);

  void FilterGesture(xstream::BaseDataVector *body_list,
                     xstream::BaseDataVector *kps_list,
                     xstream::BaseDataVector *hand_box_list,
                     xstream::BaseDataVector *hand_lmk_list,
                     xstream::BaseDataVector *gesture_list);

 private:
  MergeHandBody() = default;
};

}  // namespace xproto
#endif  // INCLUDE_SMARTPLUGIN_MERGEHANDBODY_H_
