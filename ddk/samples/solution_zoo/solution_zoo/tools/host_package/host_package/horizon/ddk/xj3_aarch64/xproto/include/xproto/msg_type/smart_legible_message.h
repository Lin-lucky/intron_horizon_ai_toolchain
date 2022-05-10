/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xudong.du
 * @Mail: xudong.du@horizon.ai
 * @Date: 2019-09-29 01:50:41
 * @Version: v0.0.1
 * @Brief: smart message declaration.
 * @Last Modified by: xudong.du
 * @Last Modified time: 2021-03-31 02:49:28
 */
#ifndef XPROTO_PLUGINS_MESSAGE_SMART_LEGIBLE_MESSAGE_H_
#define XPROTO_PLUGINS_MESSAGE_SMART_LEGIBLE_MESSAGE_H_

#include <memory>
#include <string>
#include <vector>

#include "xproto/version.h"
#include "xproto/message/flowmsg.h"
#include "xstream/vision_type.h"

namespace xproto {
namespace message {
#define TYPE_SMART_LEGIBLE_MESSAGE "XPLUGIN_SMART_LEGIBLE_MESSAGE"

using xstream::Attribute_;
using xstream::BBox;
using xstream::CharEncryptedFeature;
using xstream::FloatFeature;
using xstream::ImageFrame;
using xstream::Landmarks;
using xstream::Pose3D;
using xstream::Segmentation;

typedef std::shared_ptr<ImageFrame> ImageFramePtr;
typedef std::shared_ptr<BBox> BBoxPtr;
typedef std::shared_ptr<Landmarks> LandmarksPtr;
typedef std::shared_ptr<Attribute_<int32_t>> AttributePtr;
typedef std::shared_ptr<Segmentation> SegmentationPtr;
typedef std::shared_ptr<FloatFeature> FloatFeaturePtr;
typedef std::shared_ptr<CharEncryptedFeature> CharEncryptedFeaturePtr;
typedef std::shared_ptr<Pose3D> Pose3DPtr;

/**
 * Target
 * @type_: 区分人/动物/车类型等
 * @track_id_: 目标id
 * @snap_img_: 抓拍图集合
 * @boxs_: 人体框/人脸框等集合
 * @lmks_: 人体关键点/人脸关键点等集合
 * @attributes_: 年龄/性别/手势识别结果等集合
 * @face_feature_: 非加密人脸特征
 * @face_pose_: 人脸朝向
 * @body_seg_: 人体分割集合
 * @map_seg_: 全图分割结果
 */
struct XPROTO_EXPORT Target {
  std::string type_;
  uint64_t track_id_;
  std::vector<ImageFramePtr> snap_img_;
  std::vector<BBoxPtr> boxs_;
  std::vector<LandmarksPtr> lmks_;
  std::vector<AttributePtr> attributes_;
  FloatFeaturePtr face_feature_;
  Pose3DPtr face_pose_;
  std::vector<SegmentationPtr> body_seg_;
  SegmentationPtr map_seg_;
};
typedef std::shared_ptr<Target> TargetPtr;

/**
 * SmartData
 * @timestamp_: 原视频帧的时间戳
 * @error_code_: 错误码
 * @targets_: 目标集合
 */
struct XPROTO_EXPORT SmartData {
  uint32_t error_code_;
  std::vector<TargetPtr> targets_;
};

const char *const kSmartImageTypeNv12 = "NV12";
const char *const kSmartImageTypeJpg = "jpg";
/**
 * SmartMessage
 * @time_stamp_: 时间戳
 * @sequence_id_: 帧序号
 * @background_img_: 背景图像数据, nv12/jepg等
 * @smart_message_: 智能信息
 */
struct XPROTO_EXPORT SmartLegibleMessage : public xproto::XProtoMessage {
  int channel_id_;
  uint64_t time_stamp_;
  uint64_t frame_id_;
  uint64_t sequence_id_;
  std::string image_name_;
  std::string img_serialize_type_;
  ImageFramePtr background_img_;  //  仅包含原图编码后的jpg图像, 即金字塔0层图像
  ImageFramePtr paramid_img_;  //  金字塔图像数据，仅限进程内使用
  SmartData smart_data_;

  SmartLegibleMessage() {
    type_ = TYPE_SMART_LEGIBLE_MESSAGE;
    background_img_ = nullptr;
    paramid_img_ = nullptr;
    img_serialize_type_ = kSmartImageTypeJpg;  // default jpg
  }
  SmartLegibleMessage(const SmartLegibleMessage &example);
  SmartLegibleMessage &operator=(const SmartLegibleMessage &example);
  virtual ~SmartLegibleMessage() = default;
  std::string Serialize() override;
  bool DeSerialize(const std::string &data) override;

  virtual bool Serialize(std::string &data, int ori_w, int ori_h, int dst_w,
                         int dst_h) {
    return true;
  }
};
using SmartLegibleMessagePtr = std::shared_ptr<SmartLegibleMessage>;
}  // namespace message
}  // namespace xproto

#endif  // XPROTO_PLUGINS_MESSAGE_SMART_LEGIBLE_MESSAGE_H_
