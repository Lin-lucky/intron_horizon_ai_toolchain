/**
* Copyright (c) 2019 Horizon Robotics. All rights reserved.
* @file smart_vision_type.h
* \~English @brief this c header file defines the vision related smart and snapshot message that are used in
 * IOT, including face, head
* @date 2019/4/3
*/

#ifndef SMART_VISION_TYPE_H_
#define SMART_VISION_TYPE_H_

namespace xproto {

typedef uint32_t HorizonVisionTrackID;

typedef struct HorizonVisionBBox_ {
  /// \~Chinese 左上点x坐标
  float x1;
  /// \~Chinese 左上点y坐标
  float y1;
  /// \~Chinese 右下点x坐标
  float x2;
  /// \~Chinese 右下点y坐标
  float y2;
  /// \~Chinese 置信度
  float score;
  /// \~Chinese ID 号
  HorizonVisionTrackID id;
} HorizonVisionBBox;

typedef struct HorizonVisionPose3D_ {
  /// \~Chinese 俯仰角度
  float pitch;
  /// \~Chinese 左右摇头角度
  float yaw;
  /// \~Chinese 侧头角度
  float roll;
  /// \~Chinese 置信度
  float score;
} HorizonVisionPose3D;

typedef struct HorizonVisionPoint_ {
  /// \~Chinese x坐标
  float x;
  /// \~Chinese y坐标
  float y;
  /// \~Chinese 置信度
  float score;
} HorizonVisionPoint;

typedef struct HorizonVisionPoints_ {
  /// \~Chinese 点数目
  size_t num;
  /// \~Chinese 指向2D 坐标点集合的指针
  HorizonVisionPoint *points;
  /// \~Chinese 置信度
  float score;
} HorizonVisionPoints;

typedef struct HorizonVisionAge_ {
  /// \~Chinese 年龄分类
  int32_t value;
  /// \~Chinese 年龄段下限
  int32_t min;
  /// \~Chinese 年龄段上限
  int32_t max;
  /// \~Chinese 置信度
  float score;
} HorizonVisionAge;

/// \~Chinese 关键点
typedef HorizonVisionPoints HorizonVisionLandmarks;

typedef struct HorizonVisionAttribute_ {
  /// \~Chinese 值
  int32_t value;
  /// \~Chinese 置信度
  float score;
} HorizonVisionAttribute;

/// \~Chinese 年龄
typedef HorizonVisionAttribute HorizonVisionGender;
/// \~Chinese 眼镜
typedef HorizonVisionAttribute HorizonVisionGlass;
/// \~Chinese 口罩
typedef HorizonVisionAttribute HorizonVisionBreathingMask;
/// \~Chinese 图片质量
typedef HorizonVisionAttribute HorizonVisionQuality;
/// \~Chinese 活体
typedef HorizonVisionAttribute HorizonVisionAntiSpoofing;

typedef struct HorizonVisionFaceQuality_ {
  /// \~Chinese 人脸清晰度
  HorizonVisionQuality blur;
  /// \~Chinese 人脸亮度
  HorizonVisionQuality brightness;
  /// \~Chinese 眼睛表情
  HorizonVisionQuality eye_abnormalities;
  /// \~Chinese 嘴部
  HorizonVisionQuality mouth_abnormal;
  /// \~Chinese 左眼
  HorizonVisionQuality left_eye;
  /// \~Chinese 右眼
  HorizonVisionQuality right_eye;
  /// \~Chinese 左眉毛
  HorizonVisionQuality left_brow;
  /// \~Chinese 右眉毛
  HorizonVisionQuality right_brow;
  /// \~Chinese 额头
  HorizonVisionQuality forehead;
  /// \~Chinese 左脸颊
  HorizonVisionQuality left_cheek;
  /// \~Chinese 右脸颊
  HorizonVisionQuality right_cheek;
  /// \~Chinese 鼻子
  HorizonVisionQuality nose;
  /// \~Chinese 嘴部
  HorizonVisionQuality mouth;
  /// \~Chinese 下巴
  HorizonVisionQuality jaw;
} HorizonVisionFaceQuality;

typedef struct HorizonVisionFloatArray_ {
  /// \~Chinese 数目
  size_t num;
  /// \~Chinese 指向单精度浮点数组数组的指针
  float *values;
} HorizonVisionFloatArray;
/// \~Chinese 人脸特征
typedef HorizonVisionFloatArray HorizonVisionFeature;

typedef struct HorizonVisionCharArray_ {
  /// \~Chinese 数目
  size_t num;
  /// \~Chinese 指向字符数组的指针
  char *values;
} HorizonVisionCharArray;
typedef HorizonVisionCharArray HorizonVisionEncryptedFeature;

typedef struct HorizonVisionFaceSmartData_ {
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 人脸框
  HorizonVisionBBox face_rect;
  /// \~Chinese 人头框
  HorizonVisionBBox head_rect;
  /// \~Chinese 人脸姿态
  HorizonVisionPose3D pose3d;
  /// \~Chinese 人脸关键点
  HorizonVisionLandmarks *landmarks;
  /// \~Chinese 年龄
  HorizonVisionAge age;
  /// \~Chinese 性别
  HorizonVisionGender gender;
  /// \~Chinese 眼镜
  HorizonVisionGlass glass;
  /// \~Chinese 口罩
  HorizonVisionBreathingMask mask;
  /// \~Chinese 活体信息
  HorizonVisionAntiSpoofing anti_spoofing;
  /// \~Chinese 人脸质量
  HorizonVisionFaceQuality quality;
  /// \~Chinese 人脸特征
  HorizonVisionFeature *feature;
  /// \~Chinese 加密后的人脸特征
  HorizonVisionEncryptedFeature *encrypted_feature;
} HorizonVisionFaceSmartData;

typedef struct HorizonVisionSegmentation_ {
  /// \~Chinese 数目
  size_t num;
  /// \~Chinese 指向float数组的指针
  float *values;
  /// \~Chinese 区域宽度
  int32_t width;
  /// \~Chinese 区域高度
  int32_t height;
} HorizonVisionSegmentation;

typedef struct HorizonVisionBodySmartData_ {
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 人体框
  HorizonVisionBBox body_rect;
  /// \~Chinese 人体分割
  HorizonVisionSegmentation *segmentation;
  /// \~Chinese 骨骼点
  HorizonVisionLandmarks *skeleton;
} HorizonVisionBodySmartData;

typedef struct HorizonVisionHandSmartData_ {
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 手势框
  HorizonVisionBBox hand_rect;
  /// \~Chinese 手势类型
  uint32_t hand_gesture;
} HorizonVisionHandSmartData;

typedef struct {
  /// \~Chinese RGB活体信息
  HorizonVisionAntiSpoofing rgb_anti_spf_;
  /// \~Chinese 红外活体信息
  HorizonVisionAntiSpoofing nir_anti_spf_;
  /// \~Chinese 红外检测框
  HorizonVisionBBox nir_box_;
  /// \~Chinese 红外外扩检测框
  HorizonVisionBBox nir_norm_box_;
  /// \~Chinese RGB检测框
  HorizonVisionBBox rgb_box_;
  /// \~Chinese RGB外扩检测框
  HorizonVisionBBox rgb_norm_box_;
} HorizonVisionFaceExtraInfo;

typedef struct HorizonVisionSmartData_ {
  /// \~Chinese 目标类型 0：正常，1:已过滤 2:已消失
  uint32_t type;
  /// \~Chinese 跟踪ID
  HorizonVisionTrackID track_id;
  /// \~Chinese 人脸智能信息
  HorizonVisionFaceSmartData *face;
  /// \~Chinese 人体智能信息
  HorizonVisionBodySmartData *body;
  /// \~Chinese 手势智能信息
  HorizonVisionHandSmartData *hand;
  /// \~Chinese 人脸额外的辅助信息，默认为空
  HorizonVisionFaceExtraInfo *face_extra;
} HorizonVisionSmartData;

}   //  namespace xproto

#endif  // SMART_VISION_TYPE_H_
