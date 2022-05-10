/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail:
 * @Date:
 * @Version:
 * @Brief:
 * @Last Modified by:
 * @Last Modified time:
 */

#include "smart_plugin/utils/method_const.h"

namespace xproto {

const char* kMethodWork = "work";
const char* kMethodIdle = "idle";

/// 子模块名称: 打分模块
const char* kMethodGrading = "grading";
/// 子模块名称: 抓拍模块
const char* kMethodSnapShot = "snapshot";
/// 子模块名称: 人脸跟踪模块
const char* kMethodFaceMot = "face_mot";
/// 子模块名称: 过滤模块
const char* kMethodFilter = "filter";
/// 子模块名称: 人脸识别模块
const char* kMethodFaceFeature = "face_feature";
/// 子模块名称: 图片质量检测模块
const char* kMethodQuality = "face_quality";

/// 配置字段：设置过滤模块图像左右两边水平边界阈值
const char* kMethodBound_thr_w = "bound_thr_w";
/// 配置字段：设置过滤模块图像左右两边垂直边界阈值
const char* kMethodBound_thr_h = "bound_thr_h";
/// 配置字段：设置过滤模块人脸框大小阈值
const char* kMethodMinRectSize = "face_size_thr";
/// 配置字段：设置过滤模块人脸正侧椭球pitch阈值
const char* kMethodFrontalPitchThr = "frontal_pitch_thr";
/// 配置字段：设置过滤模块人脸正侧椭球yaw阈值
const char* kMethodFrontalYawThr = "frontal_yaw_thr";
/// 配置字段：设置过滤模块人脸正侧椭球roll阈值
const char* kMethodFrontalRollThr = "frontal_roll_thr";
/// 配置字段：设置过滤模块人脸置信度阈值
const char* kMethodPvThr = "face_pv_thr";
/// 配置字段：设置过滤模块人脸清晰度阈值
const char* kMethodQualityThr = "quality_thr";
/// 配置字段：设置过滤模块人脸关键点阈值
const char* kMethodLmkThr = "lmk_thr";
/// 配置字段：设置过滤模块黑名单区域iou阈值
const char* kMethodBlackAreaIouThr = "black_area_iou_thr";
/// 配置字段：设置过滤模块黑名单区域
const char* kMethodBlackAreaList = "black_area_list";
/// 配置字段：设置过滤模块人脸左眼遮挡阈值
const char* kMethodLeftEyeOccludedThr = "left_eye_occluded_thr";
/// 配置字段：设置过滤模块人脸右眼遮挡阈值
const char* kMethodRightEyeOccludedThr = "right_eye_occluded_thr";
/// 配置字段：设置过滤模块人脸左眉遮挡阈值
const char* kMethodLeftBrowOccludedThr = "left_brow_occluded_thr";
/// 配置字段：设置过滤模块人脸右眉遮挡阈值
const char* kMethodRightBrowOccludedThr = "right_brow_occluded_thr";
/// 配置字段：设置过滤模块人脸前额遮挡阈值
const char* kMethodForeheadOccludedThr = "forehead_occluded_thr";
/// 配置字段：设置过滤模块左脸遮挡阈值
const char* kMethodLeftCheekOccludedThr = "left_cheek_occluded_thr";
/// 配置字段：设置过滤模块右脸遮挡阈值
const char* kMethodRightCheekOccludedThr = "right_cheek_occluded_thr";
/// 配置字段：设置过滤模块鼻子遮挡阈值
const char* kMethodNoseOccludedThr = "nose_occluded_thr";
/// 配置字段：设置过滤模块嘴巴遮挡阈值
const char* kMethodMouthOccludedThr = "mouth_occluded_thr";
/// 配置字段：设置过滤模块下巴遮挡阈值
const char* kMethodJawOccludedThr = "jaw_occluded_thr";
/// 配置字段：设置过滤模块行为异常阈值
const char* kMethodAbnormalThr = "abnormal_thr";
/// 配置字段：设置过滤模块外扩系数
const char* kMethodExpandScale = "face_expand_scale";
/// 配置字段：设置亮度过滤最小值
const char* kMethodBrightnessMin = "brightness_min";
/// 配置字段：设置亮度过滤最小值
const char* kMethodBrightnessMax = "brightness_max";



/*****************************抓拍相关****************************/
/// 配置字段：设置抓拍模块消失策略
const char* kMethodVanishPost = "vanish_post";
const char* kMethodReportFlushTrackFlag = "report_flushed_track_flag";
const char* kMethodOutDateTargetPostFlag = "out_date_target_post_flag";
const char* kMethodRepeatPostFlag = "repeat_post_flag";
/// 配置字段：Debug功能开关，默认为kMethodOff
const char* kMethodDebug = " debug_mode";
/// 配置字段：设置抓拍图更新所需优选分差
const char* kMethodSnapshotUpdateStep = "update_steps";
/// 配置字段：设置每个目标最多存储抓拍图数量
const char* kMethodSnapshotPerTrack = "snaps_per_track";
/// 配置字段：设置抓拍模块跟踪的最大目标数目
const char* kMethodSnapshotMaxTracks = "max_tracks";
/// 配置字段：设置每一帧进入抓拍状态机最多抓拍数量
/// 该数量与该帧最多抓拍数量没有关系
const char* kMethodMaxCropNumPerFrame = "max_crop_num_per_frame";
/// 配置字段：设置每一帧进入抓拍状态机平均抓拍数量
const char* kMethodAvgCropNumPerFrame = "avg_crop_num_per_frame";
/// 配置字段：设置抓拍模块kMethodAvgCropNumPerFrame应用帧范围
const char* kMethodSnapshotSmoothingFrameRange = "smoothing_frame_range";
/// 配置字段：设置消失帧数量
const char* kMethodVanishFrameCount = "vanish_frame_count";
/// 配置字段：设置抓拍模块输出抓拍图宽度
const char* kMethodSnapshotOutputWidth = "output_width";
/// 配置字段：设置抓拍模块输出抓拍图高度
const char* kMethodSnapshotOutputHeight = "output_height";
/// 配置字段：设置抓拍图外扩系数
const char* kMethodScaleRate = "scale_rate";
/// 配置字段：设置每个目标触发抓拍所需的帧数阈值
const char* kMethodBeginPostFrameThr = "begin_post_frame_thr";
/// 配置字段：设置重抓拍阈值，该阈值大于kMethodBeginPostFrameThr时才生效
const char* kMethodResnapValue = "resnap_value";

}  // namespace xproto
