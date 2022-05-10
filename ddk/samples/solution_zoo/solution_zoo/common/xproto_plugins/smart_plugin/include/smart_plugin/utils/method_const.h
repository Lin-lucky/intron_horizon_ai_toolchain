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

#ifndef _SMARTPLUGIN_SRC_UTILS_METHOD_CONST_H_
#define _SMARTPLUGIN_SRC_UTILS_METHOD_CONST_H_

#include <string>

namespace xproto {

extern const char* kMethodWork;
extern const char* kMethodIdle;
/// 子模块名称: 打分模块
extern const char* kMethodGrading;
/// 子模块名称: 抓拍模块
extern const char* kMethodSnapShot;
/// 子模块名称: 人脸跟踪模块
extern const char* kMethodFaceMot;
/// 子模块名称: 过滤模块
extern const char* kMethodFilter;
/// 子模块名称: 人脸识别模块
extern const char* kMethodFaceFeature;
/// 子模块名称: 图片质量检测模块
extern const char* kMethodQuality;



/// 配置字段：设置过滤模块图像左右两边水平边界阈值
extern const char* kMethodBound_thr_w;
/// 配置字段：设置过滤模块图像左右两边垂直边界阈值
extern const char* kMethodBound_thr_h;
/// 配置字段：设置过滤模块人脸框大小阈值
extern const char* kMethodMinRectSize;
/// 配置字段：设置过滤模块人脸正侧椭球pitch阈值
extern const char* kMethodFrontalPitchThr;
/// 配置字段：设置过滤模块人脸正侧椭球yaw阈值
extern const char* kMethodFrontalYawThr;
/// 配置字段：设置过滤模块人脸正侧椭球roll阈值
extern const char* kMethodFrontalRollThr;
/// 配置字段：设置过滤模块人脸置信度阈值
extern const char* kMethodPvThr;
/// 配置字段：设置过滤模块人脸清晰度阈值
extern const char* kMethodQualityThr;
/// 配置字段：设置过滤模块人脸关键点阈值
extern const char* kMethodLmkThr;
/// 配置字段：设置过滤模块黑名单区域iou阈值
extern const char* kMethodBlackAreaIouThr;
/// 配置字段：设置过滤模块黑名单区域
extern const char* kMethodBlackAreaList;
/// 配置字段：设置过滤模块人脸左眼遮挡阈值
extern const char* kMethodLeftEyeOccludedThr;
/// 配置字段：设置过滤模块人脸右眼遮挡阈值
extern const char* kMethodRightEyeOccludedThr;
/// 配置字段：设置过滤模块人脸左眉遮挡阈值
extern const char* kMethodLeftBrowOccludedThr;
/// 配置字段：设置过滤模块人脸右眉遮挡阈值
extern const char* kMethodRightBrowOccludedThr;
/// 配置字段：设置过滤模块人脸前额遮挡阈值
extern const char* kMethodForeheadOccludedThr;
/// 配置字段：设置过滤模块左脸遮挡阈值
extern const char* kMethodLeftCheekOccludedThr;
/// 配置字段：设置过滤模块右脸遮挡阈值
extern const char* kMethodRightCheekOccludedThr;
/// 配置字段：设置过滤模块鼻子遮挡阈值
extern const char* kMethodNoseOccludedThr;
/// 配置字段：设置过滤模块嘴巴遮挡阈值
extern const char* kMethodMouthOccludedThr;
/// 配置字段：设置过滤模块下巴遮挡阈值
extern const char* kMethodJawOccludedThr;
/// 配置字段：设置过滤模块行为异常阈值
extern const char* kMethodAbnormalThr;
/// 配置字段：设置过滤模块外扩系数
extern const char* kMethodExpandScale;
/// 配置字段：设置亮度过滤最小值
extern const char* kMethodBrightnessMin;
/// 配置字段：设置亮度过滤最小值
extern const char* kMethodBrightnessMax;

/*****************************抓拍相关****************************/
/// 配置字段：设置抓拍模块消失策略
extern const char* kMethodVanishPost;
extern const char* kMethodReportFlushTrackFlag;
extern const char* kMethodOutDateTargetPostFlag;
extern const char* kMethodRepeatPostFlag;
/// 配置字段：Debug功能开关，默认为kMethodOff
extern const char* kMethodDebug;
/// 配置字段：设置抓拍图更新所需优选分差
extern const char* kMethodSnapshotUpdateStep;
/// 配置字段：设置每个目标最多存储抓拍图数量
extern const char* kMethodSnapshotPerTrack;
/// 配置字段：设置抓拍模块跟踪的最大目标数目
extern const char* kMethodSnapshotMaxTracks;
/// 配置字段：设置每一帧进入抓拍状态机最多抓拍数量
/// 该数量与该帧最多抓拍数量没有关系
extern const char* kMethodMaxCropNumPerFrame;
/// 配置字段：设置每一帧进入抓拍状态机平均抓拍数量
extern const char* kMethodAvgCropNumPerFrame;
/// 配置字段：设置抓拍模块kMethodAvgCropNumPerFrame应用帧范围
extern const char* kMethodSnapshotSmoothingFrameRange;
extern const char* kMethodVanishFrameCount;
/// 配置字段：设置抓拍模块输出抓拍图宽度
extern const char* kMethodSnapshotOutputWidth;
/// 配置字段：设置抓拍模块输出抓拍图高度
extern const char* kMethodSnapshotOutputHeight;
/// 配置字段：设置抓拍图外扩系数
extern const char* kMethodScaleRate;
/// 配置字段：设置每个目标触发抓拍所需的帧数阈值
extern const char* kMethodBeginPostFrameThr;
/// 配置字段：设置重抓拍阈值，该阈值大于kMethodBeginPostFrameThr时才生效
extern const char* kMethodResnapValue;

}  // namespace xproto

#endif   //  _SMARTPLUGIN_SRC_UTILS_METHOD_CONST_H_
