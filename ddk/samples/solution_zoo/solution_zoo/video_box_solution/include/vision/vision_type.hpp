/**
 *  Copyright (c) 2016 Horizon Robotics. All rights reserved.
 *  @file vision_type.hpp
 *  \~English @brief this c++ header file defines the vision related data
 * structure that are used in IOT, including face, hand
 *
 */
#ifndef VISION_TYPE_VISION_TYPE_HPP_
#define VISION_TYPE_VISION_TYPE_HPP_

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>



#include "vision/vision_type_common.h"

namespace hobot {
namespace vision {
/**
 * \~Chinese @brief Async Data
 */
struct AsyncData {
  std::shared_ptr<void> bpu_model = nullptr;  // bpu handle
  std::shared_ptr<void> input_tensors =
      nullptr;  // need release after run model
  std::shared_ptr<void> output_tensors =
      nullptr;  // need release after parse result
  std::shared_ptr<void> task_handle = nullptr;  // async task handle

  std::shared_ptr<void> bpu_boxes;  // std::vector<BPU_BBOX>, for resizer model
  std::vector<int> valid_box;

  // 原始图像大小
  int src_image_width;
  int src_image_height;

  // 模型输入大小
  int model_input_height;
  int model_input_width;
};

/**
 * \~Chinese @brief 基础图像帧
 */
struct ImageFrame {
  virtual ~ImageFrame() {}
  /// \~Chinese 图片编码方式
  HorizonVisionPixelFormat pixel_format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatNone;
  uint32_t channel_id = 0;
  uint64_t time_stamp = 0;
  uint64_t frame_id = 0;
  std::string type = "";
  std::string image_name = "";
  /// \~Chinese 图像数据
  virtual uint64_t Data() = 0;
  /// \~Chinese UV分量数据
  virtual uint64_t DataUV() = 0;
  /// \~Chinese 图片大小
  virtual uint32_t DataSize() = 0;
  /// \~Chinese UV分量大小
  virtual uint32_t DataUVSize() = 0;
  /// \~Chinese 宽度
  virtual uint32_t Width() = 0;
  /// \~Chinese 高度
  virtual uint32_t Height() = 0;
  /// \~Chinese 长度
  virtual uint32_t Stride() = 0;
  /// \~Chinese uv长度
  virtual uint32_t StrideUV() = 0;
};
/**
 * \~Chinese @brief 存储opencv cvmat 信息的图像帧
 */
struct CVImageFrame : public ImageFrame {
  CVImageFrame() { type = "CVImageFrame"; }
  cv::Mat img;
  /// \~Chinese 图像数据
  uint64_t Data() override { return reinterpret_cast<uint64_t>(img.data); }
  /// \~Chinese uv分量数据
  uint64_t DataUV() override { return 0; }
  /// \~Chinese 图片大小
  uint32_t DataSize() override { return img.total() * img.channels(); }
  /// \~Chinese uv分量大小
  uint32_t DataUVSize() override { return 0; }
  /// \~Chinese 宽度
  uint32_t Width() override {
    switch (pixel_format) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatX2SRC:
      case kHorizonVisionPixelFormatX2PYM:
      case kHorizonVisionPixelFormatX3SRC:
      case kHorizonVisionPixelFormatX3PYM:
      case kHorizonVisionPixelFormatPYM:
      case kHorizonVisionPixelFormatImageContainer: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawRGB565:
      case kHorizonVisionPixelFormatRawBGR:
      case kHorizonVisionPixelFormatRawGRAY:
      case kHorizonVisionPixelFormatRawNV21:
      case kHorizonVisionPixelFormatRawNV12:
      case kHorizonVisionPixelFormatRawRGBA:
      case kHorizonVisionPixelFormatRawBGRA:
      case kHorizonVisionPixelFormatRawI420: {
        return img.cols;
      }
    }
    return 0;
  }
  /// \~Chinese 高度
  uint32_t Height() override {
    switch (pixel_format) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatX2SRC:
      case kHorizonVisionPixelFormatX2PYM:
      case kHorizonVisionPixelFormatX3SRC:
      case kHorizonVisionPixelFormatX3PYM:
      case kHorizonVisionPixelFormatPYM:
      case kHorizonVisionPixelFormatImageContainer: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawRGB565:
      case kHorizonVisionPixelFormatRawBGR:
      case kHorizonVisionPixelFormatRawRGBA:
      case kHorizonVisionPixelFormatRawBGRA:
      case kHorizonVisionPixelFormatRawGRAY: {
        return img.rows;
      }
      case kHorizonVisionPixelFormatRawNV21:
      case kHorizonVisionPixelFormatRawNV12:
      case kHorizonVisionPixelFormatRawI420: {
        return img.rows / 3 * 2;
      }
    }
    return 0;
  }
  /// \~Chinese 长度
  uint32_t Stride() override {
    switch (pixel_format) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatX2SRC:
      case kHorizonVisionPixelFormatX2PYM:
      case kHorizonVisionPixelFormatX3SRC:
      case kHorizonVisionPixelFormatX3PYM:
      case kHorizonVisionPixelFormatPYM:
      case kHorizonVisionPixelFormatImageContainer: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawRGB565:
      case kHorizonVisionPixelFormatRawBGR:
      case kHorizonVisionPixelFormatRawGRAY:
      case kHorizonVisionPixelFormatRawNV21:
      case kHorizonVisionPixelFormatRawNV12:
      case kHorizonVisionPixelFormatRawRGBA:
      case kHorizonVisionPixelFormatRawBGRA:
      case kHorizonVisionPixelFormatRawI420: {
        return img.cols * img.channels();
      }
    }
    return 0;
  }
  /// \~Chinese uv长度
  uint32_t StrideUV() override {
    switch (pixel_format) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatX2SRC:
      case kHorizonVisionPixelFormatX2PYM:
      case kHorizonVisionPixelFormatX3SRC:
      case kHorizonVisionPixelFormatX3PYM:
      case kHorizonVisionPixelFormatPYM:
      case kHorizonVisionPixelFormatImageContainer:
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawRGB565:
      case kHorizonVisionPixelFormatRawBGR:
      case kHorizonVisionPixelFormatRawRGBA:
      case kHorizonVisionPixelFormatRawBGRA:
      case kHorizonVisionPixelFormatRawGRAY: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawNV21:
      case kHorizonVisionPixelFormatRawNV12: {
        return img.cols;
      }
      case kHorizonVisionPixelFormatRawI420: {
        return img.cols / 2;
      }
    }
    return 0;
  }
};

#define DOWN_SCALE_MAIN_MAX 6
#define DOWN_SCALE_MAX 24
#define UP_SCALE_MAX 6
struct ImageLevelInfo {
  uint16_t width;
  uint16_t height;
  uint16_t stride;
  uint64_t y_paddr;
  uint64_t c_paddr;
  uint64_t y_vaddr;
  uint64_t c_vaddr;
};

struct SrcImageFrame : public ImageFrame {
  SrcImageFrame() {
    type = "X3SrcImageFrame";
    pixel_format = HorizonVisionPixelFormat::kHorizonVisionPixelFormatX3SRC;
  }
  ImageLevelInfo src_info;
  void *context = nullptr;

  /// \~Chinese 图像数据
  uint64_t Data() override { return src_info.y_vaddr; }
  /// \~Chinese uv分量数据
  uint64_t DataUV() override { return src_info.c_vaddr; }
  /// \~Chinese 图片大小
  uint32_t DataSize() override { return Stride() * Height(); }
  /// \~Chinese uv分量大小
  uint32_t DataUVSize() override { return Stride() * Height() / 2; }
  /// \~Chinese 宽度
  uint32_t Width() override { return src_info.width; }
  /// \~Chinese 高度
  uint32_t Height() override { return src_info.height; }
  /// \~Chinese 长度
  uint32_t Stride() override { return src_info.stride; }
  /// \~Chinese uv长度
  uint32_t StrideUV() override { return Stride(); }
};

struct PymImageFrame : public ImageFrame {
  PymImageFrame() {
    type = "PymImageFrame";
    pixel_format = HorizonVisionPixelFormat::kHorizonVisionPixelFormatPYM;
  }
  int ds_pym_total_layer;
  int us_pym_total_layer;
  ImageLevelInfo down_scale[DOWN_SCALE_MAX];
  ImageLevelInfo up_scale[UP_SCALE_MAX];
  ImageLevelInfo down_scale_main[DOWN_SCALE_MAIN_MAX];
  ImageLevelInfo reserved[4];
  int pym_layer = 0;
  void *context = nullptr;

  /// \~Chinese y分量数据
  uint64_t Data() override {
    assert(pym_layer >= 0 && pym_layer < DOWN_SCALE_MAX);
    return down_scale[pym_layer].y_vaddr;
  }
  /// \~Chinese uv分量数据
  uint64_t DataUV() override {
    assert(pym_layer >= 0 && pym_layer < DOWN_SCALE_MAX);
    return down_scale[pym_layer].c_vaddr;
  }
  /// \~Chinese y分量大小
  uint32_t DataSize() override { return Stride() * Height(); }
  uint32_t DataUVSize() override { return StrideUV() * Height() / 2; }
  /// \~Chinese 宽度
  uint32_t Width() override {
    assert(pym_layer >= 0 && pym_layer < DOWN_SCALE_MAX);
    return down_scale[pym_layer].width;
  }
  /// \~Chinese 高度
  uint32_t Height() override {
    assert(pym_layer >= 0 && pym_layer < DOWN_SCALE_MAX);
    return down_scale[pym_layer].height;
  }
  /// \~Chinese 长度
  uint32_t Stride() override {
    assert(pym_layer >= 0 && pym_layer < DOWN_SCALE_MAX);
    return down_scale[pym_layer].stride;
  }
  /// \~Chinese uv长度
  uint32_t StrideUV() override { return Stride(); }
};
#if 0
inline void Convert(const ImageLevelInfo &addr,
                    bpu_predict_x3::x2_addr_info_t &bpu_predict_addr) {
  bpu_predict_addr.width = addr.width;
  bpu_predict_addr.height = addr.height;
  bpu_predict_addr.step = addr.stride;
  bpu_predict_addr.y_paddr = addr.y_paddr;
  bpu_predict_addr.c_paddr = addr.c_paddr;
  bpu_predict_addr.y_vaddr = addr.y_vaddr;
  bpu_predict_addr.c_vaddr = addr.c_vaddr;
}

inline void Convert(const PymImageFrame &image,
                    bpu_predict_x3::PyramidResult &bpu_predict_pyramid) {
  bpu_predict_x3::img_info_t &bpu_predict_image =
      bpu_predict_pyramid.result_info;
  bpu_predict_pyramid.valid = true;
  bpu_predict_image.ds_pym_layer = image.ds_pym_total_layer;
  bpu_predict_image.us_pym_layer = image.us_pym_total_layer;
  for (int i = 0; i < DOWN_SCALE_MAX; ++i) {
    Convert(image.down_scale[i], bpu_predict_image.down_scale[i]);
  }
  for (int i = 0; i < DOWN_SCALE_MAIN_MAX; ++i) {
    Convert(image.down_scale_main[i], bpu_predict_image.down_scale_main[i]);
  }
}
#endif

/**
 * \~Chinese @brief 2D坐标点
 */
template <typename Dtype>
struct Point_ {
  inline Point_() {}
  inline Point_(Dtype x_, Dtype y_, float score_ = 0.0)
      : x(x_), y(y_), score(score_) {}

  Dtype x = 0;
  Dtype y = 0;
  float score = 0.0;
};
typedef Point_<float> Point;
/**
 * \~Chinese @brief 3D坐标点
 */
template <typename Dtype>
struct Point3d_ {
  Dtype x = 0;
  Dtype y = 0;
  Dtype z = 0;
  float score = 0.0;
  inline Point3d_() {}
  inline Point3d_(Dtype x_, Dtype y_, Dtype z_, float score_ = 0.0)
      : x(x_), y(y_), z(z_), score(score_) {}
  inline explicit Point3d_(Point_<Dtype> point)
      : x(point.x), y(point.y), z(0), score(point.score) {}
  inline Point3d_(Point_<Dtype> point, Dtype z_)
      : x(point.x), y(point.y), z(z_), score(point.score) {}
};
typedef Point3d_<float> Point3d;
/**
 * \~Chinese @brief 2D坐标点集合，可用于存储关键点等结果
 */
struct Points {
  std::vector<Point> values;
  /// \~Chinese 置信度
  float score = 0.0;
};

typedef Points Landmarks;
/**
 * \~Chinese @brief 检测框
 */
template <typename Dtype>
struct BBox_ {
  inline BBox_() {}
  inline BBox_(Dtype x1_, Dtype y1_, Dtype x2_, Dtype y2_, float score_ = 0.0f,
               int32_t id_ = -1, const std::string &category_name_ = "") {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    id = id_;
    score = score_;
    category_name = category_name_;
  }
  inline Dtype Width() const { return (x2 - x1); }
  inline Dtype Height() const { return (y2 - y1); }
  inline Dtype CenterX() const { return (x1 + (x2 - x1) / 2); }
  inline Dtype CenterY() const { return (y1 + (y2 - y1) / 2); }

  inline friend std::ostream &operator<<(std::ostream &out, BBox_ &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }

  inline friend std::ostream &operator<<(std::ostream &out, const BBox_ &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }

  Dtype x1 = 0;
  Dtype y1 = 0;
  Dtype x2 = 0;
  Dtype y2 = 0;
  float score = 0.0;
  int32_t id = 0;
  float rotation_angle = 0.0;
  std::string category_name = "";
};
typedef BBox_<float> BBox;

template <typename Dtype>
struct Oriented_BBox_ {
  inline Oriented_BBox_() {}
  inline Oriented_BBox_(Dtype x1_, Dtype y1_, Dtype x2_, Dtype y2_, Dtype x3_,
                        Dtype y3_, Dtype x4_, Dtype y4_, float score_ = 0.0f,
                        int32_t id_ = -1,
                        const std::string &category_name_ = "") {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    x3 = x3_;
    y3 = y3_;
    x4 = x4_;
    y4 = y4_;
    id = id_;
    score = score_;
    category_name = category_name_;
  }
  inline Dtype CenterX() const { return (x1 + x2 + x3 + x4) / 4; }
  inline Dtype CenterY() const { return (y1 + y2 + y3 + y4) / 4; }

  // 默认点1-2、3-4是Length1；点2-3、1-4 Length2
  inline Dtype Length1() const {
    Dtype l1 = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    Dtype l2 = sqrt(pow((x4 - x3), 2) + pow((y4 - y3), 2));
    return (l1 + l2) / 2;
  }
  inline Dtype Length2() const {
    Dtype l1 = sqrt(pow((x4 - x1), 2) + pow((y4 - y1), 2));
    Dtype l2 = sqrt(pow((x3 - x2), 2) + pow((y3 - y2), 2));
    return (l1 + l2) / 2;
  }
  // theta is the angble between the longside and horizontal line
  // theta: [0, PI]
  inline float Theta() const {
    float theta;
    const float PI = 3.1415926;
    Dtype length1 = Length1();
    Dtype length2 = Length2();
    if (length1 > length2) {
      theta = (atan2(y2 - y1, x2 - x1) + atan2(y3 - y4, x3 - x4)) / 2;
    } else {
      theta = (atan2(y2 - y3, x2 - x3) + atan2(y1 - y4, x1 - x4)) / 2;
    }  // theta: [-PI, PI]
    if (theta < 0) {
      theta = PI + theta;
    }
    return theta;
  }
  inline friend std::ostream &operator<<(std::ostream &out,
                                         Oriented_BBox_ &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " x3: " << bbox.x3 << " y3: " << bbox.y3
        << " x4: " << bbox.x4 << " y4: " << bbox.y4 << ") score: " << bbox.score
        << " id: " << bbox.id;
    return out;
  }

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const Oriented_BBox_ &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " x3: " << bbox.x3 << " y3: " << bbox.y3
        << " x4: " << bbox.x4 << " y4: " << bbox.y4 << ") score: " << bbox.score
        << " id: " << bbox.id;
    return out;
  }

  Dtype x1 = 0;
  Dtype y1 = 0;
  Dtype x2 = 0;
  Dtype y2 = 0;
  Dtype x3 = 0;
  Dtype y3 = 0;
  Dtype x4 = 0;
  Dtype y4 = 0;
  float score = 0.0;
  int32_t id = 0;
  std::string category_name = "";
};
typedef Oriented_BBox_<float> Oriented_BBox;

/**
 * \~Chinese @brief 单精度浮点数组，可用于存储特征值、质量结果等
 */
struct FloatArray {
  std::vector<float> values;
  /// \~Chinese 置信度
  float score = 0.0;
};
typedef FloatArray Feature;
/**
 * \~Chinese @brief 人体分割
 */
struct Segmentation {
  std::vector<float> values;
  std::vector<float> pixel_score;
  int32_t width = 0;
  int32_t height = 0;
  /// \~Chinese  置信度
  float score = 0.0;
};
/**
 * \~Chinese @brief 人脸姿态
 */
struct Pose3D {
  /// \~Chinese 俯仰角度
  float pitch = 0.0;
  /// \~Chinese 左右摇头角度
  float yaw = 0.0;
  /// \~Chinese 侧头角度
  float roll = 0.0;
  /// \~Chinese 置信度
  float score = 0.0;
};
/**
 * \~Chinese @brief 年龄
 */
struct Age {
  /// \~Chinese 年龄分类
  int32_t value = -1;
  /// \~Chinese 年龄段下限
  int32_t min = -1;
  /// \~Chinese 年龄段上限
  int32_t max = -1;
  /// \~Chinese 置信度
  float score = 0.0;
};

/**
 * \~Chinese @brief 属性类检测结果，可用于性别、眼镜、活体等结果。
 */

template <typename DType>
struct Attribute {
  DType value = 0;
  /// \~Chinese 置信度
  float score = 0.0;
};

typedef Attribute<int32_t> Gender;
typedef Attribute<int32_t> Glass;
typedef Attribute<int32_t> Quality;
typedef Attribute<int32_t> AntiSpoofing;
typedef Attribute<int32_t> BreathingMask;
/**
 * \~Chinese @brief 抓拍信息
 */
template <typename DType>
struct SnapshotInfo {
  SnapshotInfo()
      : type(0),
        track_id(-1),
        select_value(0),
        origin_image_frame(nullptr),
        snap(nullptr) {}

  /**
   * \~Chinese @brief 抓拍图坐标转换接口
   *
   * \~Chinese @param in [in] 相对于原始帧的一组坐标
   * \~Chinese @return Points 相对于抓拍图的一组坐标
   */
  virtual Points PointsToSnap(const Points &in) { return Points(); }

  /// \~Chinese 抓拍类型
  int32_t type;
  /// track id
  int32_t track_id;
  /// \~Chinese 优选参考值
  float select_value;
  /// \~Chinese 抓拍发生时原始帧数据
  std::shared_ptr<ImageFrame> origin_image_frame;
  /// \~Chinese 抓拍图数据
  std::shared_ptr<ImageFrame> snap;
  /// \~Chinese 用户数据数组
  std::vector<DType> userdata;
};

struct SnapshotState {
  SnapshotState()
      : id(-1),
        overall_quality(0),
        select_index(-1),
        snap_en(0),
        snap_stop(0),
        snap_repeat(0) {}
  /// \~Chinese 抓拍人脸ID
  int32_t id;
  /// \~Chinese 抓拍人脸总体打分
  float overall_quality;
  /// \~Chinese 抓拍人脸外框
  BBox box;
  /// \~Chinese 抓拍人脸缓冲替换序号
  int32_t select_index;
  /// \~Chinese 人脸抓拍有效标识
  uint8_t snap_en : 1;
  /// \~Chinese 优选抓拍结束标识
  uint8_t snap_stop : 1;
  /// \~Chinese 重复抓拍开始标识
  uint8_t snap_repeat : 1;
};

}  // namespace vision
}  // namespace hobot

#endif  // VISION_TYPE_VISION_TYPE_HPP_
