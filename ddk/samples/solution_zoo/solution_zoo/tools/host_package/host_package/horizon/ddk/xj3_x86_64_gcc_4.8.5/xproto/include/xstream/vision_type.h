/**
 *  Copyright (c) 2016 Horizon Robotics. All rights reserved.
 *  @file vision_type.h
 *  \~English @brief this c++ header file defines the vision related data
 * structure that are used in IOT, including face, hand
 *
 */
#ifndef VISION_TYPE_VISION_TYPE_H_
#define VISION_TYPE_VISION_TYPE_H_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>     /* assert */

#include "xstream/version.h"
#include "xstream/xstream_data.h"

#define PYRAMID_DOWN_SCALE_MAX  24
#define PYRAMID_UP_SCALE_MAX  6
#define PYRAMID_DOWN_SCALE_MAIN_MAX 6

namespace xstream {

/**
 * \~Chinese @brief 图片编码方式
 */
enum XSTREAM_EXPORT HorizonVisionPixelFormat {
  /// \~Chinese 普通图片文件的二进制流
  /// 默认BGR, 需要与二进制流文件长度一起配套使用
      kHorizonVisionPixelFormatNone = 0,
  /// RGB
      kHorizonVisionPixelFormatRawRGB,
  /// BGR
      kHorizonVisionPixelFormatRawBGR,
  /// YUV420SP:NV12
      kHorizonVisionPixelFormatRawNV12,
  /// \~Chinese 图片标准格式，比如jpeg
      kHorizonVisionPixelFormatImageContainer,
};

/**
 * \~Chinese @brief 基础图像帧
 */
struct XSTREAM_EXPORT ImageFrame : public xstream::BaseData {
  ImageFrame() { type_ = "ImageFrame"; }
  virtual ~ImageFrame() {}
  /// \~Chinese 图片编码方式
  HorizonVisionPixelFormat pixel_format_ =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatNone;
  uint32_t channel_id_ = 0;
  uint64_t time_stamp_ = 0;
  uint64_t frame_id_ = 0;
  std::string image_name_ = "";
  /// \~Chinese 图像数据
  virtual uint64_t Data() { return 0; }
  /// \~Chinese 金字塔图像数据
  virtual uint64_t Data(int pym_layer) { return 0; }
  /// \~Chinese UV分量数据
  virtual uint64_t DataUV() { return 0; }
  /// \~Chinese 金字塔UV分量数据
  virtual uint64_t DataUV(int pym_layer) { return 0; }
  /// \~Chinese 图片大小
  virtual uint32_t DataSize() { return 0; }
  /// \~Chinese 金字塔图片大小
  virtual uint32_t DataSize(int pym_layer) { return 0; }
  /// \~Chinese UV分量大小
  virtual uint32_t DataUVSize() { return 0; }
  /// \~Chinese 金字塔UV分量大小
  virtual uint32_t DataUVSize(int pym_layer) { return 0; }
  /// \~Chinese 宽度
  virtual uint32_t Width() { return 0; }
  /// \~Chinese 金字塔宽度
  virtual uint32_t Width(int pym_layer) { return 0; }
  /// \~Chinese 高度
  virtual uint32_t Height() { return 0; }
  /// \~Chinese 金字塔高度
  virtual uint32_t Height(int pym_layer) { return 0; }
  /// \~Chinese 长度
  virtual uint32_t Stride() { return 0; }
  /// \~Chinese 金字塔长度
  virtual uint32_t Stride(int pym_layer) { return 0; }
  /// \~Chinese uv长度
  virtual uint32_t StrideUV() { return 0; }
  /// \~Chinese 金字塔uv长度
  virtual uint32_t StrideUV(int pym_layer) { return 0; }
};

/**
 * \~Chinese @brief 存储一般图像信息的图像帧
 */
struct XSTREAM_EXPORT RawDataImageFrame : public ImageFrame {
  RawDataImageFrame() {
    type_ = "RawDataImageFrame";
    data_ = nullptr;
    width_ = 0;
    height_ = 0;
    image_size_ = 0;
  }
  uint8_t* data_;   // stride == width_
  int width_;
  int height_;
  uint64_t image_size_;
  /// \~Chinese 图像数据
  uint64_t Data() override { return reinterpret_cast<uint64_t>(data_); }
  /// \~Chinese uv分量数据
  uint64_t DataUV() override {
    switch (pixel_format_) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatImageContainer:
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawBGR: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawNV12: {
        return reinterpret_cast<uint64_t>(data_ + width_ * height_);
      }
    }
    return 0;
  }
  /// \~Chinese 图片大小
  uint32_t DataSize() override {
    switch (pixel_format_) {
      case kHorizonVisionPixelFormatNone: {
        return 0;
      }
      case kHorizonVisionPixelFormatImageContainer: {
        return image_size_;
      }
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawBGR: {
        return width_ * height_ * 3;
      }
      case kHorizonVisionPixelFormatRawNV12: {
        return width_ * height_ * 3 / 2;
      }
    }
    return 0;
  }
  /// \~Chinese uv分量大小
  uint32_t DataUVSize() override {
    switch (pixel_format_) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatImageContainer:
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawBGR: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawNV12: {
        return width_ * height_ / 2;
      }
    }
    return 0;
  }
  /// \~Chinese 宽度
  uint32_t Width() override {
    switch (pixel_format_) {
      case kHorizonVisionPixelFormatNone: {
        return 0;
      }
      case kHorizonVisionPixelFormatImageContainer:
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawBGR:
      case kHorizonVisionPixelFormatRawNV12: {
        return width_;
      }
    }
    return 0;
  }
  /// \~Chinese 高度
  uint32_t Height() override {
    switch (pixel_format_) {
      case kHorizonVisionPixelFormatNone: {
        return 0;
      }
      case kHorizonVisionPixelFormatImageContainer:
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawBGR:
      case kHorizonVisionPixelFormatRawNV12: {
        return height_;
      }
    }
    return 0;
  }
  /// \~Chinese 长度
  uint32_t Stride() override {
    switch (pixel_format_) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatImageContainer: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawBGR:
      case kHorizonVisionPixelFormatRawNV12: {
        return width_;
      }
    }
    return 0;
  }
  /// \~Chinese uv长度
  uint32_t StrideUV() override {
    switch (pixel_format_) {
      case kHorizonVisionPixelFormatNone:
      case kHorizonVisionPixelFormatImageContainer:
      case kHorizonVisionPixelFormatRawRGB:
      case kHorizonVisionPixelFormatRawBGR: {
        return 0;
      }
      case kHorizonVisionPixelFormatRawNV12: {
        return width_;
      }
    }
    return 0;
  }
};

/**
 * \~Chinese @brief 金字塔图像基础数据结构
 */
struct XSTREAM_EXPORT PyramidAddrInfo {
  uint16_t width;
  uint16_t height;
  uint16_t step;
  uint64_t y_paddr;
  uint64_t c_paddr;
  uint64_t y_vaddr;
  uint64_t c_vaddr;
};

/**
 * \~Chinese @brief 金字塔图像数据结构
 */
struct XSTREAM_EXPORT PyramidImageInfo {
  /// \~Chinese vio_buffer索引
  int slot_id;                 // getted slot buff
  /// \~Chinese 帧id
  uint64_t frame_id;
  /// \~Chinese 时间戳
  int64_t timestamp;           // BT from Hisi; mipi & dvp from kernel time
  /// \~Chinese 图像格式，仅支持yuv420sp(nv12)
  int img_format;              // now only support yuv420sp
  /// \~Chinese 下采样层数
  int ds_pym_layer;            // get down scale layers
  /// \~Chinese 上采样层数
  int us_pym_layer;            // get up scale layers
  /// \~Chinese 原始图像数据
  PyramidAddrInfo src_img;     // for x2 src img = crop img
  /// \~Chinese 金字塔缩放层数据
  PyramidAddrInfo down_scale[PYRAMID_DOWN_SCALE_MAX];
  /// \~Chinese 金字塔放大层数据
  PyramidAddrInfo up_scale[PYRAMID_UP_SCALE_MAX];
  /// \~Chinese 金字塔基础层数据，0、4、8、12...
  PyramidAddrInfo down_scale_main[PYRAMID_DOWN_SCALE_MAIN_MAX];
  int cam_id;
};

/**
 * \~Chinese @brief 存储图像金字塔信息的图像帧
 */
struct XSTREAM_EXPORT PyramidImageFrame : public ImageFrame {
  PyramidImageFrame() {
    type_ = "PyramidImageFrame";
    pixel_format_ = HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawNV12;
  }
  /// \~English now only support nv12.
  /// \~Chinese 当前仅支持NV12格式的图像
  PyramidImageInfo img_;
  void *context_ = nullptr;

  /// \~Chinese y分量数据
  uint64_t Data(int pym_layer) override {
    assert(pym_layer >= 0 && pym_layer < PYRAMID_DOWN_SCALE_MAX);
    return img_.down_scale[pym_layer].y_vaddr;
  }
  /// \~Chinese uv分量数据
  uint64_t DataUV(int pym_layer) override {
    assert(pym_layer >= 0 && pym_layer < PYRAMID_DOWN_SCALE_MAX);
    return img_.down_scale[pym_layer].c_vaddr;
  }
  /// \~Chinese y分量大小
  uint32_t DataSize(int pym_layer) override {
     return Stride(pym_layer) * Height(pym_layer);
  }
  uint32_t DataUVSize(int pym_layer) override {
    return StrideUV(pym_layer) * Height(pym_layer) / 2;
  }
  /// \~Chinese 宽度
  uint32_t Width(int pym_layer) override {
    assert(pym_layer >= 0 && pym_layer < PYRAMID_DOWN_SCALE_MAX);
    return img_.down_scale[pym_layer].width;
  }
  /// \~Chinese 高度
  uint32_t Height(int pym_layer) override {
    assert(pym_layer >= 0 && pym_layer < PYRAMID_DOWN_SCALE_MAX);
    return img_.down_scale[pym_layer].height;
  }
  /// \~Chinese 长度
  uint32_t Stride(int pym_layer) override {
    assert(pym_layer >= 0 && pym_layer < PYRAMID_DOWN_SCALE_MAX);
    return img_.down_scale[pym_layer].step;
  }
  /// \~Chinese uv长度
  uint32_t StrideUV(int pym_layer) override { return Stride(pym_layer); }
};

/**
 * \~Chinese @brief 存储图像金字塔初始层数据，即金字塔第0层
 */
struct XSTREAM_EXPORT OriginalPyramidImageFrame : public ImageFrame {
  OriginalPyramidImageFrame() {
    type_ = "OriginalPyramidImageFrame";
    pixel_format_ = HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawNV12;
  }
  PyramidAddrInfo src_info_;
  void *context_ = nullptr;

  /// \~Chinese 图像数据
  uint64_t Data() override { return src_info_.y_vaddr; }
  /// \~Chinese uv分量数据
  uint64_t DataUV() override { return src_info_.c_vaddr; }
  /// \~Chinese 图片大小
  uint32_t DataSize() override { return Stride()*Height(); }
  /// \~Chinese uv分量大小
  uint32_t DataUVSize() override { return Stride()*Height() / 2; }
  /// \~Chinese 宽度
  uint32_t Width() override { return src_info_.width; }
  /// \~Chinese 高度
  uint32_t Height() override { return src_info_.height; }
  /// \~Chinese 长度
  uint32_t Stride() override { return src_info_.step; }
  /// \~Chinese uv长度
  uint32_t StrideUV() override { return Stride(); }
};

/**
 * \~Chinese @brief 单精度浮点数组，可用于存储特征值、质量结果等
 */
// 模板类类型命名使用下划线为后缀
template <typename DType>
struct XSTREAM_EXPORT DataArray_ : public xstream::BaseData {
  std::vector<DType> values_;
  float score_ = 0.0;
  std::string specific_type_ = "";

  DataArray_() { type_ = "DataArray"; }
  inline friend std::ostream &operator<<(std::ostream &out, DataArray_ &array) {
    out << "(";
    for (auto value : array.values_)
      out << value;
    out << ")";
    return out;
  }
};
typedef DataArray_<float> FloatFeature;          // 浮点特征数据
typedef DataArray_<char> CharEncryptedFeature;   // 加密特征数据

/**
 * \~Chinese @brief 属性类检测结果，可用于性别、眼镜、活体、分类等结果
 */
// 模板类类型命名使用下划线为后缀
template <typename DType>
struct XSTREAM_EXPORT Attribute_ : public xstream::BaseData {
  DType value_;
  float score_ = 0.0;
  std::string specific_type_ = "";

  Attribute_() { type_ = "Attribute"; }
  Attribute_(int value, float score, const std::string specific_type = "")
      : value_(value), score_(score), specific_type_(specific_type) {
    type_ = "Attribute";
  }
  friend bool operator>(const Attribute_ &lhs, const Attribute_ &rhs) {
    return (lhs.score_ > rhs.score_);
  }
  inline friend std::ostream &operator<<(std::ostream &out, Attribute_ &attr) {
    out << "(value: " << attr.value_ << ", score: " << attr.score_;
    return out;
  }
};
typedef Attribute_<int32_t> Gender;           // 性别
typedef Attribute_<int32_t> Glass;            // 眼镜
typedef Attribute_<int32_t> Quality;          // 质量
typedef Attribute_<int32_t> AntiSpoofing;     // 活体
typedef Attribute_<int32_t> BreathingMask;    // 口罩
typedef Attribute_<int32_t> Classification;   // 分类
// 车牌号, value_使用默认值, specific_type为实际结果
typedef Attribute_<int32_t> PlateNum;

/**
 * \~Chinese @brief 2D坐标点
 */
// 模板类类型命名使用下划线为后缀
template <typename Dtype>
struct XSTREAM_EXPORT Point_ : public xstream::BaseData {
  Dtype x_ = 0;
  Dtype y_ = 0;
  float score_ = 0.0;

  inline Point_() { type_ = "Point"; }
  inline Point_(Dtype x, Dtype y, float score = 0.0)
      : x_(x), y_(y), score_(score) {}

  inline friend std::ostream &operator<<(std::ostream &out, Point_ &p) {
    out << "(x: " << p.x_ << " y: " << p.y_ << " score: " << p.score_ << ")";
    return out;
  }
};
typedef Point_<float> Point;

/**
 * \~Chinese @brief 3D坐标点
 */
// 模板类类型命名使用下划线为后缀
template <typename Dtype>
struct XSTREAM_EXPORT Point3D_ : public xstream::BaseData {
  Dtype x_ = 0;
  Dtype y_ = 0;
  Dtype z_ = 0;
  float score_ = 0.0;

  inline Point3D_() { type_ = "Point3D"; }
  inline Point3D_(Dtype x, Dtype y, Dtype z, float score = 0.0)
      : x_(x), y_(y), z_(z), score_(score) {
    type_ = "Point3D";
  }
  inline explicit Point3D_(Point_<Dtype> point)
      : x_(point.x_), y_(point.y_), z_(0), score_(point.score_) {
    type_ = "Point3D";
  }
  inline Point3D_(Point_<Dtype> point, Dtype z)
      : x_(point.x_), y_(point.y_), z_(z), score_(point.score_) {
    type_ = "Point3D";
  }
  inline friend std::ostream &operator<<(std::ostream &out, Point3D_ &p) {
    out << "(x: " << p.x_ << " y: " << p.y_ << " z: " << p.z_
        << " score: " << p.score_ << ")";
    return out;
  }
};
typedef Point3D_<float> Point3D;    // 浮点3D坐标点集合

/**
 * \~Chinese @brief 2D坐标点集合，可用于存储关键点等结果
 */
// 模板类类型命名使用下划线为后缀
template <typename Dtype>
struct XSTREAM_EXPORT Points_ : public xstream::BaseData {
  std::vector<Point_<Dtype>> values_;
  float score_ = 0.0;
  std::string specific_type_ = "";

  Points_() { type_ = "Points"; }
  inline friend std::ostream &operator<<(std::ostream &out, Points_ &points) {
    for (auto p : points.values_) {
      out << p << " ";
    }
    return out;
  }
};
typedef Points_<float> Landmarks;  // 关键点
typedef Points_<float> FloatPoints;  // 浮点2D坐标点集合

/**
 * \~Chinese @brief 检测框
 */
// 模板类类型命名使用下划线为后缀
template <typename Dtype>
struct XSTREAM_EXPORT BBox_ : public xstream::BaseData {
  Dtype x1_ = 0;
  Dtype y1_ = 0;
  Dtype x2_ = 0;
  Dtype y2_ = 0;
  float score_ = 0.0;
  int32_t id_ = 0;
  float rotation_angle_ = 0.0;
  std::string specific_type_ = "";

  inline BBox_() { type_ = "BBox"; }
  inline BBox_(Dtype x1, Dtype y1, Dtype x2, Dtype y2, float score = 0.0f,
               int32_t id = -1, const std::string &specific_type = "") {
    x1_ = x1;
    y1_ = y1;
    x2_ = x2;
    y2_ = y2;
    id_ = id;
    score_ = score;
    specific_type_ = specific_type;
    type_ = "BBox";
  }
  inline Dtype Width() const { return (x2_ - x1_); }
  inline Dtype Height() const { return (y2_ - y1_); }
  inline Dtype CenterX() const { return (x1_ + (x2_ - x1_) / 2); }
  inline Dtype CenterY() const { return (y1_ + (y2_ - y1_) / 2); }

  inline friend std::ostream &operator<<(std::ostream &out, BBox_ &bbox) {
    out << "( x1: " << bbox.x1_ << " y1: " << bbox.y1_ << " x2: " << bbox.x2_
        << " y2: " << bbox.y2_ << " score: " << bbox.score_ << " )";
    return out;
  }
};
typedef BBox_<float> BBox;
typedef std::shared_ptr<BBox> BBoxPtr;

/**
 * \~Chinese @brief 人体分割
 */
struct XSTREAM_EXPORT Segmentation : public xstream::BaseData {
  std::vector<int> values_;
  std::vector<float> pixel_score_;
  std::string specific_type_ = "";
  int32_t width_ = 0;
  int32_t height_ = 0;
  float score_ = 0.0;

  Segmentation() { type_ = "Segmentation"; }
  inline friend std::ostream &operator<<(std::ostream &out, Segmentation &seg) {
    out << "(";
    for (auto value : seg.values_)
      out << value;
    out << ")";
    return out;
  }
};

/**
 * \~Chinese @brief 人脸姿态
 */
struct XSTREAM_EXPORT Pose3D : public xstream::BaseData {
  /// \~Chinese 俯仰角度
  float pitch_ = 0.0;
  /// \~Chinese 左右摇头角度
  float yaw_ = 0.0;
  /// \~Chinese 侧头角度
  float roll_ = 0.0;
  float score_ = 0.0;
  Pose3D() { type_ = "Pose3D"; }
  inline friend std::ostream &operator<<(std::ostream &out, Pose3D &pose) {
    out << "(pitch: " << pose.pitch_ << ", yaw: " << pose.yaw_
        << ", roll: " << pose.roll_ << ")";
    return out;
  }
};

/**
 * \~Chinese @brief 年龄
 */
struct XSTREAM_EXPORT Age : public xstream::BaseData {
  /// \~Chinese 年龄分类
  int32_t value_ = -1;
  /// \~Chinese 年龄段下限
  int32_t min_ = -1;
  /// \~Chinese 年龄段上限
  int32_t max_ = -1;
  float score_ = 0.0;
  Age() { type_ = "Age"; }
  inline friend std::ostream &operator<<(std::ostream &out, Age &age) {
    out << "(value: " << age.value_ << ", min: " << age.min_
        << ", max: " << age.max_ << ")";
    return out;
  }
};

/**
 * \~Chinese @brief 停车框
 */
struct XSTREAM_EXPORT ParkingBBox : public xstream::BaseData {
  float x1_ = 0;
  float y1_ = 0;
  float x2_ = 0;
  float y2_ = 0;
  float x3_ = 0;
  float y3_ = 0;
  float x4_ = 0;
  float y4_ = 0;
  float score_ = 0.0;
  int32_t id_ = 0;
  std::string specific_type_ = "ParkingBBox";

  inline ParkingBBox() { type_ = "ParkingBBox"; }
  inline ParkingBBox(
      float x1, float y1, float x2, float y2,
      float x3, float y3, float x4, float y4,
      float score = 0.0f, int32_t id = -1,
      const std::string &specific_type = "") {
    x1_ = x1;
    y1_ = y1;
    x2_ = x2;
    y2_ = y2;
    x3_ = x3;
    y3_ = y3;
    x4_ = x4;
    y4_ = y4;
    id_ = id;
    score_ = score;
    specific_type_ = specific_type;
    type_ = "ParkingBBox";
  }
  inline float CenterX() const { return (x1_ + x2_ + x3_ + x4_) / 4; }
  inline float CenterY() const { return (y1_ + y2_ + y3_ + y4_) / 4; }

  // 默认点1-2、3-4是Length1；点2-3、1-4 Length2
  inline float Length1() const {
    float l1 = sqrt(pow((x2_-x1_), 2) + pow((y2_-y1_), 2));
    float l2 = sqrt(pow((x4_-x3_), 2) + pow((y4_-y3_), 2));
    return (l1 + l2) / 2;
  }
  inline float Length2() const {
    float l1 = sqrt(pow((x4_-x1_), 2) + pow((y4_-y1_), 2));
    float l2 = sqrt(pow((x3_-x2_), 2) + pow((y3_-y2_), 2));
    return (l1 + l2) / 2;
  }
  // theta is the angble between the longside and horizontal line
  // theta: [0, PI]
  inline float Theta() const {
    float theta;
    const float PI = 3.1415926;
    float length1 = Length1();
    float length2 = Length2();
    if (length1 > length2) {
       theta = (atan2(y2_ - y1_, x2_ - x1_) + atan2(y3_ - y4_, x3_ - x4_)) / 2;
    } else {
       theta = (atan2(y2_ - y3_, x2_ - x3_) + atan2(y1_ - y4_, x1_ - x4_)) / 2;
    }  // theta: [-PI, PI]
    if (theta < 0) {
      theta = PI + theta;
    }
    return theta;
  }
  inline friend std::ostream &operator<<(std::ostream &out,
                                         ParkingBBox &bbox) {
    out << "( x1: " << bbox.x1_ << " y1: " << bbox.y1_
        << " x2: " << bbox.x2_ << " y2: " << bbox.y2_
        << " x3: " << bbox.x3_ << " y3: " << bbox.y3_
        << " x4: " << bbox.x4_ << " y4: " << bbox.y4_
        << ") score: " << bbox.score_ << " id: " << bbox.id_;
    return out;
  }
};

/**
 * \~Chinese @brief 抓拍信息
 */
// 模板类类型命名使用下划线为后缀
template <typename DType>
struct XSTREAM_EXPORT SnapshotInfo_ : public xstream::BaseData {
  /// \~Chinese 抓拍类型
  int32_t snap_type_ = 0;
  /// track id
  int32_t track_id_ = -1;
  /// \~Chinese 优选参考值
  float select_value_ = 0;
  /// \~Chinese 抓拍发生时原始帧数据
  std::shared_ptr<ImageFrame> origin_image_frame_ = nullptr;
  /// \~Chinese 抓拍图数据
  std::shared_ptr<ImageFrame> snap_ = nullptr;
  /// \~Chinese 用户数据数组
  std::vector<DType> userdata_;

  SnapshotInfo_() {
    type_ = "SnapshotInfo";
  }

  /**
   * \~Chinese @brief 抓拍图坐标转换接口
   *
   * \~Chinese @param in [in] 相对于原始帧的一组坐标
   * \~Chinese @return Points 相对于抓拍图的一组坐标
   */
  virtual FloatPoints PointsToSnap(const FloatPoints &in) { return in; }
};

}  // namespace xstream

#endif  // VISION_TYPE_VISION_TYPE_H_
