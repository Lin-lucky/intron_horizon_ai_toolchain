

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of yolov3_postprocess
 * @file   yolov3_postprocess.h
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */
#ifndef YOLOV3_POSTPROCESS_H_
#define YOLOV3_POSTPROCESS_H_

#include <vector>
#include <string>
#include <utility>
#include <memory>
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"
#include "xstream/xstream_world.h"
#include "xstream/vision_type.h"
#include "hobotlog/hobotlog.hpp"

namespace inference {

struct Detection {
  xstream::BBox bbox;
  Detection() {}
  ~Detection() {}

  explicit Detection(xstream::BBox bbox) : bbox(bbox) {}

  friend bool operator>(const Detection &lhs, const Detection &rhs) {
    return (lhs.bbox.score_ > rhs.bbox.score_);
  }

  friend std::ostream &operator<<(std::ostream &os, const Detection &det) {
    os << "{"
       << R"("bbox")"
       << ":" << "x1: " << det.bbox.x1_ << " y1: " << det.bbox.y1_
       << " x2: " << det.bbox.x2_ << " y2: " << det.bbox.y2_
       << " score: " << det.bbox.score_ << ", "
       << R"("class_name")"
       << ":" << det.bbox.specific_type_ << ","
       << R"("id")"
       << ":" << det.bbox.id_ << "}";
    return os;
  }
};

class YoloV3PostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 500;

  // 用于坐标映射，需要配置
  // 且与预测阶段选取金字塔基础层大小一致
  int basic_pyramid_image_height_;
  int basic_pyramid_image_width_;

  int src_image_height_;  // 金字塔原图大小
  int src_image_width_;

  void NMS(std::vector<Detection> &input, float iou_threshold,
           int top_k, std::vector<Detection> &result, bool suppress);
};

struct Yolo3Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;

  std::string stridesToString() {
    std::stringstream ss;
    for (const auto &stride : strides) {
      ss << stride << " ";
    }
    return ss.str();
  }

  std::string anchorsTableToString() {
    std::stringstream ss;
    for (const auto &anchors : anchors_table) {
      for (auto data : anchors) {
        ss << "[" << data.first << "," << data.second << "] ";
      }
    }
    return ss.str();
  }
};

}  // namespace inference

#endif  // YOLOV3_POSTPROCESS_H_
