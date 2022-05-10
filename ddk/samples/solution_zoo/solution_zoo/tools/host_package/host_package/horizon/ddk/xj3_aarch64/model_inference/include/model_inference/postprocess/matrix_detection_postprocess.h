

/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of matrix_detection_postprocess
 * @file   matrix_detection_postprocess.h
 * @author xudong.du
 * @email  xudong.du@horizon.ai
 * @version   0.0.0.1
 * @date      2021.09.08
 */
#ifndef MATRIX_DETECTION_POSTPROCESS_H_
#define MATRIX_DETECTION_POSTPROCESS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "model_inference/inference_task.h"
#include "model_inference/postprocess/postprocess.h"
#include "xstream/vision_type.h"
#include "xstream/xstream_world.h"

namespace inference {

struct MatrixDetection {
  xstream::BBox bbox;
  MatrixDetection() {}
  ~MatrixDetection() {}

  explicit MatrixDetection(xstream::BBox bbox) : bbox(bbox) {}

  friend bool operator>(const MatrixDetection &lhs,
                        const MatrixDetection &rhs) {
    return (lhs.bbox.score_ > rhs.bbox.score_);
  }

  friend std::ostream &operator<<(std::ostream &os,
                                  const MatrixDetection &det) {
    os << "{"
       << R"("bbox")"
       << ":"
       << "x1: " << det.bbox.x1_ << " y1: " << det.bbox.y1_
       << " x2: " << det.bbox.x2_ << " y2: " << det.bbox.y2_
       << " score: " << det.bbox.score_ << ", "
       << R"("class_name")"
       << ":" << det.bbox.specific_type_ << ","
       << R"("id")"
       << ":" << det.bbox.id_ << "}";
    return os;
  }
};

class MatrixDetectionPostProcess : public PostProcess {
 public:
  virtual int Init(const std::string &json_str);

  virtual int Execute(
      const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
      std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  /**
   * check if the data needs to be transformed
   * @param[in] property: tensor property
   * @return ture if success otherwise false
   */
  inline bool IsQuanti(TensorProperties *property);
  /**
   * Transform the model output data
   * @param[in] data: data to transform
   * @param[in] shift: shift info
   * @return transformed data
   */
  inline float QuantiShift(int32_t data, uint32_t shift);
  int ParseBox(const std::shared_ptr<InferenceEngineTask> task,
               std::vector<xstream::BaseDataPtr> *frame_result);
  int ParseSeg(const std::shared_ptr<InferenceEngineTask> task,
               std::vector<xstream::BaseDataPtr> *frame_result);

 private:
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 500;
  float min_box_width_ = 2.0f;
  float min_box_height_ = 2.0f;

  // used for coordinate mapping
  // equeal to model input size
  int basic_pyramid_image_height_;
  int basic_pyramid_image_width_;

  // original size of pyramid
  int src_image_height_;
  int src_image_width_;

  void NMS(std::vector<MatrixDetection> &input, float iou_threshold, int top_k,
           std::vector<MatrixDetection> &result, bool suppress);
};

}  // namespace inference

#endif  // MATRIX_DETECTION_POSTPROCESS_H_
