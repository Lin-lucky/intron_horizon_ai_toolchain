/**
 * Copyright (c) 2021 Horizon Robotics. All rights reserved.
 * @brief  definition of yolov3_postprocess
 * @file   yolov3_postprocess.cc
 * @author zhe.sun
 * @email  zhe.sun@horizon.ai
 * @version   0.0.0.1
 * @date      2021.04.19
 */

#include "model_inference/postprocess/matrix_detection_postprocess.h"

#include <algorithm>

#include "json/json.h"
#include "model_inference/inference_engine.h"

namespace inference {
static std::vector<std::string> detection_class_names = {
    "FullCar",         "CarTail",      "Pedestrian",  "Cyclist",
    "TrafficLightBox", "TrafficLight", "TrafficCard", "ConeBucket"};
static std::vector<std::string> semantic_class_names = {
    "Road",     "BackGround", "Fence",    "Person",    "Vehicle",
    "TwoWheel", "CrossWalk",  "SignLine", "GuideLine", "Cone",
    "StopLine", "SpeedBump",  "Pole"};
static std::vector<std::string> lane_class_names_ = {"BackGround", "lane",
                                                     "curb", "double_line"};

int MatrixDetectionPostProcess::Init(const std::string &json_str) {
  // string转json
  Json::Reader Reader;
  Json::Value config;
  Reader.parse(json_str, config);

  score_threshold_ = config["score_threshold"].isNumeric()
                         ? config["score_threshold"].asFloat()
                         : score_threshold_;
  nms_threshold_ = config["nms_threshold"].isNumeric()
                       ? config["nms_threshold"].asFloat()
                       : nms_threshold_;
  nms_top_k_ = config["nms_top_k"].isNumeric() ? config["nms_top_k"].asInt()
                                               : nms_top_k_;
  min_box_height_ = config["min_box_height"].isNumeric()
                        ? config["min_box_height"].asFloat()
                        : min_box_height_;
  min_box_width_ = config["min_box_width"].isNumeric()
                       ? config["min_box_width"].asFloat()
                       : min_box_width_;

  if (config["basic_pyramid_image_height"].isNumeric() &&
      config["basic_pyramid_image_width"].isNumeric()) {
    basic_pyramid_image_height_ = config["basic_pyramid_image_height"].asInt();
    basic_pyramid_image_width_ = config["basic_pyramid_image_width"].asInt();
  } else {
    LOGE << "please set basic_pyramid_image_size";
    return -1;
  }
  if (config["src_image_height"].isNumeric() &&
      config["src_image_width"].isNumeric()) {
    src_image_height_ = config["src_image_height"].asInt();
    src_image_width_ = config["src_image_width"].asInt();
  } else {
    LOGE << "please set src_image_size";
    return -1;
  }

  return 0;
}

float MatrixDetectionPostProcess::QuantiShift(int32_t data, uint32_t shift) {
  return static_cast<float>(data) / static_cast<float>(1 << shift);
}

bool MatrixDetectionPostProcess::IsQuanti(TensorProperties *property) {
  if (property->shift.shift_data) {
    return true;
  }
  return false;
}

int MatrixDetectionPostProcess::ParseBox(
    const std::shared_ptr<InferenceEngineTask> task,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "MatrixDetectionPostProcess PraseBox";

  auto xstream_det_result = std::make_shared<xstream::BaseDataVector>();
  frame_result->push_back(xstream_det_result);

  double h_ratio = basic_pyramid_image_height_ * 1.0 / src_image_height_;
  double w_ratio = basic_pyramid_image_width_ * 1.0 / src_image_width_;
  LOGD << "ratio = " << h_ratio << ", " << w_ratio;
  // 取Tensor
  std::vector<MatrixDetection> dets;
  int out_layer = task->output_tensors_.size();
  for (int index = 0; index < 8; ++index) {
    auto &tensor = task->output_tensors_[index];
    int shape_h, shape_w, shape_c, aligned_shape_c;
    switch (tensor.properties.tensor_layout) {
      case LAYOUT_NHWC: {
        shape_h = tensor.properties.valid_shape.dimension_size[1];
        shape_w = tensor.properties.valid_shape.dimension_size[2];
        shape_c = tensor.properties.valid_shape.dimension_size[3];
        aligned_shape_c = tensor.properties.aligned_shape.dimension_size[3];
        break;
      }
      case LAYOUT_NCHW: {
        shape_h = tensor.properties.valid_shape.dimension_size[2];
        shape_w = tensor.properties.valid_shape.dimension_size[3];
        shape_c = tensor.properties.valid_shape.dimension_size[1];
        aligned_shape_c = tensor.properties.aligned_shape.dimension_size[1];
        break;
      }
      case LAYOUT_NHWC_4W8C: {
        shape_h = tensor.properties.valid_shape.dimension_size[1];
        shape_w = tensor.properties.valid_shape.dimension_size[2];
        shape_c = tensor.properties.valid_shape.dimension_size[3];
        aligned_shape_c = tensor.properties.aligned_shape.dimension_size[3];
        break;
      }
      default:
        HOBOT_CHECK(0) << "not support output layout";
    }
    LOGD << "index = " << index
         << ", shift len = " << tensor.properties.shift.shift_len
         << tensor.properties.quanti_type;
    int valid_boxes = 0;
    for (auto i = 0; i < shape_w; i++) {
      float data[shape_c] = {0};
      int base_index = i * aligned_shape_c;
      for (auto j = 0; j < shape_c; j++) {
        float tmp;
        if (IsQuanti(&tensor.properties)) {
          tmp = QuantiShift(reinterpret_cast<int16_t *>(
                                tensor.sys_mem[0].vir_addr)[base_index + j],
                            tensor.properties.shift.shift_data[j]);
        } else {
          LOGE << "error.";
          tmp = reinterpret_cast<float *>(
              tensor.sys_mem[0].vir_addr)[base_index + j];
        }
        data[j] = tmp;
      }
      /*
      if (i < 20) {
        std::cout << "data = " << data[0] << ", " << data[1] << ", " << data[2]
                  << ", " << data[3] << ", " << data[4] << std::endl;
      }
      */
      if (i == 0) {
        valid_boxes = data[0] / 4;
        LOGD << "valid box count = " << valid_boxes;
        continue;
      }
      if (i > valid_boxes) {
        break;
      }
      if (data[4] > 0) {
        MatrixDetection det;
        float tmp = 0.0;
        tmp = data[0] / w_ratio;  // x1
        det.bbox.x1_ = tmp < 0.0                ? 0.0
                       : tmp > src_image_width_ ? src_image_width_
                                                : tmp;
        tmp = data[1] / h_ratio;  // y1
        det.bbox.y1_ = tmp < 0.0                 ? 0.0
                       : tmp > src_image_height_ ? src_image_height_
                                                 : tmp;
        tmp = data[2] / w_ratio;  // x2
        det.bbox.x2_ = tmp < 0.0                ? 0.0
                       : tmp > src_image_width_ ? src_image_width_
                                                : tmp;
        tmp = data[3] / h_ratio;  // y2
        det.bbox.y2_ = tmp < 0.0                 ? 0.0
                       : tmp > src_image_height_ ? src_image_height_
                                                 : tmp;
        det.bbox.score_ = data[4];
        det.bbox.id_ = index;
        LOGD << "box info: " << det.bbox;
        dets.push_back(det);
      }
    }
  }
  std::vector<MatrixDetection> det_result;
  NMS(dets, nms_threshold_, nms_top_k_, det_result, false);

  // Detection转换BaseData
  for (size_t i = 0; i < det_result.size(); ++i) {
    auto &box = det_result[i].bbox;
    LOGD << box << ", id: " << box.id_
         << ", category_name: " << box.specific_type_;
    auto xstream_box = std::make_shared<xstream::BBox>(box);
    xstream_det_result->datas_.push_back(xstream_box);
  }
  return 0;
}

int MatrixDetectionPostProcess::ParseSeg(
    const std::shared_ptr<InferenceEngineTask> task,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "MatrixDetectionPostProcess ParseSeg";

  // 取Tensor
  int out_layer = task->output_tensors_.size();
  for (int index = 8; index < out_layer; ++index) {
    auto xstream_seg_result = std::make_shared<xstream::BaseDataVector>();
    frame_result->push_back(xstream_seg_result);
    auto seg = std::make_shared<xstream::Segmentation>();
    xstream_seg_result->datas_.push_back(seg);

    auto &tensor = task->output_tensors_[index];
    int shape_h, shape_w, shape_c;
    switch (tensor.properties.tensor_layout) {
      case LAYOUT_NHWC: {
        shape_h = tensor.properties.valid_shape.dimension_size[1];
        shape_w = tensor.properties.valid_shape.dimension_size[2];
        shape_c = tensor.properties.valid_shape.dimension_size[3];
        break;
      }
      case LAYOUT_NCHW: {
        shape_h = tensor.properties.valid_shape.dimension_size[2];
        shape_w = tensor.properties.valid_shape.dimension_size[3];
        shape_c = tensor.properties.valid_shape.dimension_size[1];
        break;
      }
      case LAYOUT_NHWC_4W8C: {
        shape_h = tensor.properties.valid_shape.dimension_size[1];
        shape_w = tensor.properties.valid_shape.dimension_size[2];
        shape_c = tensor.properties.valid_shape.dimension_size[3];
        break;
      }
      default:
        HOBOT_CHECK(0) << "not support output layout";
    }
    seg->width_ = shape_w;
    seg->height_ = shape_h;
    auto &one_data = seg->values_;
    LOGD << "width = " << shape_w << ", " << shape_h;
    int8_t *data = reinterpret_cast<int8_t *>(tensor.sys_mem[0].vir_addr);
    for (auto i = 0; i < shape_h; i++) {
      for (auto j = 0; j < shape_w; j++) {
        int8_t elem = data[i * shape_w + j];
        one_data.push_back(elem);
      }
    }
  }
  return 0;
}

int MatrixDetectionPostProcess::Execute(
    const std::vector<std::shared_ptr<InferenceEngineTask>> &tasks,
    std::vector<xstream::BaseDataPtr> *frame_result) {
  LOGD << "MatrixDetectionPostProcess Execute";
  HOBOT_CHECK(tasks.size() == 1);
  auto task = tasks[0];
  ParseBox(task, frame_result);
  ParseSeg(task, frame_result);
  // 需要自行释放outputtensors
  InferenceEngine::GetInstance()->FreeTensor(task->output_tensors_);
  return 0;
}

void MatrixDetectionPostProcess::NMS(std::vector<MatrixDetection> &input,
                                     float iou_threshold, int top_k,
                                     std::vector<MatrixDetection> &result,
                                     bool suppress) {
  // sort order by score desc
  std::stable_sort(input.begin(), input.end(), std::greater<MatrixDetection>());
  if (input.size() > 400) {
    input.resize(400);
  }

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = input[i].bbox.Width();
    float height = input[i].bbox.Height();
    areas.push_back(width * height);
    if (width <= min_box_width_ || height <= min_box_height_) {
      skip[i] = true;
    }
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].bbox.id_ != input[j].bbox.id_) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].bbox.x1_, input[j].bbox.x1_);
      float yy1 = std::max(input[i].bbox.y1_, input[j].bbox.y1_);
      float xx2 = std::min(input[i].bbox.x2_, input[j].bbox.x2_);
      float yy2 = std::min(input[i].bbox.y2_, input[j].bbox.y2_);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }
    result.push_back(input[i]);
  }
}

}  // namespace inference
