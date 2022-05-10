// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/qat_yolo3_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(QATYolo3PostProcessMethod);

QATYolo3Config default_qat_yolo3_config = {
    {8, 16, 32},
    {{{116, 90}, {156, 198}, {373, 326}},
     {{30, 61}, {62, 45}, {59, 119}},
     {{10, 13}, {16, 30}, {33, 23}}},
    20,
    {"aeroplane",   "bicycle", "bird",  "boat",      "bottle",
     "bus",         "car",     "cat",   "chair",     "cow",
     "diningtable", "dog",     "horse", "motorbike", "person",
     "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"}};

int QATYolo3PostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "QATYolo3PostProcessMethod init from file";
  std::ifstream ifs(config_file_path.c_str());
  if (!ifs) {
    VLOG(EXAMPLE_SYSTEM) << "Open config file " << config_file_path
                         << " failed";
    return -1;
  }

  std::stringstream buffer;
  buffer << ifs.rdbuf();
  std::string contents(buffer.str());
  return this->InitFromJsonString(contents);
}

int QATYolo3PostProcessMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "QATYolo3PostProcessMethod Json string:"
                      << config.data();
  rapidjson::Document document;
  document.Parse(config.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }
  if (document.HasMember("score_threshold")) {
    score_threshold_ = document["score_threshold"].GetFloat();
  }

  if (document.HasMember("nms_threshold")) {
    nms_threshold_ = document["nms_threshold"].GetFloat();
  }

  if (document.HasMember("nms_top_k")) {
    nms_top_k_ = document["nms_top_k"].GetInt();
  }

  if (document.HasMember("strides")) {
    rapidjson::Value &stride_value = document["strides"];
    auto strides_array = stride_value.GetArray();
    for (int i = 0; i < strides_array.Size(); ++i) {
      yolo3_config_.strides[i] = strides_array[i].GetInt();
    }
  }

  if (document.HasMember("anchors_table")) {
    // Anchors table
    rapidjson::Value &anchors_value = document["anchors_table"];
    auto anchors_array = anchors_value.GetArray();
    VLOG_IF(EXAMPLE_SYSTEM,
            anchors_array.Size() != yolo3_config_.strides.size())
        << "Feature num should be equal to output num, feature num is "
        << anchors_array.Size() << ", output num is "
        << yolo3_config_.strides.size();

    yolo3_config_.anchors_table.reserve(anchors_array.Size());
    for (int i = 0; i < anchors_array.Size(); ++i) {
      auto anchor_pairs = anchors_array[i].GetArray();
      auto &anchors = yolo3_config_.anchors_table[i];
      anchors.reserve(anchor_pairs.Size());
      for (int j = 0; j < anchor_pairs.Size(); ++j) {
        auto anchor = anchor_pairs[j].GetArray();
        anchors[j] =
            std::make_pair(anchor[0].GetDouble(), anchor[1].GetDouble());
      }
    }
  }

  if (document.HasMember("class_num")) {
    yolo3_config_.class_num = document["class_num"].GetInt();
  }

  VLOG(EXAMPLE_DEBUG) << "score_threshold: " << score_threshold_
                      << ", nms_nms_threshold:" << nms_threshold_
                      << ", nms_topk: " << nms_top_k_ << ", "
                      << yolo3_config_.Str();
  return 0;
}

std::vector<xstream::BaseDataPtr> QATYolo3PostProcessMethod::DoProcess(
    const std::vector<xstream::BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  auto image_tensor = std::static_pointer_cast<ImageTensor>(input[0]);
  auto output_tensor = std::static_pointer_cast<TensorVector>(input[1]);
  auto perception = std::shared_ptr<Perception>(new Perception);

  //  PostProcess
  this->PostProcess(
      output_tensor->tensors, image_tensor.get(), perception.get());
  VLOG(EXAMPLE_DETAIL) << "PostProcess success!";
  release_output_tensor(output_tensor->tensors);
  VLOG(EXAMPLE_DETAIL) << "release output tensor success!";

  std::vector<xstream::BaseDataPtr> res;
  perception->name_ = output_name_;
  res.emplace_back(xstream::BaseDataPtr(perception));
  VLOG(EXAMPLE_DETAIL)
      << "QATYolo3PostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void QATYolo3PostProcessMethod::Finalize() {}

int QATYolo3PostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in QATYolo3PostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr QATYolo3PostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "QATYolo3PostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string QATYolo3PostProcessMethod::GetVersion() const {}

xstream::MethodInfo QATYolo3PostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "QATYolo3PostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

QATYolo3PostProcessMethod::~QATYolo3PostProcessMethod() {}

int QATYolo3PostProcessMethod::PostProcess(std::vector<hbDNNTensor> &tensors,
                                           ImageTensor *image_tensor,
                                           Perception *perception) {
  YoloAux(tensors.data(), image_tensor, perception);

  int32_t padding_height = 0;
  int32_t padding_width = 0;
  float h_base =
      (float)image_tensor->ori_height() / (float)image_tensor->height();
  float w_base =
      (float)image_tensor->ori_width() / (float)image_tensor->width();
  perception->type = Perception::DET;
  int box_index = 0;
  for (int i = 0; i < perception->det.size(); i++) {
    if (perception->det[i].score < score_threshold_) {
      continue;
    }
    perception->det[i].bbox.xmin =
        perception->det[i].bbox.xmin * w_base + padding_width;
    perception->det[i].bbox.ymin =
        perception->det[i].bbox.ymin * h_base + padding_height;
    perception->det[i].bbox.xmax =
        perception->det[i].bbox.xmax * w_base + padding_width;
    perception->det[i].bbox.ymax =
        perception->det[i].bbox.ymax * h_base + padding_height;
    perception->det[i].id = perception->det[i].id;
    perception->det[i].score = perception->det[i].score;
    if (perception->det[i].id >= yolo3_config_.class_names.size()) {
      perception->det[i].class_name = "unkown";
    } else {
      perception->det[i].class_name =
          yolo3_config_.class_names[perception->det[i].id].c_str();
    }
    box_index++;
  }
  perception->det.resize(box_index);
  return 0;
}

inline float fastExp(float x) {
  union {
    uint32_t i;
    float f;
  } v;
  v.i = (12102203.1616540672f * x + 1064807160.56887296f);
  return v.f;
}

#define BSWAP_32(x) static_cast<int32_t>(__builtin_bswap32(x))

#define r_int32(x, big_endian) \
  (big_endian) ? BSWAP_32((x)) : static_cast<int32_t>((x))

float QATYolo3PostProcessMethod::Dequanti(int32_t data,
                                          int layer,
                                          bool big_endian,
                                          int offset,
                                          hbDNNTensorProperties &properties) {
  hbDNNQuantiType quanti_type = properties.quantiType;
  uint8_t *shift_value = properties.shift.shiftData;
  float *scale_value = properties.scale.scaleData;
  if (quanti_type == SHIFT) {
    return static_cast<float>(r_int32(data, big_endian)) /
           static_cast<float>(1 << shift_value[offset]);
  } else if (quanti_type == SCALE) {
    return static_cast<float>(r_int32(data, big_endian)) * scale_value[offset];
  }
  VLOG(EXAMPLE_SYSTEM) << "quanti type error!";
  return 0.0;
}

int QATYolo3PostProcessMethod::YoloAux(hbDNNTensor *tensor,
                                       ImageTensor *image_tensor,
                                       Perception *perception) {
  std::vector<Detection> dets;
  for (int i = 0; i < yolo3_config_.strides.size(); i++) {
    perception->type = Perception::DET;
    PostProcess(&tensor[i], image_tensor, i, dets);
  }
  nms(dets, nms_threshold_, nms_top_k_, perception->det, false);
  return 0;
}

void QATYolo3PostProcessMethod::PostProcess(hbDNNTensor *tensor,
                                            ImageTensor *frame,
                                            int layer,
                                            std::vector<Detection> &dets) {
  int num_classes = yolo3_config_.class_num;
  int num_pred = num_classes + 4 + 1;
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  void *raw_data = tensor->sysMem[0].virAddr;
  int num_anchors = yolo3_config_.anchors_table[0].size();
  std::vector<std::pair<double, double>> &anchors =
      yolo3_config_.anchors_table[layer];
  auto properties = tensor->properties;
  hbDNNTensorShape aligned_shape = tensor->properties.alignedShape;
  hbDNNTensorShape valid_shape = tensor->properties.validShape;
  uint32_t aligned_h = aligned_shape.dimensionSize[2];
  uint32_t aligned_w = aligned_shape.dimensionSize[3];
  uint32_t valid_h = valid_shape.dimensionSize[2];
  uint32_t valid_w = valid_shape.dimensionSize[3];
  int elem_type = tensor->properties.tensorType;
  int elem_size = (elem_type > HB_DNN_TENSOR_TYPE_S8) ? 4 : 1;
  std::vector<float> class_pred(num_classes, 0.0);
  // TODO(ruxin.song): big_endian = true
  bool big_endian = false;

  // tensor type: NCHW
  for (uint32_t h = 0; h < aligned_h; h++) {
    if (h >= valid_h) continue;
    for (uint32_t w = 0; w < aligned_w; w++) {
      if (w >= valid_w) continue;
      for (uint32_t k = 0; k < num_anchors; k++) {
        float anchor_x = anchors[k].first;
        float anchor_y = anchors[k].second;
        uint32_t cur_c = k * num_pred;
        if (elem_type == HB_DNN_TENSOR_TYPE_S32) {
          int offset =
              aligned_shape.dimensionSize[2] * aligned_shape.dimensionSize[3];
          // get objness
          int32_t *objness_data = reinterpret_cast<int32_t *>(
              reinterpret_cast<int8_t *>(raw_data) +
              (offset * (num_pred * k + 4)) * elem_size);
          float objness = Dequanti(objness_data[h * aligned_w + w],
                                   layer,
                                   big_endian,
                                   cur_c + 4,
                                   properties);
          if (objness < 0 && is_performance_ == true) {
            continue;
          }
          float confidence;
          if (!is_performance_) {
            confidence = 1 / (1 + std::exp(-objness));
          } else {
            confidence = 1 / (1 + fastExp(-objness));
          }
          if (confidence - score_threshold_ < 0.0000001) {
            continue;
          }
          // get class score
          for (int index = 0; index < num_classes; index++) {
            int32_t *class_pred_data = reinterpret_cast<int32_t *>(
                reinterpret_cast<int8_t *>(raw_data) +
                (offset * (k * num_pred + 5 + index)) * elem_size);
            class_pred[index] = Dequanti(class_pred_data[h * aligned_w + w],
                                         layer,
                                         big_endian,
                                         cur_c + 5 + index,
                                         properties);
          }
          std::vector<float>::iterator biggest =
              max_element(begin(class_pred), end(class_pred));
          float id = distance(begin(class_pred), biggest);
          float class_threshold =
              -1 * std::log((confidence - score_threshold_) / score_threshold_);
          if (class_pred[id] > class_threshold) {
            // get box
            int32_t *box_cx_data = reinterpret_cast<int32_t *>(
                reinterpret_cast<int8_t *>(raw_data) +
                (offset * (k * num_pred + 0)) * elem_size);
            int32_t *box_cy_data = reinterpret_cast<int32_t *>(
                reinterpret_cast<int8_t *>(raw_data) +
                (offset * (k * num_pred + 1)) * elem_size);
            int32_t *box_sx_data = reinterpret_cast<int32_t *>(
                reinterpret_cast<int8_t *>(raw_data) +
                (offset * (k * num_pred + 2)) * elem_size);
            int32_t *box_sy_data = reinterpret_cast<int32_t *>(
                reinterpret_cast<int8_t *>(raw_data) +
                (offset * (k * num_pred + 3)) * elem_size);

            float center_x = Dequanti(box_cx_data[h * aligned_w + w],
                                      layer,
                                      big_endian,
                                      cur_c,
                                      properties);
            float center_y = Dequanti(box_cy_data[h * aligned_w + w],
                                      layer,
                                      big_endian,
                                      cur_c + 1,
                                      properties);
            float scale_x = Dequanti(box_sx_data[h * aligned_w + w],
                                     layer,
                                     big_endian,
                                     cur_c + 2,
                                     properties);
            float scale_y = Dequanti(box_sy_data[h * aligned_w + w],
                                     layer,
                                     big_endian,
                                     cur_c + 3,
                                     properties);
            int stride = static_cast<int>(yolo3_config_.strides[layer]);

            // parse box
            float box_center_x, box_center_y, box_scale_x, box_scale_y;
            if (!is_performance_) {
              box_center_x = ((1.0 / (1.0 + std::exp(-center_x))) + w) * stride;
              box_center_y = ((1.0 / (1.0 + std::exp(-center_y))) + h) * stride;
              box_scale_x = std::exp(scale_x) * anchor_x;
              box_scale_y = std::exp(scale_y) * anchor_y;
            } else {
              box_center_x = ((1 / (1 + fastExp(-scale_x))) + w) * stride;
              box_center_y = ((1 / (1 + fastExp(-scale_y))) + h) * stride;
              box_scale_x = fastExp(scale_x) * anchor_x;
              box_scale_y = fastExp(scale_y) * anchor_y;
            }
            float xmin = box_center_x - box_scale_x / 2.0;
            float ymin = box_center_y - box_scale_y / 2.0;
            float xmax = box_center_x + box_scale_x / 2.0;
            float ymax = box_center_y + box_scale_y / 2.0;
            Bbox bbox(xmin, ymin, xmax, ymax);
            dets.push_back(Detection(
                id, 1 / (1 + std::exp(-class_pred[id])) * confidence, bbox));
          }
        } else {
          // TODO(ruxin.song): elem type not S32
        }
      }
    }
  }
}
