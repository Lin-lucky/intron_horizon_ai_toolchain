// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/qat_retinanet_post_process_method.h"

#include <iostream>
#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(QATRetinanetPostProcessMethod);

QATRetinanetConfig default_retinanet_config = {
    {{32, 40.31747359663594, 50.79683366298238},
     {64, 80.63494719327188, 101.59366732596476},
     {128, 161.26989438654377, 203.18733465192952},
     {256, 322.53978877308754, 406.37466930385904},
     {512, 645.0795775461751, 812.7493386077181}},
    {{0.5, 1, 2}, {0.5, 1, 2}, {0.5, 1, 2}, {0.5, 1, 2}, {0.5, 1, 2}},
    80,
    {"person",        "bicycle",      "car",
     "motorcycle",    "airplane",     "bus",
     "train",         "truck",        "boat",
     "traffic light", "fire hydrant", "stop sign",
     "parking meter", "bench",        "bird",
     "cat",           "dog",          "horse",
     "sheep",         "cow",          "elephant",
     "bear",          "zebra",        "giraffe",
     "backpack",      "umbrella",     "handbag",
     "tie",           "suitcase",     "frisbee",
     "skis",          "snowboard",    "sports ball",
     "kite",          "baseball bat", "baseball glove",
     "skateboard",    "surfboard",    "tennis racket",
     "bottle",        "wine glass",   "cup",
     "fork",          "knife",        "spoon",
     "bowl",          "banana",       "apple",
     "sandwich",      "orange",       "broccoli",
     "carrot",        "hot dog",      "pizza",
     "donut",         "cake",         "chair",
     "couch",         "potted plant", "bed",
     "dining table",  "toilet",       "tv",
     "laptop",        "mouse",        "remote",
     "keyboard",      "cell phone",   "microwave",
     "oven",          "toaster",      "sink",
     "refrigerator",  "book",         "clock",
     "vase",          "scissors",     "teddy bear",
     "hair drier",    "toothbrush"}};

#define BSWAP_32(x) static_cast<int32_t>(__builtin_bswap32(x))

#define r_int32(x, big_endian) \
  (big_endian) ? BSWAP_32((x)) : static_cast<int32_t>((x))

inline int round_float(float number) {
  return (number > 0.0) ? floor(number + 0.5) : ceil(number - 0.5);
}

int QATRetinanetPostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "QATRetinanetPostProcessMethod init from file";
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

int QATRetinanetPostProcessMethod::InitFromJsonString(
    const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "QATRetinanetPostProcessMethod Json string:"
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

  int layer_num = retinanet_config_.anchor_ratios.size();
  retinanet_config_.base_anchors.resize(layer_num);
  for (int i = 0; i < layer_num; i++) {
    RetinanetAnchors(i);
  }

  return 0;
}

std::vector<xstream::BaseDataPtr> QATRetinanetPostProcessMethod::DoProcess(
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
      << "QATRetinanetPostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void QATRetinanetPostProcessMethod::Finalize() {}

int QATRetinanetPostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in QATRetinanetPostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr QATRetinanetPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "QATRetinanetPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string QATRetinanetPostProcessMethod::GetVersion() const {}

xstream::MethodInfo QATRetinanetPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "QATRetinanetPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = false;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

QATRetinanetPostProcessMethod::~QATRetinanetPostProcessMethod() = default;

int QATRetinanetPostProcessMethod::PostProcess(
    std::vector<hbDNNTensor> &tensors,
    ImageTensor *image_tensor,
    Perception *perception) {
  perception->type = Perception::DET;
  int layer_num = retinanet_config_.anchor_ratios.size();

  std::vector<Detection> total_dets;
  for (int i = 0; i < layer_num; i++) {
    std::vector<ClassScore> cls_scores;
    std::vector<Detection> dets;
    SoftmaxFromRawScore(
        &tensors[i], default_retinanet_config.class_num, i, cls_scores);
    std::vector<Bbox> bboxes;
    GetBboxFromRawData(
        &tensors[i + layer_num], cls_scores, i, dets, image_tensor);
    std::stable_sort(dets.begin(), dets.end(), std::greater<Detection>());
    if (dets.size() > 1000) dets.resize(1000);
    total_dets.insert(total_dets.end(), dets.begin(), dets.end());
  }
  yolo5_nms(total_dets, nms_threshold_, nms_top_k_, perception->det, false);
  return 0;
}

int QATRetinanetPostProcessMethod::GetBboxFromRawData(
    hbDNNTensor *tensor,
    std::vector<ClassScore> &scores,
    int layer,
    std::vector<Detection> &dets,
    ImageTensor *frame) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *raw_data = reinterpret_cast<int8_t *>(tensor->sysMem[0].virAddr);
  int32_t *valid_shape = tensor->properties.validShape.dimensionSize;
  int32_t *aligned_shape = tensor->properties.alignedShape.dimensionSize;
  int h_idx, w_idx, c_idx;
  bool big_endian = false;
  get_tensor_hwc_index(tensor, &h_idx, &w_idx, &c_idx);
  int stride_hw = aligned_shape[h_idx] * aligned_shape[w_idx];
  Bbox bbox;

  for (uint32_t i = 0; i < scores.size(); i++) {
    uint32_t hh = scores[i].h;
    uint32_t ww = scores[i].w;
    uint32_t anchor_id = scores[i].anchor;

    int offset = anchor_id * 4;
    float box_x = Dequanti(
        (raw_data + (stride_hw * offset))[hh * aligned_shape[w_idx] + ww],
        layer,
        big_endian,
        offset,
        tensor->properties);
    float box_y = Dequanti(
        (raw_data + (stride_hw * (offset + 1)))[hh * aligned_shape[w_idx] + ww],
        layer,
        big_endian,
        offset + 1,
        tensor->properties);
    float box_w = Dequanti(
        (raw_data + (stride_hw * (offset + 2)))[hh * aligned_shape[w_idx] + ww],
        layer,
        big_endian,
        offset + 2,
        tensor->properties);
    float box_h = Dequanti(
        (raw_data + (stride_hw * (offset + 3)))[hh * aligned_shape[w_idx] + ww],
        layer,
        big_endian,
        offset + 3,
        tensor->properties);
    float stride_width = image_width_ / valid_shape[w_idx];
    float stride_height = image_height_ / valid_shape[h_idx];
    auto base_anchors = retinanet_config_.base_anchors[layer][anchor_id];
    int anchors_x = ww * stride_width + base_anchors[0];
    int anchors_y = hh * stride_height + base_anchors[1];
    int anchors_w = ww * stride_width + base_anchors[2];
    int anchors_h = hh * stride_height + base_anchors[3];

    float widths = anchors_w - anchors_x;
    float heights = anchors_h - anchors_y;
    float ctr_x = anchors_x + widths / 2.0;
    float ctr_y = anchors_y + heights / 2.0;

    float pred_ctr_x = box_x * widths + ctr_x;
    float pred_ctr_y = box_y * heights + ctr_y;
    float pred_w = std::exp(box_w) * widths;
    float pred_h = std::exp(box_h) * heights;

    bbox.xmin = pred_ctr_x - 0.5 * pred_w;
    bbox.ymin = pred_ctr_y - 0.5 * pred_h;
    bbox.xmax = pred_ctr_x + 0.5 * pred_w;
    bbox.ymax = pred_ctr_y + 0.5 * pred_h;

    if (bbox.xmin < 0) {
      bbox.xmin = 0.0;
    }
    if (bbox.ymin < 0) {
      bbox.ymin = 0.0;
    }
    if (bbox.xmax > image_width_) {
      bbox.xmax = image_width_;
    }
    if (bbox.ymax > image_height_) {
      bbox.ymax = image_height_;
    }

    for (uint8_t id = 0; id < retinanet_config_.class_num; id++) {
      if (scores[i].scores[id] < score_threshold_) continue;
      dets.push_back(Detection(id,
                               scores[i].scores[id],
                               bbox,
                               retinanet_config_.class_names[id].c_str()));
    }
  }
  return 0;
}

int QATRetinanetPostProcessMethod::SoftmaxFromRawScore(
    hbDNNTensor *tensor,
    int class_num,
    int layer,
    std::vector<ClassScore> &scores) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *raw_data = reinterpret_cast<int8_t *>(tensor->sysMem[0].virAddr);
  int32_t *valid_shape = tensor->properties.validShape.dimensionSize;
  int32_t *aligned_shape = tensor->properties.alignedShape.dimensionSize;
  int32_t batch_size = aligned_shape[0];
  int h_idx, w_idx, c_idx;
  bool big_endian = false;
  get_tensor_hwc_index(tensor, &h_idx, &w_idx, &c_idx);
  int32_t h_size = aligned_shape[h_idx];
  int32_t w_size = aligned_shape[w_idx];
  int32_t c_size = aligned_shape[c_idx];
  int stride_hw = aligned_shape[h_idx] * aligned_shape[w_idx];

  ClassScore cls_score;
  uint32_t anchor_num = c_size / class_num;

  for (uint32_t hh = 0; hh < batch_size * h_size; hh++) {
    if (hh >= valid_shape[h_idx]) continue;
    for (uint32_t ww = 0; ww < w_size; ww++) {
      if (ww >= valid_shape[w_idx]) continue;
      for (uint32_t anchor_id = 0; anchor_id < anchor_num; anchor_id++) {
        int offset = anchor_id * class_num;
        cls_score.h = hh;
        cls_score.w = ww;
        cls_score.anchor = anchor_id;
        for (int cls = 0; cls < class_num; ++cls) {
          int8_t class_pred_data =
              (raw_data +
               (stride_hw * (offset + cls)))[hh * aligned_shape[w_idx] + ww];
          double cls_value = Dequanti(class_pred_data,
                                      layer,
                                      big_endian,
                                      offset + cls,
                                      tensor->properties);
          cls_score.scores[cls] = 1.0 / (1.0 + std::exp(-cls_value));
        }
        auto max_id =
            argmax(&cls_score.scores[0], &cls_score.scores[class_num]);
        if (cls_score.scores[max_id] < score_threshold_) continue;
        scores.push_back(cls_score);
      }
    }
  }
  return 0;
}

int QATRetinanetPostProcessMethod::RetinanetAnchors(int layer) {
  auto anchor_ratios = retinanet_config_.anchor_ratios[layer];
  int ratios_size = retinanet_config_.anchor_ratios[layer].size();
  std::vector<float> h_ratios(ratios_size);
  std::vector<float> w_ratios(ratios_size);
  for (int i = 0; i < retinanet_config_.anchor_ratios[layer].size(); i++) {
    h_ratios[i] = sqrt(anchor_ratios[i]);
    w_ratios[i] = 1 / h_ratios[i];
  }

  auto anchor_sizes = retinanet_config_.anchor_sizes[layer];
  std::vector<float> ws;
  std::vector<float> hs;
  for (int i = 0; i < w_ratios.size(); i++) {
    for (int j = 0; j < anchor_sizes.size(); j++) {
      ws.push_back(anchor_sizes[j] * w_ratios[i]);
      hs.push_back(anchor_sizes[j] * h_ratios[i]);
    }
  }

  retinanet_config_.base_anchors[layer].resize(ws.size());
  for (int i = 0; i < ws.size(); i++) {
    retinanet_config_.base_anchors[layer][i].push_back(round_float(-ws[i] / 2));
    retinanet_config_.base_anchors[layer][i].push_back(round_float(-hs[i] / 2));
    retinanet_config_.base_anchors[layer][i].push_back(round_float(ws[i] / 2));
    retinanet_config_.base_anchors[layer][i].push_back(round_float(hs[i] / 2));
  }
  return 0;
}

double QATRetinanetPostProcessMethod::Dequanti(
    int32_t data,
    int layer,
    bool big_endian,
    int offset,
    hbDNNTensorProperties &properties) {
  hbDNNQuantiType quanti_type = properties.quantiType;
  uint8_t *shift_value = properties.shift.shiftData;
  float *scale_value = properties.scale.scaleData;
  if (quanti_type == SHIFT) {
    return static_cast<double>(r_int32(data, big_endian)) /
           static_cast<double>(1 << shift_value[offset]);
  } else if (quanti_type == SCALE) {
    return static_cast<double>(r_int32(data, big_endian)) * scale_value[offset];
  }
  VLOG(EXAMPLE_SYSTEM) << "quanti type error!";
  return 0.0;
}
