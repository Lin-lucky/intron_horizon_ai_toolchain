// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_ssd_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQSSDPostProcessMethod);

#define SSD_CLASS_NUM_P1 21

SSDConfig default_ssd_config = {
    {0.1, 0.1, 0.2, 0.2},
    {0, 0, 0, 0},
    {0.5, 0.5},
    {15, 30, 60, 100, 150, 300},
    {{60, -1}, {105, 150}, {150, 195}, {195, 240}, {240, 285}, {285, 300}},
    {{2, 0.5, 0, 0},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3}},
    20,
    {"aeroplane",   "bicycle", "bird",  "boaupdate", "bottle",
     "bus",         "car",     "cat",   "chair",     "cow",
     "diningtable", "dog",     "horse", "motorbike", "person",
     "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"}};

int PTQSSDPostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQSSDPostProcessMethod init from file";
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

int PTQSSDPostProcessMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQSSDPostProcessMethod Json string:"
                      << config.data();

  char *latency_log = getenv("SHOW_LATENCY_LOG");
  latency_status_ = (latency_log != nullptr) ? atoi(latency_log) : 0;

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

  if (document.HasMember("ssd")) {
    rapidjson::Value &ssd = document["ssd"];

    // Std
    auto std_array = ssd["std"].GetArray();
    ssd_config_.std.resize(std_array.Size());
    for (int i = 0; i < std_array.Size(); i++) {
      ssd_config_.std[i] = std_array[i].GetFloat();
    }

    // Mean
    auto mean_array = ssd["mean"].GetArray();
    ssd_config_.mean.resize(mean_array.Size());
    for (int i = 0; i < mean_array.Size(); i++) {
      ssd_config_.mean[i] = mean_array[i].GetFloat();
    }

    // Offset
    auto offset_array = ssd["offset"].GetArray();
    ssd_config_.offset.resize(offset_array.Size());
    for (int i = 0; i < offset_array.Size(); i++) {
      ssd_config_.offset[i] = offset_array[i].GetFloat();
    }

    // Step
    auto step_array = ssd["step"].GetArray();
    ssd_config_.step.resize(step_array.Size());
    for (int i = 0; i < step_array.Size(); i++) {
      ssd_config_.step[i] = step_array[i].GetInt();
    }

    // Anchor size
    auto anchor_size_array = ssd["anchor_size"].GetArray();
    ssd_config_.anchor_size.resize(anchor_size_array.Size() / 2);
    for (int i = 0; i < anchor_size_array.Size(); i += 2) {
      ssd_config_.anchor_size[i] = std::make_pair(
          anchor_size_array[i].GetFloat(), anchor_size_array[i + 1].GetFloat());
    }

    // Anchor ratio
    auto ratio_array = ssd["anchor_ratio"].GetArray();
    ssd_config_.anchor_ratio.resize(ratio_array.Size() / 4);
    for (int i = 0; i < ratio_array.Size(); i += 4) {
      std::vector<float> &ratio = ssd_config_.anchor_ratio[i];
      ratio[0] = ratio_array[i].GetFloat();
      ratio[1] = ratio_array[i + 1].GetFloat();
      ratio[2] = ratio_array[i + 2].GetFloat();
      ratio[3] = ratio_array[i + 3].GetFloat();
    }

    // Class num
    ssd_config_.class_num = ssd["class_num"].GetInt();

    // Class names
    auto class_names_arr = ssd["class_names"].GetArray();
    ssd_config_.class_names.resize(ssd_config_.class_num);
    for (int i = 0; i < ssd_config_.class_names.size(); i++) {
      ssd_config_.class_names[i] = class_names_arr[i].GetString();
    }
  }

  return 0;
}

std::vector<xstream::BaseDataPtr> PTQSSDPostProcessMethod::DoProcess(
    const std::vector<xstream::BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  auto image_tensor = std::static_pointer_cast<ImageTensor>(input[0]);
  auto output_tensor = std::static_pointer_cast<TensorVector>(input[1]);
  auto perception = std::shared_ptr<Perception>(new Perception);

  static thread_local Latency latency;
  if (latency_status_) SetLatency(latency);
  //  PostProcess
  this->PostProcess(
      output_tensor->tensors, image_tensor.get(), perception.get());
  if (latency_status_) UpdateLatency(latency);

  VLOG(EXAMPLE_DETAIL) << "PostProcess success!";
  release_output_tensor(output_tensor->tensors);
  VLOG(EXAMPLE_DETAIL) << "release output tensor success!";

  std::vector<xstream::BaseDataPtr> res;
  perception->name_ = output_name_;
  res.emplace_back(xstream::BaseDataPtr(perception));
  VLOG(EXAMPLE_DETAIL)
      << "PTQSSDPostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void PTQSSDPostProcessMethod::Finalize() {}

int PTQSSDPostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in PTQSSDPostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr PTQSSDPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQSSDPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQSSDPostProcessMethod::GetVersion() const {}

xstream::MethodInfo PTQSSDPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQSSDPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQSSDPostProcessMethod::~PTQSSDPostProcessMethod() {}

int PTQSSDPostProcessMethod::PostProcess(std::vector<hbDNNTensor> &tensors,
                                         ImageTensor *image_tensor,
                                         Perception *perception) {
  perception->type = Perception::DET;
  std::vector<std::vector<Anchor>> anchors_table;
  int layer_num = ssd_config_.step.size();

  anchors_table.resize(layer_num);
  std::vector<Detection> dets;
  for (int i = 0; i < layer_num; i++) {
    int height, width;
    std::vector<Anchor> &anchors = anchors_table[i];
    get_tensor_hw(tensors[i * 2], &height, &width);
    SsdAnchors(anchors_table[i], i, height, width);
    GetBboxAndScores(&tensors[i * 2 + 1],
                     &tensors[i * 2],
                     dets,
                     anchors,
                     ssd_config_.class_num + 1,
                     0.0001,
                     image_tensor);
  }
  nms(dets, nms_threshold_, nms_top_k_, perception->det, false);
  return 0;
}

int PTQSSDPostProcessMethod::SsdAnchors(std::vector<Anchor> &anchors,
                                        int layer,
                                        int layer_height,
                                        int layer_width) {
  int step = ssd_config_.step[layer];
  float min_size = ssd_config_.anchor_size[layer].first;
  float max_size = ssd_config_.anchor_size[layer].second;
  auto &anchor_ratio = ssd_config_.anchor_ratio[layer];
  for (int i = 0; i < layer_height; i++) {
    for (int j = 0; j < layer_width; j++) {
      float cy = (i + ssd_config_.offset[0]) * step;
      float cx = (j + ssd_config_.offset[1]) * step;
      anchors.emplace_back(Anchor(cx, cy, min_size, min_size));
      if (max_size > 0) {
        anchors.emplace_back(Anchor(cx,
                                    cy,
                                    std::sqrt(max_size * min_size),
                                    std::sqrt(max_size * min_size)));
      }
      for (int k = 0; k < 4; k++) {
        if (anchor_ratio[k] == 0) continue;
        float sr = std::sqrt(anchor_ratio[k]);
        float w = min_size * sr;
        float h = min_size / sr;
        anchors.emplace_back(Anchor(cx, cy, w, h));
      }
    }
  }
  return 0;
}

int PTQSSDPostProcessMethod::GetBboxAndScores(hbDNNTensor *c_tensor,
                                              hbDNNTensor *bbox_tensor,
                                              std::vector<Detection> &dets,
                                              std::vector<Anchor> &anchors,
                                              int class_num,
                                              float cut_off_threshold,
                                              ImageTensor *frame) {
  int *shape = c_tensor->properties.validShape.dimensionSize;
  int32_t c_batch_size = shape[0];
  int h_idx, w_idx, c_idx;
  get_tensor_hwc_index(c_tensor, &h_idx, &w_idx, &c_idx);

  int32_t c_hnum = shape[h_idx];
  int32_t c_wnum = shape[w_idx];
  int32_t c_cnum = shape[c_idx];
  uint32_t anchor_num_per_pixel = c_cnum / class_num;

  shape = bbox_tensor->properties.validShape.dimensionSize;
  int32_t b_batch_size = shape[0];
  get_tensor_hwc_index(c_tensor, &h_idx, &w_idx, &c_idx);

  int32_t b_hnum = shape[h_idx];
  int32_t b_wnum = shape[w_idx];
  int32_t b_cnum = shape[c_idx];

  assert(anchor_num_per_pixel == b_cnum / 4);
  assert(c_batch_size == b_batch_size && c_hnum == b_hnum && c_wnum == b_wnum);
  auto box_num = b_batch_size * b_hnum * b_wnum * anchor_num_per_pixel;

  hbSysFlushMem(&(c_tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *raw_cls_data = reinterpret_cast<float *>(c_tensor->sysMem[0].virAddr);

  hbSysFlushMem(&(bbox_tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *raw_box_data =
      reinterpret_cast<float *>(bbox_tensor->sysMem[0].virAddr);
  // all scores for each bbox
  std::vector<double> scores(box_num * class_num, 0.0);
  std::vector<Bbox> bboxes(box_num);

  for (int i = 0; i < box_num; i++) {
    int start = i * 4;
    float dx = raw_box_data[start];
    float dy = raw_box_data[start + 1];
    float dw = raw_box_data[start + 2];
    float dh = raw_box_data[start + 3];

    auto x_min = (anchors[i].cx - anchors[i].w / 2) / frame->width();
    auto y_min = (anchors[i].cy - anchors[i].h / 2) / frame->height();
    auto x_max = (anchors[i].cx + anchors[i].w / 2) / frame->width();
    auto y_max = (anchors[i].cy + anchors[i].h / 2) / frame->height();

    auto prior_w = x_max - x_min;
    auto prior_h = y_max - y_min;
    auto prior_center_x = (x_max + x_min) / 2;
    auto prior_center_y = (y_max + y_min) / 2;
    auto decode_x = ssd_config_.std[0] * dx * prior_w + prior_center_x;
    auto decode_y = ssd_config_.std[1] * dy * prior_h + prior_center_y;
    auto decode_w = std::exp(ssd_config_.std[2] * dw) * prior_w;
    auto decode_h = std::exp(ssd_config_.std[3] * dh) * prior_h;

    auto xmin_org = (decode_x - decode_w * 0.5) * frame->ori_image_width;
    auto ymin_org = (decode_y - decode_h * 0.5) * frame->ori_image_height;
    auto xmax_org = (decode_x + decode_w * 0.5) * frame->ori_image_width;
    auto ymax_org = (decode_y + decode_h * 0.5) * frame->ori_image_height;

    xmin_org = std::max(xmin_org, 0.0);
    xmax_org = std::min(xmax_org, frame->ori_image_width - 1.0);
    ymin_org = std::max(ymin_org, 0.0);
    ymax_org = std::min(ymax_org, frame->ori_image_height - 1.0);

    if (xmax_org <= 0 || ymax_org <= 0) continue;
    if (xmin_org > xmax_org || ymin_org > ymax_org) continue;

    uint32_t res_id_cur_anchor = i * class_num;
    // get softmax sum
    double sum = 0;
    int max_id = 0;
    double max_score = 0;
    for (int cls = 0; cls < class_num; ++cls) {
      float cls_value = raw_cls_data[res_id_cur_anchor + cls];
      scores[res_id_cur_anchor + cls] = std::exp(cls_value);
      sum += scores[res_id_cur_anchor + cls];
      /* scores should be larger than background score, or else will not be
      selected */
      if (cls != 0 && scores[res_id_cur_anchor + cls] > max_score &&
          scores[res_id_cur_anchor + cls] > scores[res_id_cur_anchor]) {
        // the first one is background score, id should be minus 1
        max_id = cls - 1;
        max_score = scores[res_id_cur_anchor + cls];
      }
    }
    // get softmax score
    max_score = max_score / sum;
    if (max_score > score_threshold_) {
      Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
      dets.emplace_back(Detection((int)max_id,
                                  max_score,
                                  bbox,
                                  ssd_config_.class_names[max_id].c_str()));
    }
  }
  return 0;
}
