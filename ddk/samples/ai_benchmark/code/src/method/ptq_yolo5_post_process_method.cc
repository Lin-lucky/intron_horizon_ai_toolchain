// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_yolo5_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQYolo5PostProcessMethod);

PTQYolo5Config default_ptq_yolo5_config = {
    {8, 16, 32},
    {{{10, 13}, {16, 30}, {33, 23}},
     {{30, 61}, {62, 45}, {59, 119}},
     {{116, 90}, {156, 198}, {373, 326}}},
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

int PTQYolo5PostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQYolo5PostProcessMethod init from file";
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

int PTQYolo5PostProcessMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQYolo5PostProcessMethod Json string:"
                      << config.data();
  rapidjson::Document document;
  document.Parse(config.data());

  char *latency_log = getenv("SHOW_LATENCY_LOG");
  latency_status_ = (latency_log != nullptr) ? atoi(latency_log) : 0;

  VLOG(EXAMPLE_DEBUG) << "start to load yolo5 output config";
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
    nms_top_k_ = document["nms_top_k"].GetFloat();
  }

  if (document.HasMember("strides")) {
    rapidjson::Value &stride_value = document["strides"];
    auto strides_array = stride_value.GetArray();
    for (int i = 0; i < strides_array.Size(); ++i) {
      yolo5_config_.strides[i] = strides_array[i].GetInt();
    }
  }

  if (document.HasMember("anchors_table")) {
    // Anchors table
    rapidjson::Value &anchors_value = document["anchors_table"];
    auto anchors_array = anchors_value.GetArray();
    VLOG_IF(EXAMPLE_SYSTEM,
            anchors_array.Size() != yolo5_config_.strides.size())
        << "Feature num should be equal to output num";

    yolo5_config_.anchors_table.reserve(anchors_array.Size());
    for (int i = 0; i < anchors_array.Size(); ++i) {
      auto anchor_pairs = anchors_array[i].GetArray();
      auto &anchors = yolo5_config_.anchors_table[i];
      anchors.reserve(anchor_pairs.Size());

      VLOG_IF(EXAMPLE_SYSTEM,
              anchors.size() != yolo5_config_.anchors_table[0].size())
          << "Anchors num should be equal in every feature";

      for (int j = 0; j < anchor_pairs.Size(); ++j) {
        auto anchor = anchor_pairs[j].GetArray();
        anchors[j] =
            std::make_pair(anchor[0].GetDouble(), anchor[1].GetDouble());
      }
    }
  }

  if (document.HasMember("class_num")) {
    // Class number
    yolo5_config_.class_num = document["class_num"].GetInt();
  }

  VLOG(EXAMPLE_DEBUG) << "score_threshold: " << score_threshold_
                      << ", nms_nms_threshold:" << nms_threshold_
                      << ", nms_topk: " << nms_top_k_ << ", "
                      << yolo5_config_.Str();

  return 0;
}

std::vector<xstream::BaseDataPtr> PTQYolo5PostProcessMethod::DoProcess(
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
      << "PTQYolo5PostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void PTQYolo5PostProcessMethod::Finalize() {}

int PTQYolo5PostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in PTQYolo5PostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr PTQYolo5PostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQYolo5PostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQYolo5PostProcessMethod::GetVersion() const {}

xstream::MethodInfo PTQYolo5PostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQYolo5PostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = false;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQYolo5PostProcessMethod::~PTQYolo5PostProcessMethod() {}

int PTQYolo5PostProcessMethod::PostProcess(std::vector<hbDNNTensor> &tensors,
                                           ImageTensor *image_tensor,
                                           Perception *perception) {
  perception->type = Perception::DET;
  std::vector<Detection> dets;
  for (int i = 0; i < tensors.size(); i++) {
    PostProcess(&tensors[i], image_tensor, i, dets);
  }
  yolo5_nms(dets, nms_threshold_, nms_top_k_, perception->det, false);
  return 0;
}

void PTQYolo5PostProcessMethod::PostProcess(hbDNNTensor *tensor,
                                            ImageTensor *frame,
                                            int layer,
                                            std::vector<Detection> &dets) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  int num_classes = yolo5_config_.class_num;
  int stride = yolo5_config_.strides[layer];
  int num_pred = yolo5_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo5_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      yolo5_config_.anchors_table[layer];

  double h_ratio = frame->height() * 1.0 / frame->ori_height();
  double w_ratio = frame->width() * 1.0 / frame->ori_width();
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (frame->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }

  //  int *shape = tensor->data_shape.d;
  int height, width;
  auto ret = get_tensor_hw(*tensor, &height, &width);
  if (ret != 0) {
    VLOG(EXAMPLE_SYSTEM) << "get_tensor_hw failed";
  }

  int anchor_num = anchors.size();
  //  for (uint32_t a = 0; a < anchors.size(); a++) {
  for (uint32_t h = 0; h < height; h++) {
    for (uint32_t w = 0; w < width; w++) {
      for (int k = 0; k < anchor_num; k++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        float *cur_data = data + k * num_pred;
        float objness = cur_data[4];

        int id = argmax(cur_data + 5, cur_data + 5 + num_classes);
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-cur_data[id + 5]));
        double confidence = x1 * x2;

        if (confidence < score_threshold_) {
          continue;
        }

        float center_x = cur_data[0];
        float center_y = cur_data[1];
        float scale_x = cur_data[2];
        float scale_y = cur_data[3];

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

        double box_scale_x =
            std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
        double box_scale_y =
            std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        double w_padding =
            (frame->width() - w_ratio * frame->ori_width()) / 2.0;
        double h_padding =
            (frame->height() - h_ratio * frame->ori_height()) / 2.0;

        double xmin_org = (xmin - w_padding) / w_ratio;
        double xmax_org = (xmax - w_padding) / w_ratio;
        double ymin_org = (ymin - h_padding) / h_ratio;
        double ymax_org = (ymax - h_padding) / h_ratio;

        if (xmax_org <= 0 || ymax_org <= 0) {
          continue;
        }

        if (xmin_org > xmax_org || ymin_org > ymax_org) {
          continue;
        }

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, frame->ori_image_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, frame->ori_image_height - 1.0);

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        dets.emplace_back((int)id,
                          confidence,
                          bbox,
                          yolo5_config_.class_names[(int)id].c_str());
      }
      data = data + num_pred * anchors.size();
    }
  }
}
