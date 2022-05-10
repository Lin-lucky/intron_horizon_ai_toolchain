// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_yolo2_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQYolo2PostProcessMethod);

PTQYolo2Config default_ptq_yolo2_config = {
    32,
    {{0.57273, 0.677385},
     {1.87446, 2.06253},
     {3.33843, 5.47434},
     {7.88282, 3.52778},
     {9.77052, 9.16828}},
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

int PTQYolo2PostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQYolo2PostProcessMethod init from file";
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

int PTQYolo2PostProcessMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQYolo2PostProcessMethod Json string:"
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

  if (document.HasMember("nms_top_k")) {
    nms_top_k_ = document["nms_top_k"].GetInt();
  }

  if (document.HasMember("strides")) {
    yolo2_config_.stride = document["strides"].GetInt();
  }

  if (document.HasMember("anchors_table")) {
    // Anchors table
    rapidjson::Value &anchors_value = document["anchors_table"];
    auto anchors_array = anchors_value.GetArray();
    yolo2_config_.anchors_table.resize(anchors_array.Size() / 2);
    for (int i = 0; i < anchors_array.Size() / 2; i++) {
      yolo2_config_.anchors_table[i] =
          std::make_pair(anchors_array[i * 2].GetDouble(),
                         anchors_array[i * 2 + 1].GetDouble());
    }
  }

  if (document.HasMember("class_num")) {
    // Class num
    yolo2_config_.class_num = document["class_num"].GetInt();
  }

  VLOG(EXAMPLE_DEBUG) << "score_threshold: " << score_threshold_
                      << ", nms_nms_threshold:" << nms_threshold_
                      << ", nms_topk: " << nms_top_k_ << ", "
                      << yolo2_config_.Str();
  return 0;
}

std::vector<xstream::BaseDataPtr> PTQYolo2PostProcessMethod::DoProcess(
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
      << "PTQYolo2PostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void PTQYolo2PostProcessMethod::Finalize() {}

int PTQYolo2PostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in PTQYolo2PostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr PTQYolo2PostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQYolo2PostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQYolo2PostProcessMethod::GetVersion() const {}

xstream::MethodInfo PTQYolo2PostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQYolo2PostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = true;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQYolo2PostProcessMethod::~PTQYolo2PostProcessMethod() = default;

int PTQYolo2PostProcessMethod::PostProcess(std::vector<hbDNNTensor> &tensors,
                                           ImageTensor *image_tensor,
                                           Perception *perception) {
  perception->type = Perception::DET;
  hbSysFlushMem(&(tensors[0].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *data = reinterpret_cast<float *>(tensors[0].sysMem[0].virAddr);
  auto &anchors_table = yolo2_config_.anchors_table;
  int num_classes = yolo2_config_.class_num;
  float stride = static_cast<float>(yolo2_config_.stride);
  int num_pred = num_classes + 4 + 1;
  std::vector<Detection> dets;
  std::vector<float> class_pred(num_classes, 0.0);
  double w_ratio = image_tensor->width() * 1.0 / image_tensor->ori_width();
  double h_ratio = image_tensor->height() * 1.0 / image_tensor->ori_height();
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (image_tensor->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }
  int height, width;
  get_tensor_hw(tensors[0], &height, &width);
  // int *shape = tensor->data_shape.d;
  for (uint32_t h = 0; h < height; h++) {
    for (uint32_t w = 0; w < width; w++) {
      for (int k = 0; k < anchors_table.size(); k++) {
        double anchor_x = anchors_table[k].first;
        double anchor_y = anchors_table[k].second;
        float *cur_data = data + k * num_pred;

        float objness = cur_data[4];
        for (int index = 0; index < num_classes; ++index) {
          class_pred[index] = cur_data[5 + index];
        }

        float id = argmax(class_pred.begin(), class_pred.end());

        float confidence = (1.f / (1 + std::exp(-objness))) *
                           (1.f / (1 + std::exp(-class_pred[id])));

        if (confidence < score_threshold_) {
          continue;
        }

        float center_x = cur_data[0];
        float center_y = cur_data[1];
        float scale_x = cur_data[2];
        float scale_y = cur_data[3];

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) + h) * stride;

        double box_scale_x = std::exp(scale_x) * anchor_x * stride;
        double box_scale_y = std::exp(scale_y) * anchor_y * stride;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        double w_padding =
            (image_tensor->width() - w_ratio * image_tensor->ori_width()) / 2.0;
        double h_padding =
            (image_tensor->height() - h_ratio * image_tensor->ori_height()) /
            2.0;

        double xmin_org = (xmin - w_padding) / w_ratio;
        double xmax_org = (xmax - w_padding) / w_ratio;
        double ymin_org = (ymin - h_padding) / h_ratio;
        double ymax_org = (ymax - h_padding) / h_ratio;

        if (xmin_org > xmax_org || ymin_org > ymax_org) {
          continue;
        }

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, image_tensor->ori_image_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, image_tensor->ori_image_height - 1.0);

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        dets.emplace_back(
            Detection((int)id,
                      confidence,
                      bbox,
                      yolo2_config_.class_names[(int)id].c_str()));
      }
      data = data + num_pred * anchors_table.size();
    }
  }

  nms(dets, nms_threshold_, nms_top_k_, perception->det, false);
  return 0;
}
