// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_yolo3_darknet_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQYolo3DarknetPostProcessMethod);

PTQYolo3DarknetConfig default_ptq_yolo3_darknet_config = {
    {32, 16, 8},
    {{{3.625, 2.8125}, {4.875, 6.1875}, {11.65625, 10.1875}},
     {{1.875, 3.8125}, {3.875, 2.8125}, {3.6875, 7.4375}},
     {{1.25, 1.625}, {2.0, 3.75}, {4.125, 2.875}}},
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

int PTQYolo3DarknetPostProcessMethod::Init(
    const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQYolo3DarknetPostProcessMethod init from file";
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

int PTQYolo3DarknetPostProcessMethod::InitFromJsonString(
    const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQYolo3DarknetPostProcessMethod Json string:"
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

      VLOG_IF(EXAMPLE_SYSTEM,
              anchors.size() != yolo3_config_.anchors_table[0].size())
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
    yolo3_config_.class_num = document["class_num"].GetInt();
  }

  VLOG(EXAMPLE_DEBUG) << "score_threshold: " << score_threshold_
                      << ", nms_nms_threshold:" << nms_threshold_
                      << ", nms_topk: " << nms_top_k_ << ", "
                      << yolo3_config_.Str();

  return 0;
}

std::vector<xstream::BaseDataPtr> PTQYolo3DarknetPostProcessMethod::DoProcess(
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
      << "PTQYolo3DarknetPostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void PTQYolo3DarknetPostProcessMethod::Finalize() {}

int PTQYolo3DarknetPostProcessMethod::UpdateParameter(
    xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT) << "UpdateParameter in PTQYolo3DarknetPostProcessMethod "
                          "not supported yet.";
  return 0;
}

xstream::InputParamPtr PTQYolo3DarknetPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQYolo3DarknetPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQYolo3DarknetPostProcessMethod::GetVersion() const {}

xstream::MethodInfo PTQYolo3DarknetPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQYolo3DarknetPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQYolo3DarknetPostProcessMethod::~PTQYolo3DarknetPostProcessMethod() {}

int PTQYolo3DarknetPostProcessMethod::PostProcess(
    std::vector<hbDNNTensor> &tensors,
    ImageTensor *image_tensor,
    Perception *perception) {
  perception->type = Perception::DET;
  std::vector<Detection> dets;
  for (int i = 0; i < yolo3_config_.strides.size(); i++) {
    if (tensors[i].properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
      PostProcessNHWC(&tensors[i], image_tensor, i, dets);
    } else if (tensors[i].properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
      PostProcessNCHW(&tensors[i], image_tensor, i, dets);
    } else {
      VLOG(EXAMPLE_SYSTEM) << "tensor layout error.";
    }
  }
  nms(dets, nms_threshold_, nms_top_k_, perception->det, false);
  return 0;
}

void PTQYolo3DarknetPostProcessMethod::PostProcessNHWC(
    hbDNNTensor *tensor,
    ImageTensor *frame,
    int layer,
    std::vector<Detection> &dets) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  int num_classes = yolo3_config_.class_num;
  int stride = yolo3_config_.strides[layer];
  int num_pred = yolo3_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo3_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      yolo3_config_.anchors_table[layer];

  double h_ratio = frame->height() * 1.0 / frame->ori_height();
  double w_ratio = frame->width() * 1.0 / frame->ori_width();
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (frame->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }

  // int *shape = tensor->data_shape.d;
  int height, width;
  auto ret = get_tensor_hw(*tensor, &height, &width);
  if (ret != 0) {
    VLOG(EXAMPLE_SYSTEM) << "get_tensor_hw failed";
  }

  for (uint32_t h = 0; h < height; h++) {
    for (uint32_t w = 0; w < width; w++) {
      for (int k = 0; k < anchors.size(); k++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        float *cur_data = data + k * num_pred;
        float objness = cur_data[4];
        for (int index = 0; index < num_classes; ++index) {
          class_pred[index] = cur_data[5 + index];
        }

        float id = argmax(class_pred.begin(), class_pred.end());
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-class_pred[id]));
        double confidence = x1 * x2;

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
            (frame->width() - w_ratio * frame->ori_width()) / 2.0;
        double h_padding =
            (frame->height() - h_ratio * frame->ori_height()) / 2.0;

        double xmin_org = (xmin - w_padding) / w_ratio;
        double xmax_org = (xmax - w_padding) / w_ratio;
        double ymin_org = (ymin - h_padding) / h_ratio;
        double ymax_org = (ymax - h_padding) / h_ratio;

        if (xmin_org > xmax_org || ymin_org > ymax_org) {
          continue;
        }

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, frame->ori_image_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, frame->ori_image_height - 1.0);

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        dets.push_back(Detection((int)id,
                                 confidence,
                                 bbox,
                                 yolo3_config_.class_names[(int)id].c_str()));
      }
      data = data + num_pred * anchors.size();
    }
  }
}

void PTQYolo3DarknetPostProcessMethod::PostProcessNCHW(
    hbDNNTensor *tensor,
    ImageTensor *frame,
    int layer,
    std::vector<Detection> &dets) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  int num_classes = yolo3_config_.class_num;
  int stride = yolo3_config_.strides[layer];
  int num_pred = yolo3_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo3_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      yolo3_config_.anchors_table[layer];

  double h_ratio = frame->height() * 1.0 / frame->ori_height();
  double w_ratio = frame->width() * 1.0 / frame->ori_width();
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (frame->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }

  int height, width;
  auto ret = get_tensor_hw(*tensor, &height, &width);
  if (ret != 0) {
    VLOG(EXAMPLE_SYSTEM) << "get_tensor_hw failed";
  }
  int aligned_h = tensor->properties.validShape.dimensionSize[2];
  int aligned_w = tensor->properties.validShape.dimensionSize[3];
  int aligned_hw = aligned_h * aligned_w;

  for (int k = 0; k < anchors.size(); k++) {
    for (uint32_t h = 0; h < height; h++) {
      for (uint32_t w = 0; w < width; w++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        int stride_hw = h * aligned_w + w;

        float objness = data[(k * num_pred + 4) * aligned_hw + stride_hw];
        for (int index = 0; index < num_classes; ++index) {
          class_pred[index] =
              data[(k * num_pred + index + 5) * aligned_hw + stride_hw];
        }

        float id = argmax(class_pred.begin(), class_pred.end());
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-class_pred[id]));
        double confidence = x1 * x2;

        if (confidence < score_threshold_) {
          continue;
        }

        float center_x = data[(k * num_pred) * aligned_hw + stride_hw];
        float center_y = data[(k * num_pred + 1) * aligned_hw + stride_hw];
        float scale_x = data[(k * num_pred + 2) * aligned_hw + stride_hw];
        float scale_y = data[(k * num_pred + 3) * aligned_hw + stride_hw];

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
            (frame->width() - w_ratio * frame->ori_width()) / 2.0;
        double h_padding =
            (frame->height() - h_ratio * frame->ori_height()) / 2.0;

        double xmin_org = (xmin - w_padding) / w_ratio;
        double xmax_org = (xmax - w_padding) / w_ratio;
        double ymin_org = (ymin - h_padding) / h_ratio;
        double ymax_org = (ymax - h_padding) / h_ratio;

        if (xmin_org > xmax_org || ymin_org > ymax_org) {
          continue;
        }

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, frame->ori_image_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, frame->ori_image_height - 1.0);

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        dets.push_back(Detection((int)id,
                                 confidence,
                                 bbox,
                                 yolo3_config_.class_names[(int)id].c_str()));
      }
    }
  }
}
