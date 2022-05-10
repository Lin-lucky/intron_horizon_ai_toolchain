// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_fcos_post_process_method.h"

#include <iostream>
#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQFcosPostProcessMethod);

PTQFcosConfig default_ptq_fcos_config = {
    {{8, 16, 32, 64, 128}},
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
     "hair drier",    "toothbrush"},
    ""};

struct ScoreId {
  float score;
  int id;
};

int PTQFcosPostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQFcosPostProcessMethod init from file";
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

int PTQFcosPostProcessMethod::InitFromJsonString(const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQFcosPostProcessMethod Json string:"
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

  if (document.HasMember("topk")) {
    topk_ = document["topk"].GetInt();
  }

  if (document.HasMember("strides")) {
    rapidjson::Value &stride_value = document["strides"];
    auto strides_array = stride_value.GetArray();
    for (int i = 0; i < strides_array.Size(); ++i) {
      fcos_config_.strides[i] = strides_array[i].GetInt();
    }
  }

  if (document.HasMember("class_num")) {
    fcos_config_.class_num = document["class_num"].GetInt();
  }

  if (document.HasMember("det_name_list")) {
    // det_name_list
    fcos_config_.det_name_list = document["det_name_list"].GetString();
    ifs_.open(fcos_config_.det_name_list, std::ios::in);
    if (!ifs_.is_open()) {
      VLOG(EXAMPLE_SYSTEM) << "Open " << fcos_config_.det_name_list
                           << " failed!";
      return -1;
    }
    std::string line;
    fcos_config_.class_names.clear();
    while (std::getline(ifs_, line)) {
      fcos_config_.class_names.push_back(line);
    }
  }

  VLOG(EXAMPLE_DEBUG) << " topk: " << topk_
                      << ", class_num: " << fcos_config_.class_num
                      << ", score_threshold: " << score_threshold_
                      << ", stride {" << fcos_config_.strides[0] << ", "
                      << fcos_config_.strides[1] << ", "
                      << fcos_config_.strides[2] << ", "
                      << fcos_config_.strides[3] << ", "
                      << fcos_config_.strides[4] << "}";
  return 0;
}

std::vector<xstream::BaseDataPtr> PTQFcosPostProcessMethod::DoProcess(
    const std::vector<xstream::BaseDataPtr> &input,
    const xstream::InputParamPtr &param) {
  auto image_tensor = std::static_pointer_cast<ImageTensor>(input[0]);
  auto output_tensor = std::static_pointer_cast<TensorVector>(input[1]);
  auto perception = std::shared_ptr<Perception>(new Perception);
  static thread_local Latency latency;
  if (latency_status_) SetLatency(latency);
  //  PostProcess
  auto ret = this->PostProcess(
      output_tensor->tensors, image_tensor.get(), perception.get());
  if (latency_status_) UpdateLatency(latency);

  VLOG(EXAMPLE_DETAIL) << "PostProcess success!";
  release_output_tensor(output_tensor->tensors);
  VLOG(EXAMPLE_DETAIL) << "release output tensor success!";

  std::vector<xstream::BaseDataPtr> res;
  perception->name_ = output_name_;
  res.emplace_back(xstream::BaseDataPtr(perception));
  VLOG(EXAMPLE_DETAIL)
      << "PTQFcosPostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void PTQFcosPostProcessMethod::Finalize() {}

int PTQFcosPostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in PTQFcosPostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr PTQFcosPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQFcosPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQFcosPostProcessMethod::GetVersion() const {}

xstream::MethodInfo PTQFcosPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQFcosPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQFcosPostProcessMethod::~PTQFcosPostProcessMethod() {}

void PTQFcosPostProcessMethod::GetBboxAndScoresNHWC(
    std::vector<hbDNNTensor> &tensors,
    ImageTensor *image_tensor,
    std::vector<Detection> &dets) {
  int ori_h = image_tensor->ori_image_height;
  int ori_w = image_tensor->ori_image_width;
  int input_h = image_tensor->height();
  int input_w = image_tensor->width();
  float w_scale;
  float h_scale;
  // preprocess action is pad and resize
  if (image_tensor->is_pad_resize) {
    float scale = ori_h > ori_w ? ori_h : ori_w;
    w_scale = scale / input_w;
    h_scale = scale / input_h;
  } else {
    w_scale = static_cast<float>(ori_w) / input_w;
    h_scale = static_cast<float>(ori_h) / input_h;
  }

  // fcos stride is {8, 16, 32, 64, 128}
  for (int i = 0; i < 5; i++) {
    auto *cls_data = reinterpret_cast<float *>(tensors[i].sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<float *>(tensors[i + 5].sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<float *>(tensors[i + 10].sysMem[0].virAddr);

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    int *shape = tensors[i].properties.alignedShape.dimensionSize;
    int tensor_h = shape[1];
    int tensor_w = shape[2];
    int tensor_c = shape[3];

    for (int h = 0; h < tensor_h; h++) {
      int offset = h * tensor_w;
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = offset + w;
        ce_data[ce_offset] = 1.0 / (1.0 + exp(-ce_data[ce_offset]));

        int cls_offset = ce_offset * tensor_c;
        ScoreId tmp_score = {cls_data[cls_offset], 0};
        for (int cls_c = 1; cls_c < tensor_c; cls_c++) {
          int cls_index = cls_offset + cls_c;
          if (cls_data[cls_index] > tmp_score.score) {
            tmp_score.id = cls_c;
            tmp_score.score = cls_data[cls_index];
          }
        }
        tmp_score.score = 1.0 / (1.0 + exp(-tmp_score.score));
        tmp_score.score = std::sqrt(tmp_score.score * ce_data[ce_offset]);
        if (tmp_score.score <= score_threshold_) continue;

        // get detection box
        Detection detection;
        int index = 4 * (h * tensor_w + w);
        auto &strides = fcos_config_.strides;

        detection.bbox.xmin =
            ((w + 0.5) * strides[i] - bbox_data[index]) * w_scale;
        detection.bbox.ymin =
            ((h + 0.5) * strides[i] - bbox_data[index + 1]) * h_scale;
        detection.bbox.xmax =
            ((w + 0.5) * strides[i] + bbox_data[index + 2]) * w_scale;
        detection.bbox.ymax =
            ((h + 0.5) * strides[i] + bbox_data[index + 3]) * h_scale;

        detection.score = tmp_score.score;
        detection.id = tmp_score.id;
        detection.class_name = fcos_config_.class_names[detection.id].c_str();
        dets.push_back(detection);
      }
    }
  }
}

void PTQFcosPostProcessMethod::GetBboxAndScoresNCHW(
    std::vector<hbDNNTensor> &tensors,
    ImageTensor *image_tensor,
    std::vector<Detection> &dets) {
  int ori_h = image_tensor->ori_image_height;
  int ori_w = image_tensor->ori_image_width;
  int input_h = image_tensor->height();
  int input_w = image_tensor->width();
  float w_scale;
  float h_scale;
  // preprocess action is pad and resize
  if (image_tensor->is_pad_resize) {
    float scale = ori_h > ori_w ? ori_h : ori_w;
    w_scale = scale / input_w;
    h_scale = scale / input_h;
  } else {
    w_scale = static_cast<float>(ori_w) / input_w;
    h_scale = static_cast<float>(ori_h) / input_h;
  }

  for (int i = 0; i < 5; i++) {
    auto *cls_data = reinterpret_cast<float *>(tensors[i].sysMem[0].virAddr);
    auto *bbox_data =
        reinterpret_cast<float *>(tensors[i + 5].sysMem[0].virAddr);
    auto *ce_data =
        reinterpret_cast<float *>(tensors[i + 10].sysMem[0].virAddr);

    // 同一个尺度下，tensor[i],tensor[i+5],tensor[i+10]出来的hw都一致，64*64/32*32/...
    int *shape = tensors[i].properties.alignedShape.dimensionSize;
    int tensor_c = shape[1];
    int tensor_h = shape[2];
    int tensor_w = shape[3];
    int aligned_hw = tensor_h * tensor_w;

    for (int h = 0; h < tensor_h; h++) {
      int offset = h * tensor_w;
      for (int w = 0; w < tensor_w; w++) {
        // get score
        int ce_offset = offset + w;
        ce_data[ce_offset] = 1.0 / (1.0 + exp(-ce_data[ce_offset]));

        ScoreId tmp_score = {cls_data[offset + w], 0};
        for (int cls_c = 1; cls_c < tensor_c; cls_c++) {
          int cls_index = cls_c * aligned_hw + offset + w;
          if (cls_data[cls_index] > tmp_score.score) {
            tmp_score.id = cls_c;
            tmp_score.score = cls_data[cls_index];
          }
        }
        tmp_score.score = 1.0 / (1.0 + exp(-tmp_score.score));
        tmp_score.score = std::sqrt(tmp_score.score * ce_data[ce_offset]);
        if (tmp_score.score <= score_threshold_) continue;

        // get detection box
        auto &strides = fcos_config_.strides;
        Detection detection;
        detection.bbox.xmin =
            ((w + 0.5) * strides[i] - bbox_data[offset + w]) * w_scale;
        detection.bbox.ymin =
            ((h + 0.5) * strides[i] - bbox_data[1 * aligned_hw + offset + w]) *
            h_scale;
        detection.bbox.xmax =
            ((w + 0.5) * strides[i] + bbox_data[2 * aligned_hw + offset + w]) *
            w_scale;
        detection.bbox.ymax =
            ((h + 0.5) * strides[i] + bbox_data[3 * aligned_hw + offset + w]) *
            h_scale;

        detection.score = tmp_score.score;
        detection.id = tmp_score.id;
        detection.class_name = fcos_config_.class_names[detection.id].c_str();
        dets.push_back(detection);
      }
    }
  }
}

int PTQFcosPostProcessMethod::PostProcess(std::vector<hbDNNTensor> &tensors,
                                          ImageTensor *image_tensor,
                                          Perception *perception) {
  perception->type = Perception::DET;
  // TODO(daofu.zhang) check namelist for model output, compare with config file
  int h_index, w_index, c_index;
  int ret = get_tensor_hwc_index(&tensors[0], &h_index, &w_index, &c_index);
  if (ret != 0 &&
      fcos_config_.class_names.size() !=
          tensors[0].properties.alignedShape.dimensionSize[c_index]) {
    VLOG(EXAMPLE_SYSTEM)
        << "User det_name_list in config file: '" << fcos_config_.det_name_list
        << "', is not compatible with this model. "
        << fcos_config_.class_names.size()
        << " != " << tensors[0].properties.alignedShape.dimensionSize[c_index]
        << ".";
  }
  for (int i = 0; i < tensors.size(); i++) {
    hbSysFlushMem(&(tensors[i].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  }
  std::vector<std::vector<ScoreId>> scores;
  std::vector<Detection> dets;
  if (tensors[0].properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    GetBboxAndScoresNHWC(tensors, image_tensor, dets);
  } else if (tensors[0].properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    GetBboxAndScoresNCHW(tensors, image_tensor, dets);
  } else {
    VLOG(EXAMPLE_SYSTEM) << "tensor layout error.";
  }
  yolo5_nms(dets, 0.6, topk_, perception->det, false);
  return 0;
}
