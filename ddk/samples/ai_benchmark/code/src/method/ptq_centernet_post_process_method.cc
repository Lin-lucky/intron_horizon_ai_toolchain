// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "method/ptq_centernet_post_process_method.h"

#include <queue>

#include "method/method_data.h"
#include "method/method_factory.h"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "utils/algorithm.h"
#include "utils/nms.h"
#include "utils/tensor_utils.h"

DEFINE_AND_REGISTER_METHOD(PTQCenternetPostProcessMethod);

PTQCenternetConfig default_ptq_centernet_config = {
    80, {"person",        "bicycle",      "car",
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

struct DecodeData {
  float topk_score;
  int topk_inds;
  int topk_clses;
  float topk_ys;
  float topk_xs;

  friend bool operator>(const DecodeData &ldt, const DecodeData &rdt) {
    return (ldt.topk_clses > rdt.topk_clses);
  }
};

struct DataNode {
  float value;
  int indx;

  friend bool operator>(const DataNode &ldt, const DataNode &rdt) {
    return (ldt.value > rdt.value);
  }
};

// order topK data in node
static void top_k_helper(DataNode *node, int topk, int len) {
  std::priority_queue<int, std::vector<DataNode>, std::greater<DataNode>> heap;
  int i = 0;
  while (i < len) {
    if (i < topk) {
      heap.push(node[i]);
    } else {
      if (heap.top().value < node[i].value) {
        heap.pop();
        heap.push(node[i]);
      }
    }
    i++;
  }
  for (int j = 0; j < topk; j++) {
    node[j] = heap.top();
    heap.pop();
  }
}

static void sigmoid(std::vector<DecodeData> &decode) {
  int size = decode.size();
  for (int i = 0; i < size; i++) {
    decode[i].topk_score = 1.0 / (1.0 + exp(-decode[i].topk_score));
  }
}

// 对于特征图上单个点来说如果是周围（默认是包含自己周围９个点）最大则保留，记录下标．
static void nms_max_pool2d(hbDNNTensor tensor, std::vector<int> &index) {
  int *shape = tensor.properties.validShape.dimensionSize;
  int input_c = shape[1];
  int input_h = shape[2];
  int input_w = shape[3];

  float *raw_heat_map_data =
      reinterpret_cast<float *>(tensor.sysMem[0].virAddr);

  // padding stride is 1, around array and init 0
  int pad_h = input_h + 2;
  int pad_w = input_w + 2;
  std::vector<std::vector<float>> padding_out(pad_h, std::vector<float>(pad_w));

  for (int c = 0; c < input_c; c++) {
    // copy data to padding arrary
    int channel_offset = c * input_h * input_w;
    for (int h = 1; h < pad_h - 1; h++) {
      int h_offset = (h - 1) * input_w;
      memcpy(&padding_out[h][1],
             raw_heat_map_data + channel_offset + h_offset,
             4 * input_w);
    }

    for (int h = 1; h < pad_h - 1; h++) {
      int h_offset = (h - 1) * input_w;
      for (int w = 1; w < pad_w - 1; w++) {
        float tmp[9] = {0.0};
        tmp[0] = padding_out[h - 1][w - 1];
        tmp[1] = padding_out[h - 1][w];
        tmp[2] = padding_out[h - 1][w + 1];
        tmp[3] = padding_out[h][w - 1];
        tmp[4] = padding_out[h][w];
        tmp[5] = padding_out[h][w + 1];
        tmp[6] = padding_out[h + 1][w - 1];
        tmp[7] = padding_out[h + 1][w];
        tmp[8] = padding_out[h + 1][w + 1];

        int out_indx = channel_offset + h_offset + w - 1;
        int i = 0;
        for (; i < 9; i++) {
          if (tmp[4] < tmp[i]) break;
        }
        if (i == 9) index.push_back(out_indx);
      }
    }
  }
}

static void topk(hbDNNTensor tensor,
                 int topk,
                 std::vector<DecodeData> &decode,
                 std::vector<int> &index) {
  int len = index.size();
  DataNode *node = new DataNode[len]();

  float *raw_heat_map_data =
      reinterpret_cast<float *>(tensor.sysMem[0].virAddr);
  // 从原始数据中取出有效数据，80*128*128 / 9 的数据量
  for (int i = 0; i < len; i++) {
    node[i].indx = index[i];
    node[i].value = raw_heat_map_data[index[i]];
  }
  // 对有效数据topk堆排序
  top_k_helper(node, topk, len);

  int input_h = tensor.properties.validShape.dimensionSize[2];
  int input_w = tensor.properties.validShape.dimensionSize[3];
  for (int i = 0; i < topk; i++) {
    decode[i].topk_score = node[i].value;
    // 除以面积w*h，在哪一层就属于哪一类
    decode[i].topk_clses = node[i].indx / (input_h * input_w);
    decode[i].topk_inds = node[i].indx % (input_h * input_w);
    decode[i].topk_ys = static_cast<float>(decode[i].topk_inds / input_w);
    decode[i].topk_xs = static_cast<float>(decode[i].topk_inds % input_w);
  }
  // 按照类别排序，方便终端打印查看结果
  std::stable_sort(&decode[0], &decode[0] + topk, std::greater<DecodeData>());
  delete[] node;
}

int PTQCenternetPostProcessMethod::Init(const std::string &config_file_path) {
  VLOG(EXAMPLE_DEBUG) << "PTQCenternetPostProcessMethod init from file";
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

int PTQCenternetPostProcessMethod::InitFromJsonString(
    const std::string &config) {
  VLOG(EXAMPLE_DEBUG) << "PTQCenternetPostProcessMethod Json string:"
                      << config.data();

  char *latency_log = getenv("SHOW_LATENCY_LOG");
  latency_status_ = (latency_log != nullptr) ? atoi(latency_log) : 0;

  rapidjson::Document document;
  document.Parse(config.data());

  if (document.HasParseError()) {
    VLOG(EXAMPLE_SYSTEM) << "Parsing config file failed";
    return -1;
  }
  if (document.HasMember("class_num")) {
    centernet_config_.class_num = document["class_num"].GetInt();
  }

  if (document.HasMember("score_threshold")) {
    score_threshold_ = document["score_threshold"].GetFloat();
  }

  if (document.HasMember("topk")) {
    top_k_ = document["topk"].GetInt();
  }

  if (document.HasMember("det_name_list")) {
    std::string det_name_list = document["det_name_list"].GetString();
    if (!det_name_list.empty()) {
      std::ifstream ifs;
      ifs.open(det_name_list);
      if (!ifs.is_open()) {
        VLOG(EXAMPLE_SYSTEM) << "Open " << det_name_list << " failed!";
        return -1;
      }
      std::string line;
      centernet_config_.class_names.clear();
      while (std::getline(ifs, line)) {
        centernet_config_.class_names.push_back(line);
      }
    }
  }

  VLOG(EXAMPLE_DEBUG) << "score_threshold: " << score_threshold_
                      << ", topk: " << top_k_;
  return 0;
}

std::vector<xstream::BaseDataPtr> PTQCenternetPostProcessMethod::DoProcess(
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
      << "PTQCenternetPostProcessMethod DoProcess finished, predict result: "
      << *(perception.get());
  return res;
}

void PTQCenternetPostProcessMethod::Finalize() {}

int PTQCenternetPostProcessMethod::UpdateParameter(xstream::InputParamPtr ptr) {
  VLOG(EXAMPLE_REPORT)
      << "UpdateParameter in PTQCenternetPostProcessMethod not supported yet.";
  return 0;
}

xstream::InputParamPtr PTQCenternetPostProcessMethod::GetParameter() const {
  VLOG(EXAMPLE_DETAIL) << "PTQCenternetPostProcessMethod GetParameter";
  xstream::InputParamPtr ptr;
  return ptr;
}

std::string PTQCenternetPostProcessMethod::GetVersion() const { return ""; }

xstream::MethodInfo PTQCenternetPostProcessMethod::GetMethodInfo() {
  VLOG(EXAMPLE_DETAIL) << "PTQCenternetPostProcessMethod GetMethodInfo";
  xstream::MethodInfo order_method = xstream::MethodInfo();
  order_method.is_thread_safe_ = true;
  order_method.is_need_reorder_ = false;
  order_method.is_src_ctx_dept_ = false;
  return order_method;
}

PTQCenternetPostProcessMethod::~PTQCenternetPostProcessMethod() = default;

int PTQCenternetPostProcessMethod::PostProcess(
    std::vector<hbDNNTensor> &tensors,
    ImageTensor *image_tensor,
    Perception *perception) {
  perception->type = Perception::DET;
  for (int i = 0; i < tensors.size(); i++) {
    hbSysFlushMem(&(tensors[i].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  }
  // index记录nms后有效数据的下标
  std::vector<int> index;
  nms_max_pool2d(tensors[0], index);
  // 根据下标缩小参与top_k计算的数据量，然后top_k选出前一百进行排序，保存至decode中
  std::vector<DecodeData> decode(top_k_);
  topk(tensors[0], top_k_, decode, index);
  // 对前一百数据进行sigmoid
  sigmoid(decode);
  // 获取wh数据，用于计算框的宽高
  float *wh = reinterpret_cast<float *>(tensors[1].sysMem[0].virAddr);
  // 根据索引，获取reg
  float *reg = reinterpret_cast<float *>(tensors[2].sysMem[0].virAddr);
  // nchw获取面积128x128
  int area = tensors[2].properties.validShape.dimensionSize[2] *
             tensors[2].properties.validShape.dimensionSize[3];
  std::vector<float> reg_x(top_k_);
  std::vector<float> reg_y(top_k_);

  std::vector<Detection> tmp_box(top_k_);
  // 将检测框还原到128x128上，根据下标索得到相应reg和wh偏移量
  for (int i = 0; i < top_k_; i++) {
    if (decode[i].topk_score <= score_threshold_) {
      continue;
    }
    reg_x[i] = reg[decode[i].topk_inds];
    decode[i].topk_xs += reg_x[i];
    reg_y[i] = reg[area + decode[i].topk_inds];
    decode[i].topk_ys += reg_y[i];

    float x_min = decode[i].topk_xs - wh[decode[i].topk_inds] / 2;
    float x_max = decode[i].topk_xs + wh[decode[i].topk_inds] / 2;
    float y_min = decode[i].topk_ys - wh[area + decode[i].topk_inds] / 2;
    float y_max = decode[i].topk_ys + wh[area + decode[i].topk_inds] / 2;
    tmp_box[i].bbox.xmin = x_min;
    tmp_box[i].bbox.xmax = x_max;
    tmp_box[i].bbox.ymin = y_min;
    tmp_box[i].bbox.ymax = y_max;
    tmp_box[i].score = decode[i].topk_score;
    tmp_box[i].id = decode[i].topk_clses;
    tmp_box[i].class_name =
        centernet_config_.class_names[decode[i].topk_clses].c_str();
    perception->det.push_back(tmp_box[i]);
  }

  // box mapping to original image
  // 取长边，得到缩放比例
  int origin_height = image_tensor->ori_image_height;
  int origin_width = image_tensor->ori_image_width;
  float scale_x = 1.0;
  float scale_y = 1.0;
  float offset_x = 0.0;
  float offset_y = 0.0;
  if (image_tensor->is_pad_resize) {
    float pad_len = origin_height > origin_width ? origin_height : origin_width;
    // 模型输出尺寸是128x128
    scale_x = pad_len / 128.0;
    scale_y = pad_len / 128.0;
    // 获得非等比缩放偏移量，长边放缩，短边需要减|w-h|/2
    offset_x = (pad_len - origin_width) / 2;
    offset_y = (pad_len - origin_height) / 2;
  } else {
    scale_x = origin_width / 128.0;
    scale_y = origin_height / 128.0;
  }
  auto &detections = perception->det;
  int det_num = perception->det.size();
  for (int i = 0; i < det_num; i++) {
    detections[i].bbox.xmin = detections[i].bbox.xmin * scale_x - offset_x;
    detections[i].bbox.xmax = detections[i].bbox.xmax * scale_x - offset_x;
    detections[i].bbox.ymin = detections[i].bbox.ymin * scale_y - offset_y;
    detections[i].bbox.ymax = detections[i].bbox.ymax * scale_y - offset_y;
  }
  return 0;
}
