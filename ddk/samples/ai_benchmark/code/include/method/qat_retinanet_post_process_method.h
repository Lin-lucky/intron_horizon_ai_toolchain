// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_QAT_RETINANET_POST_PROCESS_METHOD_H_
#define _METHOD_QAT_RETINANET_POST_PROCESS_METHOD_H_

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "method/method_data.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

/**
 * Config definition for Retinanet
 */
struct QATRetinanetConfig {
  std::vector<std::vector<float>> anchor_sizes;
  std::vector<std::vector<float>> anchor_ratios;
  int class_num;
  std::vector<std::string> class_names;
  std::vector<std::vector<std::vector<int>>> base_anchors;
};

struct ClassScore {
  int h;
  int w;
  int anchor;
  double scores[80];
};

/**
 * Default retinanet config
 * anchor_sizes: [[32, 40.31747359663594, 50.79683366298238],
 *                [64, 80.63494719327188, 101.59366732596476],
 *                [128, 161.26989438654377, 203.18733465192952],
 *                [256, 322.53978877308754, 406.37466930385904],
 *                [512, 645.0795775461751, 812.7493386077181]]
 * anchor_ratios: [[0.5, 1, 2], [0.5, 1, 2],
 *                 [0.5, 1, 2], [0.5, 1, 2],
 *                 [0.5, 1, 2]]
 * class_num: 80
 * class_names: ["person",        "bicycle",      "car",
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
     "hair drier",    "toothbrush"]
 */
extern QATRetinanetConfig default_retinanet_config;

/**
 * Method for post processing
 */
class QATRetinanetPostProcessMethod : public xstream::SimpleMethod {
 public:
  /**
   * Init post process from file
   * @param[in] config_file_path: config file path
   *        config file should be in the json format
   *        for example:
   *        {
   *          "det_name_list":
   * "../../config/model/data_name_list/voc_detlist.list", "score_threshold":
   * 0.35
   *        }
   * @return 0 if success
   */
  int Init(const std::string &config_file_path) override;

  /**
   * Init post process from json string
   * @param[in] config: config json string
   *        config should be in the json format
   *        for example:
   *        {
   *          "det_name_list":
   * "../../config/model/data_name_list/voc_detlist.list", "score_threshold":
   * 0.35
   *        }
   * @return 0 if success
   */
  int InitFromJsonString(const std::string &config) override;

  std::vector<xstream::BaseDataPtr> DoProcess(
      const std::vector<xstream::BaseDataPtr> &input,
      const xstream::InputParamPtr &param) override;

  void Finalize() override;

  int UpdateParameter(xstream::InputParamPtr ptr) override;

  xstream::InputParamPtr GetParameter() const override;

  std::string GetVersion() const override;

  xstream::MethodInfo GetMethodInfo() override;

  ~QATRetinanetPostProcessMethod() override;

 private:
  int PostProcess(std::vector<hbDNNTensor> &tensors,
                  ImageTensor *image_tensor,
                  Perception *perception);

  int RetinanetAnchors(int layer);

  int SoftmaxFromRawScore(hbDNNTensor *tensor,
                          int class_num,
                          int layer,
                          std::vector<ClassScore> &scores);

  int GetBboxFromRawData(hbDNNTensor *tensor,
                         std::vector<ClassScore> &scores,
                         int layer,
                         std::vector<Detection> &dets,
                         ImageTensor *frame);

  double Dequanti(int32_t data,
                  int layer,
                  bool big_endian,
                  int offset,
                  hbDNNTensorProperties &properties);

 private:
  std::string output_name_ = "perception_data";
  QATRetinanetConfig retinanet_config_ = default_retinanet_config;
  float score_threshold_ = 0.05;
  float nms_threshold_ = 0.5;
  int nms_top_k_ = 300;
  int image_height_ = 1024;
  int image_width_ = 1024;
};

#endif  // _METHOD_QAT_RETINANET_POST_PROCESS_METHOD_H_
