// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_PTQ_SSD_POST_PROCESS_METHOD_H_
#define _METHOD_PTQ_SSD_POST_PROCESS_METHOD_H_

#include <string>
#include <utility>
#include <vector>

#include "base/perception_common.h"
#include "glog/logging.h"
#include "method/method_data.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

/**
 * Config definition for SSD
 */
struct SSDConfig {
  std::vector<float> std;
  std::vector<float> mean;
  std::vector<float> offset;
  std::vector<int> step;
  std::vector<std::pair<float, float>> anchor_size;
  std::vector<std::vector<float>> anchor_ratio;
  int class_num;
  std::vector<std::string> class_names;
};

/**
 * Default ssd config
 * std: [0.1, 0.1, 0.2, 0.2]
 * mean: [0, 0, 0, 0]
 * offset: [0.5, 0.5]
 * step: [8, 16, 32, 64, 100, 300]
 * anchor_size: [[30, 60], [60, 111], [111, 162], [162, 213], [213, 264],
 *              [264,315]]
 * anchor_ratio: [[2, 0.5, 0, 0], [2, 0.5, 3, 1.0 / 3],
 *              [2, 0.5,3, 1.0 / 3], [2, 0.5, 3, 1.0 / 3],
 *              [2, 0.5, 0, 0], [2,0.5, 0, 0]]
 * class_num: 20
 * class_names: ["aeroplane",   "bicycle", "bird",  "boaupdate", "bottle",
     "bus",         "car",     "cat",   "chair",     "cow",
     "diningtable", "dog",     "horse", "motorbike", "person",
     "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"]
 */
extern SSDConfig default_ssd_config;

/**
 * Method for post processing
 */
class PTQSSDPostProcessMethod : public xstream::SimpleMethod {
 public:
  /**
   * Load configuration from file
   * @param[in] config_file: config file path
   *    Config file should be json format
   *    for example:
   *    {
   *        "score_threshold": 0.2,
   *        "nms_threshold": 0.2,
   *        "ssd": {
   *            "std": ...
   *            "mean": ...
   *            "offset": ...
   *            "step": ...
   *            ...
   *        }
   *    }
   * @param[in] config_string: config string
   * @param[in] properties: model output tensor properties
   * @return 0 if success
   */
  int Init(const std::string &config_file_path) override;

  /**
   * Init post process from json string
   * @param[in] config: config json string
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

  //  void OnProfilerChanged(bool on) override{};

  xstream::MethodInfo GetMethodInfo() override;

  ~PTQSSDPostProcessMethod() override;

 private:
  /**
   * Post process
   * @param[in] tensor: Model output tensors
   * @param[in] image_tensor: Input image tensor
   * @param[out] perception: Perception output data
   * @return 0 if success
   */
  int PostProcess(std::vector<hbDNNTensor> &tensors,
                  ImageTensor *image_tensor,
                  Perception *perception);

  int SsdAnchors(std::vector<Anchor> &anchors,
                 int layer,
                 int layer_height,
                 int layer_width);

  int GetBboxAndScores(hbDNNTensor *c_tensor,
                       hbDNNTensor *bbox_tensor,
                       std::vector<Detection> &dets,
                       std::vector<Anchor> &anchors,
                       int class_num,
                       float cut_off_threshold,
                       ImageTensor *frame);

 private:
  std::string output_name_ = "perception_data";
  SSDConfig ssd_config_ = default_ssd_config;
  float score_threshold_ = 0.25;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 200;
  int latency_status_ = 0;
};

#endif  // _METHOD_PTQ_SSD_POST_PROCESS_METHOD_H_
