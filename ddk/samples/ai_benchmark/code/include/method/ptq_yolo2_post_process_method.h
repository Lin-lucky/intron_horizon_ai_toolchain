// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_PTQ_YOLO2_POST_PROCESS_METHOD_H_
#define _METHOD_PTQ_YOLO2_POST_PROCESS_METHOD_H_

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "method/method_data.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

/**
 * Config definition for Yolo2
 */
struct PTQYolo2Config {
  int stride;
  std::vector<std::pair<double, double>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;

  std::string Str() {
    std::stringstream ss;
    ss << "stride: " << stride;
    ss << "; anchors_table: ";
    for (auto anchors : anchors_table) {
      ss << "[" << anchors.first << "," << anchors.second << "] ";
    }
    ss << "; class_num: " << class_num;
    return ss.str();
  }
};

extern PTQYolo2Config default_ptq_yolo2_config;
/**
 * Method for post processing
 */
class PTQYolo2PostProcessMethod : public xstream::SimpleMethod {
 public:
  /**
   * Init post process from file
   * @param[in] config_file_path: config file path
   *        config file should be in the json format
   *        for example:
   *        {
   *          "top_k": 1,
   *          "cls_names_list":
   * "../../config/model/data_name_list/imagenet.list"
   *        }
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

  ~PTQYolo2PostProcessMethod() override;

 private:
  int PostProcess(std::vector<hbDNNTensor> &tensors,
                  ImageTensor *image_tensor,
                  Perception *perception);

 private:
  std::string output_name_ = "perception_data";
  PTQYolo2Config yolo2_config_ = default_ptq_yolo2_config;
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 500;
  int latency_status_ = 0;
};

#endif  // _METHOD_PTQ_YOLO2_POST_PROCESS_METHOD_H_
