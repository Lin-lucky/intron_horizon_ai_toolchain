// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_QAT_YOLO3_POST_PROCESS_METHOD_H_
#define _METHOD_QAT_YOLO3_POST_PROCESS_METHOD_H_

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "method/method_data.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

/**
 * Config definition for Yolo3
 */
struct QATYolo3Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;

  std::string Str() {
    std::stringstream ss;
    ss << "strides: ";
    for (const auto &stride : strides) {
      ss << stride << " ";
    }

    ss << "; anchors_table: ";
    for (const auto &anchors : anchors_table) {
      for (auto data : anchors) {
        ss << "[" << data.first << "," << data.second << "] ";
      }
    }
    ss << "; class_num: " << class_num;
    return ss.str();
  }
};

extern QATYolo3Config default_qat_yolo3_config;
/**
 * Method for post processing
 */
class QATYolo3PostProcessMethod : public xstream::SimpleMethod {
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

  ~QATYolo3PostProcessMethod() override;

 private:
  int PostProcess(std::vector<hbDNNTensor> &tensors,
                  ImageTensor *image_tensor,
                  Perception *perception);

  void PostProcess(hbDNNTensor *tensor,
                   ImageTensor *frame,
                   int layer,
                   std::vector<Detection> &dets);

  int YoloAux(hbDNNTensor *tensor,
              ImageTensor *image_tensor,
              Perception *perception);

  float Dequanti(int32_t data,
                 int layer,
                 bool big_endian,
                 int offset,
                 hbDNNTensorProperties &properties);

 private:
  std::string output_name_ = "perception_data";
  float score_threshold_ = 0.01;
  // reduce accuracy to improve performance
  bool is_performance_ = false;
  float nms_threshold_ = 0.45;
  int nms_top_k_ = 200;
  int performance_nms_topk_ = 100;
  QATYolo3Config yolo3_config_ = default_qat_yolo3_config;
  std::vector<std::string> det_names_;  // category name list of voc
};

#endif  // _METHOD_QAT_YOLO3_POST_PROCESS_METHOD_H_
