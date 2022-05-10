// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_PTQ_CENTERNET_POST_PROCESS_METHOD_H_
#define _METHOD_PTQ_CENTERNET_POST_PROCESS_METHOD_H_

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "method/method_data.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

/**
 * Config definition for Centernet
 */
struct PTQCenternetConfig {
  int class_num;
  std::vector<std::string> class_names;
};

extern PTQCenternetConfig default_ptq_centernet_config;
/**
 * Method for post processing
 */
class PTQCenternetPostProcessMethod : public xstream::SimpleMethod {
 public:
  /**
   * Init post process from file
   * @param[in] config_file_path: config file path
   *        config file should be in the json format
   *        for example:
   *        {
   *          "topk": 100,
   *          "score_threshold": 0.4
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

  ~PTQCenternetPostProcessMethod() override;

 private:
  int PostProcess(std::vector<hbDNNTensor> &tensors,
                  ImageTensor *image_tensor,
                  Perception *perception);

 private:
  std::string output_name_ = "perception_data";
  PTQCenternetConfig centernet_config_ = default_ptq_centernet_config;
  float score_threshold_ = 0.1;
  int top_k_ = 100;
  int latency_status_ = 0;
};

#endif  // _METHOD_PTQ_CENTERNET_POST_PROCESS_METHOD_H_
