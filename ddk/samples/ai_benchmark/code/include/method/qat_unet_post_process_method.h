// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_QAT_UNET_POST_PROCESS_METHOD_H_
#define _METHOD_QAT_UNET_POST_PROCESS_METHOD_H_

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "method/method_data.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

/**
 * Method for post processing
 */
class QATUnetPostProcessMethod : public xstream::SimpleMethod {
 public:
  /**
   * Load configuration from file
   * @param[in] config_file: config file path
   *    Config file should be json format
   *    for example:
   *    {
   *        "num_classes" :20
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

  ~QATUnetPostProcessMethod() override;

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

 private:
  int num_classes_ = 20;
  std::string output_name_ = "perception_data";
};

#endif  // _METHOD_QAT_UNET_POST_PROCESS_METHOD_H_
