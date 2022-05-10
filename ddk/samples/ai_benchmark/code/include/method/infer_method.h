// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_PREDICT_METHOD_H_
#define _METHOD_PREDICT_METHOD_H_

#include <string>
#include <vector>

#include "base/common_def.h"
#include "dnn/hb_dnn.h"
#include "glog/logging.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

#define LATENCY_LOG "SHOW_LATENCY_LOG"

/**
 * Method for prediction
 */
class InferMethod : public xstream::SimpleMethod {
 public:
  /**
   * Init predict method from config file
   * @param[in] config_file: config file path
   *    config file should be in the json format
   *    for example:
   *    {
   *        "core": 1 # 1 for single core, 2 for dual core
   *        "model_file": ".bin",
   *        "output_name": "tensors"
   *    }
   * @return 0 if success
   */
  int Init(const std::string &config_file_path) override;

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

  ~InferMethod();

 private:
  std::string output_name_ = "tensors";
  hbPackedDNNHandle_t packed_dnn_handle_ = nullptr;
  int core_ = 0;
  int32_t latency_status_ = 0;
  Latency latency_;
  hbDNNHandle_t dnn_handle_;
};

#endif  // _METHOD_PREDICT_METHOD_H_
