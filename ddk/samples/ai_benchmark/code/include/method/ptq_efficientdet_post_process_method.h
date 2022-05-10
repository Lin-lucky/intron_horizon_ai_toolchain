// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef _METHOD_PTQ_EFFICIENTDET_POST_PROCESS_METHOD_H_
#define _METHOD_PTQ_EFFICIENTDET_POST_PROCESS_METHOD_H_

#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "method/method_data.h"
#include "xstream/simple_method.h"
#include "xstream/xstream_data.h"

/**
 * Config definition for EfficientDet
 */
struct EfficientDetConfig {
  std::vector<std::vector<double>> anchor_scales;
  std::vector<double> anchor_ratio;
  std::vector<int> feature_strides;
  int class_num;
  std::vector<std::string> class_names;
  std::vector<std::vector<float>> scales;
};

struct EDBaseAnchor {
  EDBaseAnchor(float x1, float y1, float x2, float y2)
      : x1_(x1), y1_(y1), x2_(x2), y2_(y2) {}
  float x1_;
  float y1_;
  float x2_;
  float y2_;
};

struct EDAnchor {
  EDAnchor(float c_x, float c_y, float w, float h)
      : c_x_(c_x), c_y_(c_y), w_(w), h_(h) {}
  float c_x_;
  float c_y_;
  float w_;
  float h_;
};

extern EfficientDetConfig default_efficient_det_config;

/**
 * Method for post processing
 */
class PTQEfficientDetPostProcessMethod : public xstream::SimpleMethod {
 public:
  // TODO(ruxin.song): config file information
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

  ~PTQEfficientDetPostProcessMethod() override;

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

  int GetAnchors(std::vector<EDAnchor> &anchors,
                 int layer,
                 int feat_height,
                 int feat_width);

  int GetBboxAndScores(hbDNNTensor *c_tensor,
                       hbDNNTensor *bbox_tensor,
                       std::vector<Detection> &dets,
                       std::vector<EDAnchor> &anchors,
                       int class_num,
                       float img_h,
                       float img_w,
                       int tensor_index);

 private:
  std::string output_name_ = "perception_data";
  EfficientDetConfig efficient_det_config_ = default_efficient_det_config;
  float score_threshold_ = 0.05;
  float nms_threshold_ = 0.5;
  int nms_top_k_ = 100;
  bool anchor_init_ = false;
  std::vector<std::vector<EDAnchor>> anchors_table_;
  int latency_status_ = 0;
  std::string dequanti_file_ = "";
  bool has_dequanti_node_ = true;
};

#endif  // _METHOD_PTQ_EFFICIENTDET_POST_PROCESS_METHOD_H_
