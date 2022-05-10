// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef ADVANCED_SAMPLES_CUSTOM_IDENTITY_H_
#define ADVANCED_SAMPLES_CUSTOM_IDENTITY_H_

#include <string>
#include <vector>

#include "dnn/hb_dnn.h"
#include "dnn/plugin/hb_dnn_layer.h"
#include "dnn/plugin/hb_dnn_ndarray.h"

namespace hobot {
namespace dnn {

Layer *CustomIdentity_layer_creator();

class CustomIdentity : public Layer {
 public:
  CustomIdentity() = default;
  ~CustomIdentity() override = default;

 public:
  int32_t Init(const Attribute &attributes) override;

  int32_t Forward(const std::vector<NDArray *> &bottomBlobs,
                  std::vector<NDArray *> &topBlobs,
                  const hbDNNInferCtrlParam *inferCtrlParam) override;

  std::string GetType() const override { return "CustomIdentity"; }

 private:
  std::string module_;
};

}  // namespace dnn
}  // namespace hobot

#endif
