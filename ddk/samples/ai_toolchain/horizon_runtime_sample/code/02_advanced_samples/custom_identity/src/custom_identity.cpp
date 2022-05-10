// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "custom_identity.h"

namespace hobot {
namespace dnn {

Layer *CustomIdentity_layer_creator() { return new CustomIdentity; }

int32_t CustomIdentity::Init(const Attribute &attributes) {
  // unused attribute, just demonstrating
  attributes.GetAttributeValue(&module_, "module");
  return 0;
}

int32_t CustomIdentity::Forward(const std::vector<NDArray *> &bottomBlobs,
                                std::vector<NDArray *> &topBlobs,
                                const hbDNNInferCtrlParam *inferCtrlParam) {
  const NDArray *input = bottomBlobs[0];
  NDArray *out = topBlobs[0];
  const auto *input_data = input->Dptr<float>();
  auto *out_data = out->Dptr<float>();
  uint32_t size = input->Size();

  for (uint32_t i = 0U; i < size; i++) {
    out_data[i] = input_data[i];
  }
  return 0;
}
}  // namespace dnn
}  // namespace hobot
