#!/usr/bin/env sh
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

set -e -v
cd $(dirname $0)

model_type="onnx"
onnx_model="../../../01_common/model_zoo/mapper/detection/fcos/fcos.onnx"
output="./fcos_checker.log"
march="bernoulli2"

hb_mapper checker --model-type ${model_type} \
                  --model ${onnx_model} \
                  --output ${output} --march ${march}