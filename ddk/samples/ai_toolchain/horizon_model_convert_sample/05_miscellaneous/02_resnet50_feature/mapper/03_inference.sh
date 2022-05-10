#!/bin/bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

cd $(dirname $0) || exit
set -ex

model_file="./model_output/resnet50_64x56x56_featuremap_quantized_model.onnx"
feature_file="../../../01_common/calibration_data/cat_cls"

python3 inference.py \
        --model ${model_file} \
        --feature ${feature_file}
