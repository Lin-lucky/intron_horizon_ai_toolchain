#!/bin/bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

set -e -v

cd $(dirname $0) || exit

config_file_bgr="./mobilenet_config_bgr.yaml"
config_file_yuv444="./mobilenet_config_yuv444.yaml"
config_file_feature="./resnet50_featuremap_config.yaml"
model_type="caffe"
# build bgr model
hb_mapper makertbin --config ${config_file_bgr}  \
                    --model-type  ${model_type}
          
# build yuv444 model
hb_mapper makertbin --config ${config_file_yuv444}  \
                    --model-type  ${model_type}
          
# build feature model
hb_mapper makertbin --config ${config_file_feature}  \
                    --model-type  ${model_type}
