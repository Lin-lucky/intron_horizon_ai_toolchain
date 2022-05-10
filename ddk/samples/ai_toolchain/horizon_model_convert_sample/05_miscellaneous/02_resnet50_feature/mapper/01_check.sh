#!/usr/bin/env sh
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

cd $(dirname $0) || exit
set -e

model_type="caffe"
proto="../../../01_common/model_zoo/mapper/other/resnet50_feature/resnet50_feature_deploy.prototxt"
caffe_model="../../../01_common/model_zoo/mapper/other/resnet50_feature/resnet50_feature.caffemodel"
model_type="caffe"
output="./resnet50_feature_checker.log"
march="bernoulli2"

hb_mapper checker --model-type ${model_type} \
                  --proto ${proto} --model ${caffe_model} \
                  --output ${output} --march ${march}
