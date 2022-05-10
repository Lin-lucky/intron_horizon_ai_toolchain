#!/usr/bin/env sh
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

set -ex
cd $(dirname $0) || exit

model_type="caffe"
proto="../../../01_common/model_zoo/mapper/other/lenet/lenet_train_test.prototxt"
caffe_model="../../../01_common/model_zoo/mapper/other/lenet/lenet_iter_100000.caffemodel"
output="./lenet_gray_checker.log"
march="bernoulli2"

hb_mapper checker --model-type ${model_type} \
                  --proto ${proto} --model ${caffe_model} \
                  --output ${output} --march ${march}
