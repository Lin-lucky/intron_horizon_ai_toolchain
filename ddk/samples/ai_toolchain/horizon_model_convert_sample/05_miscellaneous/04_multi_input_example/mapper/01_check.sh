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

model_type="caffe"
proto="../../../01_common/model_zoo/mapper/other/mobilenetv2_three_inputs/mobilenetv2_three_inputs.prototxt"
caffe_model="../../../01_common/model_zoo/mapper/other/mobilenetv2_three_inputs/mobilenetv2_three_inputs.caffemodel"
output="./mobilenet_multi_checker.log"
input_name1=image_r
input_name2=image_g
input_name3=image_b
march="bernoulli2"

hb_mapper checker --model-type ${model_type} \
                  --proto ${proto} --model ${caffe_model} \
                  --input-shape ${input_name1} 1x1x224x224 \
                  --input-shape ${input_name2} 1x1x224x224 \
                  --input-shape ${input_name3} 1x1x224x224 \
                  --output ${output}  --march ${march}

                  
