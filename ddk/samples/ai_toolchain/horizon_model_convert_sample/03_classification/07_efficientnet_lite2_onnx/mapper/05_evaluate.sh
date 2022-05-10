#!/bin/bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

set -v -e
cd $(dirname $0) || exit

#for converted quanti model evaluation
quanti_model_file="./model_output/efficientnet_lite2_260x260_nv12_quantized_model.onnx"
quanti_input_layout="NHWC"

original_model_file="./model_output/efficientnet_lite2_260x260_nv12_original_float_model.onnx"
original_input_layout="NHWC"

if [[ $1 =~ "origin" ]];  then
  model=$original_model_file
  layout=$original_input_layout
  input_offset=128
else
  model=$quanti_model_file
  layout=$quanti_input_layout
  input_offset=128  
fi

imagenet_data_path="../../../01_common/data/imagenet/val"
if [ -z $2 ]; then 
  imagenet_label_path="../../../01_common/data/imagenet/val.txt"
else
  imagenet_label_path="../../../01_common/test_data/val_piece.txt"
fi

# -------------------------------------------------------------------------------------------------------------
# shell command "sh 05_evaluate.sh" runs quanti full evaluation by default 
# If quanti model eval is intended, please run the shell via command "sh 05_evaluate.sh quanti"
# If float  model eval is intended, please run the shell via command "sh 05_evaluate.sh origin"#
# If quanti model quick eval test is intended, please run the shell via command "sh 05_evaluate.sh quanti test"
# If float  model quick eval test is intended, please run the shell via command "sh 05_evaluate.sh origin test"
# -------------------------------------------------------------------------------------------------------------

python3 -u ../../cls_evaluate.py \
        --model ${model} \
        --image_path ${imagenet_data_path} \
        --label_path ${imagenet_label_path} \
        --input_layout ${layout}  \
        --input_offset ${input_offset}
