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




#for converted quanti model inference
quanti_model_file="./model_output/mobilenetv1_224x224_yuv444_quantized_model.onnx"
quanti_input_layout="NHWC"

#for original float model inference
original_model_file="./model_output/mobilenetv1_224x224_yuv444_original_float_model.onnx"
original_input_layout="NCHW"

if [[ $1 =~ "origin" ]];  then
  model=$original_model_file
  layout=$original_input_layout
  input_offset=128
else
  model=$quanti_model_file
  layout=$quanti_input_layout
  input_offset=128
fi

infer_image="../../../01_common/test_data/cls_images/zebra_cls.jpg"


# -----------------------------------------------------------------------------------------------------
# shell command "sh 04_inference.sh" runs quanti inference by default 
# If quanti model infer is intended, please run the shell via command "sh 04_inference.sh quanti"
# If float  model infer is intended, please run the shell via command "sh 04_inference.sh origin"
# -----------------------------------------------------------------------------------------------------

python3 -u ../../cls_inference.py \
        --model ${model} \
        --image ${infer_image} \
        --input_layout ${layout} \
        --input_offset ${input_offset}
