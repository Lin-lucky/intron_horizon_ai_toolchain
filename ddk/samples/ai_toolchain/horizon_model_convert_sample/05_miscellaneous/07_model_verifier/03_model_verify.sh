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

if [ -z "$1" ]; then
    echo "please input ip for on board test"
    echo "sh 03_model_verify.sh {board_ip}"
    exit 1
fi

bgr_quanti_model=bgr_model_output//mobilenet_224x224_bgr_planar_quantized_model.onnx
bgr_bin_model=bgr_model_output/mobilenet_224x224_bgr_planar.bin

yuv444_quanti_model=yuv444_model_output/mobilenet_224x224_yuv444_quantized_model.onnx
yuv444_bin_model=yuv444_model_output/mobilenet_224x224_yuv444.bin

featuremap_quanti_model=featuremap_model_output/resnet50_64x56x56_featuremap_quantized_model.onnx
featuremap_bin_model=featuremap_model_output/resnet50_64x56x56_featuremap.bin

board_ip=$1

hb_model_verifier -q ${bgr_quanti_model} -b ${bgr_bin_model} -a ${board_ip}

hb_model_verifier -q ${yuv444_quanti_model} -b ${yuv444_bin_model} -a ${board_ip}

hb_model_verifier -q ${featuremap_quanti_model} -b ${featuremap_bin_model} -a ${board_ip}
