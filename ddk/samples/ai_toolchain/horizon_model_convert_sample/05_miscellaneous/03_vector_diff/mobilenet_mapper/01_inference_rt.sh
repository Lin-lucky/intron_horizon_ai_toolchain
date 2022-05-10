#!/usr/bin/env bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

echo "==================================================================="
echo -e -n "\033[91m"
echo -e ">> before running this shell, please make sure you run"
echo -e ">> model build with layer_out_dump: True in your 'config.yaml' "
echo -e -n "\033[0m"
echo "==================================================================="

cd $(dirname $0)

#====================== config area ==================================
model_name=mobilenetv1_224x224_nv12
config_file="../../../03_classification/01_mobilenet/mapper/mobilenet_config.yaml" # with layer_out_dump: True
model_output="../../../03_classification/01_mobilenet/mapper/model_output"
image_file="../../../03_classification/01_mobilenet/mapper/calibration_data_bgr_f32/ILSVRC2012_val_00000001.bgr"
input_height=224
input_width=224
input_type=2 #for input type enum definition, please see readme.txt
# only gray and yuv444 input types are supported for now
output_dir="./output"
#===================================================================

float_model_file="${model_output}/${model_name}_original_float_model.onnx"
quantized_model_file="${model_output}/${model_name}_quantized_model.onnx"
hybrid_model_file="${model_output}/${model_name}.bin"  
mapping_file="${model_output}/${model_name}_quantized_model_conv_output_map.json"
input_file_path="ILSVRC2012_val_00000001.bin"
float_output_path=$output_dir/float_dump
quantized_output_path=$output_dir/quantized_dump
hybrid_output_path=$output_dir/hybrid_dump

rm -rf $output_dir && mkdir $output_dir
rm -rf $float_output_path && rm -rf $quantized_output_path && rm -rf $hybrid_output_path
mkdir $float_output_path && mkdir $quantized_output_path && mkdir $hybrid_output_path

if [ ! -f ${mapping_file} ];then
    echo "ERROR: conv output mapping file is not exist, please make sure layer_out_dump: True"
    echo "If you are trying out the sample example, please also make sure 'layer_out_dump' is set to True in example 03_classification\01_mobilenet"
    exit
fi

export LD_LIBRARY_PATH=aarch64_x86_xj3/lib/:$LD_LIBRARY_PATH
chmod 777 aarch64_x86_xj3/bin/hrt_bin_dump

set -vex
float_model_layout="NCHW"
quanti_model_layout="NHWC"
# infer float model
hb_mapper infer --config ${config_file}  \
                --model-type caffe \
                --model-file ${float_model_file}  \
                --image-file data ${image_file} \
                --output-dir ${float_output_path}  \
                --input-layout ${float_model_layout}

# infer quantized model
hb_mapper infer --config ${config_file} \
                --model-type caffe \
                --model-file ${quantized_model_file}  \
                --image-file data ${image_file} \
                --output-dir ${quantized_output_path}  \
                --input-layout ${quanti_model_layout}

# infer hybrid model on simulator
./aarch64_x86_xj3/bin/hrt_bin_dump \
  --model_file="${hybrid_model_file}" \
  --input_file="${input_file_path}" \
  --conv_mapping_file="${mapping_file}" \
  --conv_dump_path="${hybrid_output_path}" \
