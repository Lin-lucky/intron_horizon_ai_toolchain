#!/usr/bin/env bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

cd $(dirname $0)
set -ve

#====================== config area ==================================
output_dir="output"
#===================================================================

rm -rf $output_dir/vec_diff_*.csv

float_output_path=$output_dir/float_dump
quantized_output_path=$output_dir/quantized_dump
hybrid_output_path=$output_dir/hybrid_dump 

vec_diff $float_output_path $quantized_output_path -o $output_dir/vec_diff_float_quantized_result.csv
vec_diff $hybrid_output_path $quantized_output_path -o $output_dir/vec_diff_hybrid_quantized_result.csv
vec_diff $hybrid_output_path $float_output_path -o $output_dir/vec_diff_hybrid_float_result.csv
