#!/usr/bin/env sh
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.


bin=../aarch64/bin/run_lenet_gray
lib=../aarch64/lib

export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
export BMEM_CACHEABLE=true

${bin} \
  --model_file=../../model/runtime/lenet_gray/lenet_28x28_gray.bin \
  --data_file=../../data/misc_data/7.bin \
  --image_height=28 \
  --image_width=28 \
  --top_k=5
