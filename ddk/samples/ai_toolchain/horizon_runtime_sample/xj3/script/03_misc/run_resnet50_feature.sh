#!/usr/bin/env sh
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.


bin=../aarch64/bin/run_resnet_feature
lib=../aarch64/lib

export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
export BMEM_CACHEABLE=true

${bin} \
  --model_file=../../model/runtime/resnet50_feature/resnet50_64x56x56_featuremap.bin \
  --data_file=../../data/misc_data/np_0 \
  --top_k=5
