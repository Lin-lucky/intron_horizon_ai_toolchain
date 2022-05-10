# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

bin=../aarch64/bin/run_custom_op
lib=../aarch64/lib

export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
export BMEM_CACHEABLE=true

${bin} \
  --model_file=../../model/runtime/googlenet_cop/googlenet_cop_224x224_nv12.bin \
  --image_file=../../data/cls_images/zebra_cls.jpg \
  --image_height=224 \
  --image_width=224 \
  --top_k=5
