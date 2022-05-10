#!/usr/bin/env bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

set -e -v

cd $(dirname $0) || exit

python3 ../../data_preprocess.py \
  --src_dir ../../01_common/calibration_data/imagenet \
  --dst_dir ./calibration_data_bgr
