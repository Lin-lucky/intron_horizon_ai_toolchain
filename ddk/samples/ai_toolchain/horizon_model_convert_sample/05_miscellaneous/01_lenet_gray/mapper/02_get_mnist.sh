#!/usr/bin/env bash
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

# !!! 脚本会下载mnist数据集。如果您没有网络，请手动下载之后拷贝到当前文件夹下。并使用gunzip解压。


cd $(dirname $0) || exit

python3 process_mnist.py ../../../01_common/calibration_data/t10k-images-idx3-ubyte
