# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

rm -rf data
mkdir data
cd data
wget ftp://vrftp.horizon.ai//Open_Explorer/qat_dataset/imagenet/imagenet_1k.zip
unzip -q imagenet_1k.zip
cd ..
python3 mobilenet_example.py
