#!/usr/bin/env bash
## Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
##
## The material in this file is confidential and contains trade secrets
## of Horizon Robotics Inc. This is proprietary information owned by
## Horizon Robotics Inc. No part of this work may be disclosed,
## reproduced, copied, transmitted, or used in any way for any purpose,
## without the express written permission of Horizon Robotics Inc.
set -x
set -e

export CC=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc
export CXX=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++


DIR=$(cd "$(dirname "$0")";pwd)


rm -rf arm_build
mkdir arm_build

cd arm_build
build_type=Release
cmake -DXJ3=XJ3 -DCMAKE_BUILD_TYPE=${build_type} -DCMAKE_INSTALL_PREFIX=../../xj3/ptq/script ${DIR}

make -j8
make install

cd ..
rm -rf arm_build
