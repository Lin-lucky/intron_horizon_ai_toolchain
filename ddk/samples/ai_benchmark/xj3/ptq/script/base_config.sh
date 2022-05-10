#!/bin/sh
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.


# define bin and lib info
app=../../aarch64/bin/example
lib=../../aarch64/lib
export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}

# maintain CPU/BPU frequency
echo 105000 >/sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
# set bpu0/1 governor, will be fix next image version(use one policy with 2 core)
echo performance > /sys/class/devfreq/devfreq1/governor
echo performance > /sys/class/devfreq/devfreq2/governor
