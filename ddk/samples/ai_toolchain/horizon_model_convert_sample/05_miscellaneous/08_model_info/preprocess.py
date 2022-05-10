# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

from easydict import EasyDict
import sys
sys.path.append("../../01_common/python/data/")
from transformer import *


def calibration_transformers():
    """
    step：
        1、short size resize to 256
        2、crop size 224*224 from center
        3、NHWC to NCHW
        4、rgb to bgr
        5、Scale from 0 ~ 1 to 0 ~ 255
    """
    transformers = [
        ShortSideResizeTransformer(short_size=256),
        CenterCropTransformer(crop_size=224),
        HWC2CHWTransformer(),
        RGB2BGRTransformer(data_format="CHW"),
        ScaleTransformer(scale_value=255),
    ]
    return transformers
