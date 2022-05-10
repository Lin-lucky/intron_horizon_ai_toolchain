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


def data_transformer():
    transformers = [
        ShortSideResizeTransformer(short_size=256),
        CenterCropTransformer(crop_size=224),
        HWC2CHWTransformer(),
        RGB2BGRTransformer(data_format="CHW"),
        ScaleTransformer(scale_value=255),
    ]
    return transformers


dp = EasyDict()
dp.data_transformer = data_transformer
