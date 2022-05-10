# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import sys
sys.path.append("../../../01_common/python/data/")
from transformer import *
from dataloader import *


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


def infer_transformers():
    """
    step：
        1、short size resize to 256
        2、crop size 224*224 from center
        3、NHWC to NCHW
        4、rgb to bgr
        5、Scale from 0 ~ 1 to 0 ~ 255
        6、bgr to nv12
        7、nv12 to yuv444
    """
    transformers = [
        ShortSideResizeTransformer(short_size=256),
        CenterCropTransformer(crop_size=224),
        HWC2CHWTransformer(),
        RGB2BGRTransformer(data_format="CHW"),
        ScaleTransformer(scale_value=255),
        BGR2NV12Transformer(data_format="CHW"),
        NV12ToYUV444Transformer((224, 224)),
    ]
    return transformers


def infer_image_preprocess(image_file, input_layout):
    """
    image for single image inference
    note: imread_mode [skimage / opencv]
        opencv read image as 8-bit unsigned integers BGR in range [0, 255]
        skimage read image as float32 RGB in range [0, 1]
        make sure to use the same imread_mode as the model training
    :param image_file: image path
    :param input_layout: NCHW / NHWC
    :return: processed image (uint8, 0-255)
    """
    transformers = infer_transformers()
    image = SingleImageDataLoader(transformers, image_file)
    if input_layout == "NCHW":
        image = image.transpose([0, 3, 1, 2])
    return image
