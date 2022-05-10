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
import cv2


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
    image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
    image = image.reshape(1, 1, 28, 28)
    if input_layout == "NHWC":
        image = image.transpose([0, 2, 3, 1])

    return image
