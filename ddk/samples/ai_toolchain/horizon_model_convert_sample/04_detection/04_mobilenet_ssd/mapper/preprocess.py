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
        1、resize to 300 * 300
    """
    transformers = [
        ResizeTransformer(target_size=(300, 300), mode='opencv', method=1),
    ]
    return transformers


def infer_transformers(input_shape, input_layout="NHWC"):
    """
    step：
        1、resize to 300 * 300
        2、bgr to nv12
        3、nv12 to yuv444
    :param input_shape: input shape(target size)
    :param input_layout: NCHW / NHWC
    """
    transformers = [
        ResizeTransformer(target_size=(300, 300), mode='opencv', method=1),
        BGR2NV12Transformer(data_format="HWC"),
        NV12ToYUV444Transformer(target_size=(300, 300),
                                yuv444_output_layout=input_layout[1:]),
    ]
    return transformers


def infer_image_preprocess(image_file, input_layout, input_shape):
    """
    image for single image inference
    note: imread_mode [skimage / opencv]
        opencv read image as 8-bit unsigned integers BGR in range [0, 255]
        skimage read image as float32 RGB in range [0, 1]
        make sure to use the same imread_mode as the model training
    :param image_file: image file
    :param input_layout: NCHW / NHWC
    :param input_shape: input shape（target size）
    :return: origin image, processed image (uint8, 0-255)
    """
    transformers = infer_transformers(input_shape, input_layout)
    origin_image, processed_image = SingleImageDataLoaderWithOrigin(
        transformers, image_file, imread_mode="opencv")
    return origin_image, processed_image


def eval_image_preprocess(image_path, annotation_path, val_txt_path,
                          input_shape, input_layout):
    """
    image for full scale evaluation
    note: imread_mode [skimage / opencv]
        opencv read image as 8-bit unsigned integers BGR in range [0, 255]
        skimage read image as float32 RGB in range [0, 1]
        make sure to use the same imread_mode as the model training
    :param image_path: image path
    :param annotation_path: annotation path
    :param val_txt_path: val txt path
    :param input_shape: input shape（target size）
    :param input_layout: NCHW / NHWC
    :return: data loader, evaluation image number
    """
    transformers = infer_transformers(input_shape, input_layout)
    data_loader = VOCDataLoader(transformers,
                                image_path,
                                annotation_path,
                                val_txt_path,
                                imread_mode='opencv',
                                batch_size=1)
    total_image_number = sum(1 for line in open(val_txt_path))
    return data_loader, total_image_number
