# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import os
import sys
sys.path.append('.')

from absl import flags
from absl import logging
from absl import app

import numpy as np
from data_transformer import data_transformer
import skimage.io

FLAGS = flags.FLAGS
flags.DEFINE_string('src_dir', default=None, help='Input image file.')
flags.DEFINE_string('dst_dir', default=None, help='Output bgr dir.')

transformers = data_transformer()


def read_image(src_file):
    image = skimage.img_as_float(skimage.io.imread(src_file)).astype(
        np.float32)
    if image.ndim != 3:  # expend gray scale image to three channels
        image = image[..., np.newaxis]
        image = np.concatenate([image, image, image], axis=-1)
    return image


def create_preprocessed_file(src_file, transformers, dst_dir):
    image = [read_image(src_file)]
    for trans in transformers:
        image = trans(image)
    filename = os.path.basename(src_file)
    short_name, ext = os.path.splitext(filename)
    bgr_name = os.path.join(dst_dir, short_name + '.bgr')
    logging.info("write:%s" % bgr_name)
    image[0].astype(np.uint8).tofile(bgr_name)


def main(_):
    src_dir = FLAGS.src_dir
    dst_dir = FLAGS.dst_dir
    os.makedirs(dst_dir, exist_ok=True)
    for src_name in os.listdir(src_dir):
        src_file = os.path.join(src_dir, src_name)
        create_preprocessed_file(src_file, transformers, dst_dir)


if __name__ == "__main__":
    logging.set_verbosity(logging.INFO)
    app.run(main)
