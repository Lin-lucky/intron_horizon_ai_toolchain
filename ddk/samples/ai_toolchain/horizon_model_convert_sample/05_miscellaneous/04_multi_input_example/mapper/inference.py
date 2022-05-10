# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import os
import click
import logging
import numpy as np
from horizon_tc_ui import HB_ONNXRuntime
from horizon_tc_ui.utils.tool_utils import init_root_logger

import sys
sys.path.append("../../../01_common/python/data/")
from transformer import *
from horizon_nn.data.data_loader import ImageLoader

from data_transformer import data_transformer


def setup_dp(path):
    """ setup config """
    dp_file = path
    with open(dp_file) as f:
        code = compile(f.read(), dp_file, "exec")
        local = {}
        exec(code, globals(), local)
        return local["dp"]


def load_image(image_file, input_shape):
    transformers = data_transformer()
    image = ImageLoader(transformers, image_file)
    return image


def infer(onnx_model_file, data, input_offset):
    sess = HB_ONNXRuntime(model_file=onnx_model_file)

    image_b = data[:, 0, ...][:, np.newaxis, ...].transpose([0, 2, 3, 1])
    image_g = data[:, 1, ...][:, np.newaxis, ...].transpose([0, 2, 3, 1])
    image_r = data[:, 2, ...][:, np.newaxis, ...].transpose([0, 2, 3, 1])

    sess.set_providers(['CPUExecutionProvider'])
    input_name = []
    for model_input in sess.get_inputs():
        input_name.append(model_input.name)
    output_name = sess.get_outputs()[0].name
    feed_dict = {
        input_name[0]: image_b,
        input_name[1]: image_g,
        input_name[2]: image_r
    }
    output = sess.run([output_name], feed_dict,
                      input_offset=input_offset)
    prob = np.squeeze(output[0])
    idx = np.argsort(-prob)

    print("The input picture is classified to be:")
    for i in range(5):
        label = idx[i]
        print('  label %d: prob %.2f' % (label, prob[label]))


@click.version_option(version="1.0.0")
@click.command()
@click.option('-m', '--model', type=str, help='Input onnx model(.onnx) file')
@click.option('-i', '--image', type=str, help='Input image file.')
@click.option('-y',
              '--input_layout',
              type=str,
              default="",
              help='Model input layout')
@click.option('-o',
              '--input_offset',
              type=str,
              default=0,
              help='input inference offset.')
@click.option('-c',
              '--color_sequence',
              type=str,
              default=None,
              help='Color sequence')
def main(model, image, input_layout, input_offset, color_sequence):
    init_root_logger("inference.log",
                     console_level=logging.INFO,
                     file_level=logging.DEBUG)
    if color_sequence:
        logging.warning("option color_sequence is deprecated.")
    input_shape = (224, 224)
    image_data = load_image(image, input_shape)
    infer(model, image_data, input_offset)


if __name__ == '__main__':
    main()
