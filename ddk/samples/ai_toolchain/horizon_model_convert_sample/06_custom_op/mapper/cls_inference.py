# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import click
import logging

from horizon_tc_ui import HB_ONNXRuntime
from horizon_tc_ui.utils.tool_utils import init_root_logger

from preprocess import infer_image_preprocess
from postprocess import postprocess


def inference(sess, image_name, input_layout, input_offset):
    if input_layout is None:
        logging.warning(f"input_layout not provided. Using {sess.layout[0]}")
        input_layout = sess.layout[0]
    image_data = infer_image_preprocess(image_name, input_layout)
    input_name = sess.input_names[0]
    output_name = sess.output_names
    output = sess.run(output_name, {input_name: image_data},
                      input_offset=input_offset)
    top_five_label_probs = postprocess(output)
    logging.info("The input picture is classified to be:")
    for label_item, prob_item, class_item in top_five_label_probs:
        logging.info(
            f"label {label_item:3d}, prob {prob_item:.5f}, class {class_item}")


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
              default=128,
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
    sess = HB_ONNXRuntime(model_file=model)
    sess.set_dim_param(0, 0, '?')
    sess.set_providers(['CPUExecutionProvider'])
    inference(sess, image, input_layout, input_offset)


if __name__ == '__main__':
    main()
