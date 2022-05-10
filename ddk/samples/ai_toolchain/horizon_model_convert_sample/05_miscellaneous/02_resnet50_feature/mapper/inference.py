# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import os
import sys
import numpy as np
import click
import logging
sys.path.append('.')

from horizon_tc_ui import HB_ONNXRuntime
from horizon_tc_ui.data.raw_data_reader import raw_data_read
from horizon_tc_ui.utils.tool_utils import init_root_logger


@click.version_option(version="1.0.0")
@click.command()
@click.option('-m', '--model', type=str, help='Input onnx model(.onnx) file')
@click.option('-f', '--feature', type=str, help='Inference feature file path.')
def main(model, feature):
    init_root_logger("inference.log",
                     console_level=logging.INFO,
                     file_level=logging.DEBUG)
    shape = [1, 64, 56, 56]
    feature_data = raw_data_read(feature, shape, np.float32)
    sess = HB_ONNXRuntime(model_file=model)
    input_name = sess.get_inputs()[0].name
    output_name = sess.get_outputs()[0].name
    output = sess.run_feature([output_name], {input_name: feature_data},
                              input_offset=0)
    prob = np.squeeze(output[0])
    idx = np.argsort(-prob)

    top_five_label_probs = [(idx[i], prob[idx[i]]) for i in range(5)]
    logging.info(
        "The input picture is classified to be:\n%s" %
        "\n".join(['label %d: prob %.2f' % e for e in top_five_label_probs]))


if __name__ == '__main__':
    main()
