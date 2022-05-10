# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import logging
import multiprocessing
import click
import os

from horizon_tc_ui import HB_ONNXRuntime
from horizon_tc_ui.utils.tool_utils import init_root_logger

from preprocess import eval_image_preprocess
from postprocess import eval_postprocess, calc_accuracy, gen_report

sess = None


def init_sess(model):
    global sess
    sess = HB_ONNXRuntime(model_file=model)
    sess.set_dim_param(0, 0, '?')
    sess.set_providers(['CPUExecutionProvider'])


class ParallelExector(object):
    def __init__(self, parallel_num, input_layout, input_offset):
        if sess.get_input_type() == 3:
            logging.info(f"Init {parallel_num} processes")
            self._pool = multiprocessing.Pool(processes=parallel_num)
            self._pool._taskqueue.maxsize = parallel_num
        self._results = []
        self.input_layout = input_layout
        self.input_offset = input_offset

    def get_accuracy(self, total_batch):
        return calc_accuracy(self._results, total_batch)

    def infer(self, val_data, batch_id, total_batch):
        if sess.get_input_type() == 3:
            self.feed(val_data, batch_id, total_batch)
        else:
            logging.info(f"Feed batch {batch_id}/{total_batch}")
            eval_result = run(val_data, batch_id, total_batch,
                              self.input_layout, self.input_offset)
            self._results.append(eval_result)

    def feed(self, val_data, batch_id, total_batch):
        logging.info("Feed batch {}/{}".format(batch_id, total_batch))
        r = self._pool.apply_async(func=run,
                                   args=(val_data, batch_id, total_batch,
                                         self.input_layout, self.input_offset),
                                   error_callback=logging.error)
        self._results.append(r)

    def close(self):
        if hasattr(self, "_pool"):
            self._pool.close()
            self._pool.join()


def run(val_data, batch_id, total_batch, input_layout, input_offset):
    logging.info("Eval batch {}/{}".format(batch_id, total_batch))
    input_name = sess.input_names[0]
    output_name = sess.output_names
    (data, label, filename) = val_data
    # make sure pre-process logic is the same with runtime
    output = sess.run(output_name, {input_name: data},
                      input_offset=input_offset)
    predict_label, sample_correction = eval_postprocess(output, label)
    return batch_id, predict_label, sample_correction, filename


def evaluate(image_path, label_path, input_layout, input_offset, parallel_num):
    if not input_layout:
        logging.warning(f"input_layout not provided. Using {sess.layout[0]}")
        input_layout = sess.layout[0]
    data_loader, loader_size = eval_image_preprocess(image_path, label_path,
                                                     input_layout)
    val_exe = ParallelExector(parallel_num, input_layout, input_offset)
    total_batch = loader_size
    for batch_id, val_data in enumerate(data_loader):
        val_exe.infer(val_data, batch_id, total_batch)

    val_exe.close()
    metric_result = val_exe.get_accuracy(total_batch)
    gen_report(metric_result)


@click.version_option(version="1.0.0")
@click.command()
@click.option('-m', '--model', type=str, help='Input onnx model(.onnx) file')
@click.option('-i',
              '--image_path',
              type=str,
              help='Evaluation image directory.')
@click.option('-l', '--label_path', type=str, help='Evaluate image label path')
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
@click.option('-p',
              '--parallel_num',
              type=int,
              default=10,
              help='Parallel eval process number')
@click.option('-c',
              '--color_sequence',
              type=str,
              default=None,
              help='Color sequence')
def main(model, image_path, label_path, input_layout, input_offset,
         parallel_num, color_sequence):
    init_root_logger("evaluation.log",
                     console_level=logging.INFO,
                     file_level=logging.DEBUG)
    if color_sequence:
        logging.warning("option color_sequence is deprecated.")
    init_sess(model)
    sess.set_dim_param(0, 0, '?')
    sess.set_providers(['CPUExecutionProvider'])
    parallel_num = int(os.environ.get('PARALLEL_PROCESS_NUM', parallel_num))
    evaluate(image_path, label_path, input_layout, input_offset, parallel_num)


if __name__ == '__main__':
    main()
