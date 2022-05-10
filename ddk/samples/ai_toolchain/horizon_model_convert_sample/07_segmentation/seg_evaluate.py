# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
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

from horizon_tc_ui.utils import tool_utils
from horizon_tc_ui import HB_ONNXRuntime
from horizon_tc_ui.utils.tool_utils import init_root_logger

from preprocess import eval_image_preprocess
from postprocess import eval_postprocess, calc_accuracy

sess = None


def init_sess(model):
    global sess
    sess = HB_ONNXRuntime(model_file=model)
    sess.set_dim_param(0, 0, '?')
    sess.set_providers(['CPUExecutionProvider'])


class ParallelExector(object):
    def __init__(self, input_layout, input_offset, parallel_num):
        logging.info(f"Init {parallel_num} processes")
        self._pool = multiprocessing.Pool(processes=parallel_num)
        self._pool._taskqueue.maxsize = parallel_num
        self._results = []
        self.input_layout = input_layout
        self.input_offset = input_offset

    def get_accuracy(self):
        return calc_accuracy(self._results)

    def infer(self, val_data, batch_id, total_batch):
        if sess.get_input_type() == 3:
            self.feed(val_data, batch_id, total_batch)
        else:
            logging.info(f"Feed batch {batch_id}/{total_batch}")
            gt_seg, pred_seg, filename = run(val_data, self.input_layout,
                                             self.input_offset)
            self._results.append((gt_seg, pred_seg, filename))

    def feed(self, val_data, batch_id, total_batch):
        logging.info("Feed batch {}/{}".format(batch_id, total_batch))
        r = self._pool.apply_async(func=run,
                                   args=(val_data, self.input_layout,
                                         self.input_offset),
                                   error_callback=logging.error)
        self._results.append(r)

    def close(self):
        self._pool.close()
        self._pool.join()


def run(val_data, input_layout, input_offset):
    input_name = sess.input_names[0]
    output_name = sess.output_names
    input_shape = sess.get_hw()
    (data, label, filename) = val_data
    # make sure pre-process logic is the same with runtime
    output = sess.run(output_name, {input_name: data},
                      input_offset=input_offset)
    pred_seg, gt_seg = eval_postprocess(output, label, input_shape)
    return pred_seg, gt_seg, filename


def _report(acc):
    print()
    tool_utils.report_flag_start('MAPPER-EVAL')
    print('{:.4f}'.format(acc))
    tool_utils.report_flag_end('MAPPER-EVAL')


def evaluate(image_path, label_path, input_layout, input_offset,
             total_image_number, parallel_num):
    if not input_layout:
        logging.warning(f"input_layout not provided. Using {sess.layout[0]}")
        input_layout = sess.layout[0]
    data_loader = eval_image_preprocess(image_path, label_path, sess.get_hw(),
                                        input_layout)
    val_exe = ParallelExector(input_layout, input_offset, parallel_num)

    for batch_id, val_data in enumerate(data_loader):
        if batch_id >= total_image_number:
            break
        val_exe.infer(val_data, batch_id, total_image_number)
    val_exe.close()
    accu, mIoU = val_exe.get_accuracy()
    _report(mIoU)


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
              default=5,
              help='Parallel eval process number')
@click.option('-c',
              '--color_sequence',
              type=str,
              default=None,
              help='Color sequence')
@click.option('-t',
              '--total_image_number',
              type=int,
              default=50,
              help='Total Image Number')
def main(model, image_path, label_path, input_layout, input_offset,
         parallel_num, color_sequence, total_image_number):
    init_root_logger("evaluation.log",
                     console_level=logging.INFO,
                     file_level=logging.DEBUG)
    if color_sequence:
        logging.warning("option color_sequence is deprecated.")
    init_sess(model)
    sess.set_dim_param(0, 0, '?')
    sess.set_providers(['CPUExecutionProvider'])
    parallel_num = int(os.environ.get('PARALLEL_PROCESS_NUM', parallel_num))
    evaluate(image_path, label_path, input_layout, input_offset,
             total_image_number, parallel_num)


if __name__ == '__main__':
    main()
