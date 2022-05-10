# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import threading
import numpy as np
import math
import os
import sys
import argparse
import json
import cv2
import re
import skimage.io
from PIL import Image
from cityscapes_metric import SegmentationMetric

dst_w = 2048
dst_h = 1024

pred_w = 2048 // 4
pred_h = 1024 // 4

pixLabels = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    4: 0,
    5: 0,
    6: 0,
    7: 1,
    8: 2,
    9: 0,
    10: 0,
    11: 3,
    12: 4,
    13: 5,
    14: 0,
    15: 0,
    16: 0,
    17: 6,
    18: 0,
    19: 7,
    20: 8,
    21: 9,
    22: 10,
    23: 11,
    24: 12,
    25: 13,
    26: 14,
    27: 15,
    28: 16,
    29: 0,
    30: 0,
    31: 17,
    32: 18,
    33: 19,
    -1: 0
}


def get_pred_dict(log_file):
    res = dict()
    nbv_reg = re.compile(
        r'^.*input_image_name: .*?(?P<name>[^/]*)_leftImg8bit.png_block_(?P<block>[\d]*) (?P<value>.*)\n$'
    )

    with open(log_file, 'r') as file:
        lines = file.readlines()

    image_name = None
    value_str = None
    image_block = -1
    for line in lines:
        if 'input_image_name' not in line:
            continue

        if '.png' not in line:
            continue

        nbv_group = nbv_reg.match(line).groupdict()
        name = nbv_group['name']
        block = int(nbv_group['block'])
        value = nbv_group['value']

        if name != image_name or block < image_block:
            if image_name is not None and image_name not in res:
                res[image_name] = decry_mask(value_str)
            image_name = name
            value_str = ''
            image_block = -1

        image_block = block
        value_str += value

    if value_str != '' and image_name not in res:
        res[image_name] = decry_mask(value_str)

    return res


def decry_mask(value_str):
    value = value_str.split(' ')
    del value[-1]
    value = np.array(value).astype(np.uint8).reshape(pred_h, pred_w)
    value = cv2.resize(value, (dst_w, dst_h), interpolation=cv2.INTER_NEAREST)

    return value


def get_gt_file(val_path):
    gt_path_list = [
        os.path.join(val_path, 'frankfurt'),
        os.path.join(val_path, 'lindau'),
        os.path.join(val_path, 'munster')
    ]
    res = dict()
    for gt_path in gt_path_list:
        files = os.listdir(gt_path)
        for file in files:
            if '_gtFine_labelIds.png' not in file:
                continue

            map_key = file[:-20]
            gt = cv2.imread(os.path.join(gt_path, file), 0)
            binary_gt = np.zeros_like(gt).astype(np.int32)
            for key in pixLabels.keys():
                index = np.where(gt == key)
                binary_gt[index] = pixLabels[key] - 1
            res[map_key] = binary_gt

    return res


def get_eval(val_path, log_file):
    metric = SegmentationMetric(nclass=19)
    gt_dict = get_gt_file(val_path)
    pred_dict = get_pred_dict(log_file)

    assert len(gt_dict) == len(pred_dict), \
        "the number of image:{} is not equal to the number of label:{}" \
        .format(len(gt_dict), len(pred_dict))

    gt_num = len(gt_dict)

    for key in pred_dict:
        pred = np.expand_dims(pred_dict[key], axis=0)
        label = np.expand_dims(gt_dict[key], axis=0)
        gt_seg = np.squeeze(label)

        metric.update(gt_seg, pred)
        accu, mIoU = metric.get()
        print("pixel accuracy:{:.4f}, mIoU:{:.6f}".format(accu, mIoU))


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gt_path",
        "-g",
        type=str,
        help="gt label file directory.",
        required=True)
    parser.add_argument(
        "--log_file", "-l", type=str, help="eval.txt file", required=True)
    args = parser.parse_args()
    return args


def main():
    args = get_args()
    get_eval(args.gt_path, args.log_file)


if __name__ == '__main__':
    main()
