# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import os
import re
import sys
import json
import argparse


def get_predict_result(file):
    res = dict()
    with open(file, 'r') as file:
        lines = file.readlines()

    predict_nv_reg = re.compile(
        r'.*input_image_name: *.*?(?P<name>[^\\/]*)\.(?:png|jpg|jpeg|JPEG|bmp) *(?P<value>.*)\n'
    )
    predict_pair_reg = re.compile(r'(class_id: [^ ]{1,} class_name: [^ ]{1,})')
    predict_id_reg = re.compile(r'class_id: (?P<id>[^ ]{1,}) class_name.*')

    for line in lines:
        if 'input_image_name' not in line:
            continue

        nv_group = predict_nv_reg.match(line).groupdict()

        name = nv_group['name']
        value = nv_group['value']
        if name in res:
            continue

        res[name] = []
        pairs = re.findall(predict_pair_reg, value)
        for pair in pairs:
            id = predict_id_reg.match(pair).groupdict()['id']
            res[name].append(id)

    return res


def get_ground_truth(gt_file):
    gt_res = dict()
    gt_nv_reg = re.compile(
        r' *?(?P<name>[^\\/]*)\.(?:png|jpg|jpeg|JPEG|bmp) {1,}(?P<value>.*) *\n'
    )

    with open(gt_file, 'r') as file:
        lines = file.readlines()

    for line in lines:
        nv_group = gt_nv_reg.match(line).groupdict()
        name = nv_group['name'].strip()
        value = nv_group['value'].strip()

        gt_res[name] = value

    return gt_res


def get_class_accuracy_alone(file, val_file):
    predict_res = get_predict_result(file)
    gt_res = get_ground_truth(val_file)

    correct = 0
    error = 0

    for key in predict_res:
        if key not in gt_res:
            print('image: ', key, ' is not labeled')
            continue

        if gt_res[key] in predict_res[key][:1]:
            correct += 1
        else:
            error += 1

    valid_sum = correct + error
    if valid_sum > 0:
        return correct / valid_sum
    else:
        return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--log_file',
        type=str,
        required=True,
        help='log file from embedded application.')
    parser.add_argument(
        '--gt_file',
        type=str,
        required=True,
        help='the file contains image name and label.')
    args = parser.parse_args()
    gt_file = args.gt_file
    log_file = args.log_file

    print('{:.4f}'.format(get_class_accuracy_alone(log_file, gt_file)))
