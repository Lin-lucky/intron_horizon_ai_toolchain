# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import argparse
import re
import os
import json

import numpy as np
from coco_metric import MSCOCODetMetric
from x3_tc_ui.utils import tool_utils


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--eval_result_path",
        type=str,
        help="eetection eval result file path",
        required=True)
    parser.add_argument(
        '--annotation_path',
        type=str,
        help='evaluate coco dataset annotation path',
        required=True)
    args = parser.parse_args()
    return args


def GetImageData(pred_path):
    lines = open(pred_path).read().splitlines()
    img_data = {}
    # for line in lines:
    #     data = json.loads(line.strip())
    #     image_name = data['frame']['image_name']
    #     box_list = []
    #     for r in data['result']:
    #         box_data = r['bbox'] + [r['score'], r['id']]
    #         box_list.append({'bbox': np.array(box_data), 'mask': False})
    #     img_data[image_name] = box_list
    # return img_data
    for line in lines:
        line = line.strip()
        data = line.split(" ")
        image_name = data[1]
        boxes_data = data[2:]
        box_list = []
        for r in boxes_data:
            box_data = r.split(",")
            box_list.append({'bbox': np.array(box_data), 'mask': False})
        img_data[image_name] = box_list
    return img_data


def _report(metric):
    names, values = metric.get()
    summary = values[0]
    summary = summary.splitlines()
    pattern = re.compile(r'(IoU.*?) .* (.*)$')
    ctx = os.getenv('HORIZON_RUN_CONTEXT', '')
    print()
    tool_utils.report_flag_start('RUNTIME-%s-EVAL' % ctx)
    for v in summary[0:2]:
        valid_data = pattern.findall(v)[0]
        print("[%s] = %s" % (valid_data[0], valid_data[1]))
    tool_utils.report_flag_end('RUNTIME-%s-EVAL' % ctx)


def main():
    args = get_args()
    img_data = GetImageData(args.eval_result_path)
    # img_data = GetImageData("./yolov2_eval.out")
    # img_data = GetImageData("./yolov2.log")
    metric = MSCOCODetMetric(args.annotation_path, with_mask=False)
    print(args)
    for img_name in img_data:
        img = img_data[img_name]
        pred_result = []
        for one_bbox in img:
            one_result = {
                'bbox': one_bbox["bbox"].astype(np.float),
                'mask': False
            }
            pred_result.append(one_result)
        metric.update(pred_result, img_name)

    with open('result.txt', 'w') as f:
        names, values = metric.get()
        for name, value in zip(names, values):
            print(name, value)
            record_string = name + ' ' + value + '\n'
            f.write(record_string)
    _report(metric)


if __name__ == '__main__':
    main()
