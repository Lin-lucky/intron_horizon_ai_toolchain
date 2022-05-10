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
import xml.etree.ElementTree as ET

import numpy as np
from voc_metric import VOCMApMetric

np.set_printoptions(suppress=True)


def get_classes():
    VOC_CLASSES = {
        "aeroplane": (0, "Vehicle"),
        "bicycle": (1, "Vehicle"),
        "bird": (2, "Animal"),
        "boat": (3, "Vehicle"),
        "bottle": (4, "Indoor"),
        "bus": (5, "Vehicle"),
        "car": (6, "Vehicle"),
        "cat": (7, "Animal"),
        "chair": (8, "Indoor"),
        "cow": (9, "Animal"),
        "diningtable": (10, "Indoor"),
        "dog": (11, "Animal"),
        "horse": (12, "Animal"),
        "motorbike": (13, "Vehicle"),
        "person": (14, "Person"),
        "pottedplant": (15, "Indoor"),
        "sheep": (16, "Animal"),
        "sofa": (17, "Indoor"),
        "train": (18, "Vehicle"),
        "tvmonitor": (19, "Indoor"),
    }
    return VOC_CLASSES


def GetValData(annotations_path, val_txt_path):
    val_file = open(val_txt_path, 'r')
    res = []
    for f in val_file:
        file_name = f.strip() + '.xml'
        annotation_path = os.path.join(annotations_path, file_name)
        tree = ET.ElementTree(file=annotation_path)
        root = tree.getroot()
        object_set = root.findall('object')
        image_path = root.find('filename').text
        val_res = {}
        info_dict = {}
        info_dict['image_name'] = image_path
        info_dict['class_name'] = []
        info_dict['class_id'] = []
        info_dict['bbox'] = []
        info_dict["difficult"] = []
        for obj in object_set:
            obj_name = obj.find('name').text
            bbox = obj.find('bndbox')
            x1 = int(bbox.find('xmin').text)
            y1 = int(bbox.find('ymin').text)
            x2 = int(bbox.find('xmax').text)
            y2 = int(bbox.find('ymax').text)
            difficult = int(obj.find("difficult").text)
            bbox_loc = [x1, y1, x2, y2]

            info_dict['class_name'].append(obj_name)
            info_dict['class_id'].append(get_classes()[obj_name][0])
            info_dict['bbox'].append(bbox_loc)
            info_dict["difficult"].append(difficult)
        val_res[image_path] = info_dict
        res.append(val_res)
    return res


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
        help='evaluate voc dataset annotation path',
        required=True)
    parser.add_argument(
        '--val_txt_path',
        type=str,
        help='voc validation txt path',
        required=True)
    args = parser.parse_args()
    return args


def GetImageData(pred_path):
    lines = open(pred_path).read().splitlines()
    img_data = {}
    for line in lines:
        line = line.strip()
        data = line.split(" ")
        image_name = data[1]
        boxes_data = data[2:]
        box_list = []
        for r in boxes_data:
            box_data = r.split(",")
            data = []
            data.append(float(box_data[0]))
            data.append(float(box_data[1]))
            data.append(float(box_data[2]))
            data.append(float(box_data[3]))
            data.append(float(box_data[4]))
            data.append(float(box_data[5]))
            box_list.append(data)
        img_data[image_name] = box_list
    return img_data


def eval_result(metric):
    class_names, map_value = metric.get()
    print("\n\n=============mAP=============")
    for i in range(20):
        print("{}: {:4f}".format(class_names[i], map_value[i]))
    print("=============================")
    print('{}: {:.4f}'.format(class_names[-3], map_value[-3]))
    print('{}: {:.4f}'.format(class_names[-2], map_value[-2]))
    print('{}: {:.4f}'.format(class_names[-1], map_value[-1]))
    print("=============================")


def main():
    args = get_args()
    img_data = GetImageData(args.eval_result_path)
    val_data = GetValData(args.annotation_path, args.val_txt_path)
    if len(img_data) != len(val_data):
        print("eval result count is not equal gt count !!!")
        exit(-1)
    metric = VOCMApMetric(class_names=list(get_classes().keys()))
    print(args)

    for data in val_data:
        for k, v in data.items():
            anno_info = v
            gt_bboxes = np.array(anno_info['bbox'])[np.newaxis, ...]
            gt_labels = np.array(anno_info['class_id'])[np.newaxis, ...]
            gt_difficults = np.array(anno_info['difficult'])[np.newaxis, ...]
            pred_result = np.array(img_data[k])
            if pred_result.shape[0] == 0:
                pred_bboxes = np.zeros([1, 1, 4])
                pred_scores = np.zeros([1, 1, 1])
                pred_labels = np.zeros([1, 1, 1])
            else:
                pred_bboxes = pred_result[np.newaxis, ..., :4]
                pred_scores = pred_result[np.newaxis, ..., 4]
                pred_labels = pred_result[np.newaxis, ..., 5]
            metric.update(pred_bboxes, pred_labels, pred_scores, gt_bboxes,
                          gt_labels, gt_difficults)

    eval_result(metric)
    with open('result.txt', 'w') as f:
        names, values = metric.get()
        for name, value in zip(names, values):
            print(name, '{:.4f}'.format(value))
            record_string = name + ' ' + str('{:.4f}'.format(value)) + '\n'
            f.write(record_string)


if __name__ == '__main__':
    main()
