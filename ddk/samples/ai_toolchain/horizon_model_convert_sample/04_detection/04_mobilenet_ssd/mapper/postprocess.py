# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import numpy as np
import logging
import cv2
import re
import time
import math
import colorsys
from easydict import EasyDict
from PIL import Image
from matplotlib import pyplot as plt
from multiprocessing.pool import ApplyResult
from voc_metric import VOCMApMetric
from horizon_tc_ui.utils import tool_utils


def postprocess(model_output,
                model_hw_shape,
                origin_image=None,
                origin_img_shape=None,
                score_threshold=0.4,
                nms_threshold=0.45,
                dump_image=True):
    """decode all outputs for a single image input

    Arguments:
        model_output: a list contains 12 np array, like:
            [pred_loc0, pred_prob0, pred_loc1, pred_prob1 ...]
            pred_loc0 repesents the prediction location with shape:
            (b, h_0, w_0, 4 * num_anchor0)
            pred_prob0 repesents the prediction logits with shape:
            (b, h_0, w_0, num_anchor0 * num_class)

        model_hw_shape: (h, w) of original input image shape

    Returns:
        predicting_bbox: [type] -- np array with shape of (N, 6),
            N means the objects in the input image,
            6 is (x_min, y_min, x_max, y_max, score, class_index)
    """

    ssd_config = get_ssd_config()
    if origin_image is not None:
        ori_img_shape = origin_image.shape[1:-1]
    else:
        ori_img_shape = origin_img_shape
    decode_bboxes = []
    decode_prob = []
    for i in range(0, len(model_output) // 2):
        loc, prob = model_output[i * 2], model_output[i * 2 + 1]
        loc = _bbox_decode(loc, i, ori_img_shape, ssd_config)
        prob = _prob_decode(ssd_config, prob)
        decode_bboxes.append(loc)
        decode_prob.append(prob)

    concat_bboxes = np.concatenate(decode_bboxes, axis=0)
    concat_conf = np.concatenate(decode_prob, axis=0)
    predicting_bbox = postprocess_boxes(concat_bboxes, concat_conf,
                                        ori_img_shape,
                                        ssd_config.BACKGROUND_INDEX,
                                        ssd_config.CONF_THRESHOLD)

    predicting_bbox[..., 5] = predicting_bbox[..., 5] - 1
    predicting_bbox = nms(predicting_bbox, ssd_config.NMS_THRESHOLD)
    if dump_image:
        print("detection num: ", predicting_bbox.shape[0])
        draw_bboxs(origin_image, predicting_bbox)
    return predicting_bbox


def eval_postprocess(model_output,
                     model_hw_shape,
                     entry_dict,
                     score_threshold=0.001,
                     nms_threshold=0.65):
    anno_info = entry_dict[0]
    gt_bboxes = np.array(anno_info['bbox'])[np.newaxis, ...]
    gt_labels = np.array(anno_info['class_id'])[np.newaxis, ...]
    gt_difficults = np.array(anno_info['difficult'])[np.newaxis, ...]
    pred_result = postprocess(model_output,
                              model_hw_shape,
                              origin_img_shape=entry_dict[0]['origin_shape'],
                              score_threshold=score_threshold,
                              nms_threshold=nms_threshold,
                              dump_image=False)
    if pred_result.shape[0] == 0:
        pred_bboxes = np.zeros([1, 1, 4])
        pred_scores = np.zeros([1, 1, 1])
        pred_labels = np.zeros([1, 1, 1])
    else:
        pred_bboxes = pred_result[np.newaxis, ..., :4]
        pred_scores = pred_result[np.newaxis, ..., 4]
        pred_labels = pred_result[np.newaxis, ..., 5]

    return pred_bboxes, pred_labels, pred_scores, gt_bboxes, gt_labels, gt_difficults, anno_info[
        'image_name']


def calc_accuracy(annotation_path, accuracy):
    metric = VOCMApMetric(class_names=list(get_classes().keys()))

    if isinstance(accuracy[0], ApplyResult):
        batch_result = [i.get() for i in accuracy]
    else:
        batch_result = list(accuracy)
    total_samples = 0
    pred_list = []
    with open("mobilenetssd_eval.log", "w") as eval_log_handle:
        logging.info("start to calc eval info...")
        for pred_bboxes, pred_labels, pred_scores, gt_bboxes, gt_labels, gt_difficults, filename in batch_result:
            metric.update(pred_bboxes, pred_labels, pred_scores, gt_bboxes,
                          gt_labels, gt_difficults)
            pred_list = []
            eval_log_handle.write(f"input_image_name: {filename} ")
            for box_item, label_item, score_item in zip(
                    pred_bboxes[0], pred_labels[0], pred_scores[0]):
                pred_list_item = []
                label_item = np.squeeze(label_item)
                score_item = np.squeeze(score_item)
                pred_list_item.append(box_item[0])
                pred_list_item.append(box_item[1])
                pred_list_item.append(box_item[2])
                pred_list_item.append(box_item[3])
                pred_list_item.append(score_item)
                pred_list_item.append(label_item)
                pred_list.append(pred_list_item)
            pred_list.sort(key=lambda x: x[4], reverse=True)
            for box_item in pred_list:
                eval_log_handle.write(
                    f"{box_item[0]:6f},{box_item[1]:6f},{box_item[2]:6f},{box_item[3]:6f},{box_item[4]:6f},{box_item[5]:0.0f} "
                )
            eval_log_handle.write("\n")
    return metric.get()


def gen_report(metric_result):
    ssd_config = get_ssd_config()
    class_names, map_value = metric_result
    print("\n\n=============mAP=============")
    for i in range(ssd_config.NUM_CLASSES - 1):
        print("{}: {:4f}".format(class_names[i], map_value[i]))
    print("=============================")
    print('{}: {:.4f}'.format(class_names[-3], map_value[-3]))
    print('{}: {:.4f}'.format(class_names[-2], map_value[-2]))
    print('{}: {:.4f}'.format(class_names[-1], map_value[-1]))
    print()
    tool_utils.report_flag_start('MAPPER-EVAL')
    print('{:.4f}'.format(map_value[-3]))
    tool_utils.report_flag_end('MAPPER-EVAL')


def get_ssd_config():
    config = EasyDict()
    # config for caffe_ssd model, no used
    # config.variance_encoded_in_target = False
    # config.code_type = "center_size"
    # config.S_MIN = 0.2
    # config.S_MAX = 0.9
    config.NUM_CLASSES = 20 + 1  # VOC + background
    config.BACKGROUND_INDEX = 0
    config.STEPS = [15, 30, 60, 100, 150, 300]
    config.INPUT_SHAPE = (1, 3, 300, 300)
    config.MIN_SIZE = [60, 105, 150, 195, 240, 285]
    config.MAX_SIZE = [-1, 150, 195, 240, 285, 300]
    config.ASPECT_RATIOS = [[2, 1 / 2], [2, 1 / 2, 3, 1 / 3],
                            [2, 1 / 2, 3, 1 / 3], [2, 1 / 2, 3, 1 / 3],
                            [2, 1 / 2, 3, 1 / 3], [2, 1 / 2, 3, 1 / 3]]
    config.VARIANCE = [0.1, 0.2]
    config.CONF_THRESHOLD = 0.25
    config.NMS_THRESHOLD = 0.45

    config.NUM_OF_VOC_VALID_IMAGE = 5823
    return config


def _prob_decode(ssd_config, prob):
    """decode the predicting prossibility with softmax

    Arguments:
        prob {np array} -- shape of prob:
            (N, H, W, num_anchor * (num_classes + 1))

    Returns:
        prob np array -- prob of each classes processed by softmax
    """
    prob = np.reshape(prob, (-1, ssd_config.NUM_CLASSES))
    prob_exp = np.exp(prob)
    prob_exp_sum = np.sum(prob_exp, axis=1, keepdims=True)
    prob = prob_exp / prob_exp_sum
    return prob


def _bbox_decode(loc, index, origin_shape, ssd_config):
    """decode the predicting bboxes

        Arguments:
            loc {np array} -- shape of loc:(N, feat_h, feat_w, 4 * num_anchor),
                contains (pred_x_min, pred_y_min, pred_x_max, pred_y_max)
            index {int} -- the index of the ssd output
            origin_shape {tuple} -- (orgin h, origin w)

        Returns:
            decode_bboxes [np array] -- decoded predicting bboxes,
                contains (box_x_min, box_y_min, box_x_max, box_y_max)
        """
    def get_prior_box(ssd_config):
        # this formula is from SSD paper, but not same with the ssd-caffe model.
        # sk_generate_func = lambda k : self.ssd_config.S_MIN + \
        #     (self.ssd_config.S_MAX - self.ssd_config.S_MIN) / \
        #     (self.ssd_config.NUM_OUTPUTS) * k
        # min_size = sk_generate_func(index) * self.input_height
        # max_size = sk_generate_func(index + 1) * self.input_height
        input_height = ssd_config.INPUT_SHAPE[2]
        input_width = ssd_config.INPUT_SHAPE[3]

        min_size = ssd_config.MIN_SIZE[index]
        max_size = ssd_config.MAX_SIZE[index]
        aspect_ratio = ssd_config.ASPECT_RATIOS[index]
        if max_size > 0:
            prior_bboxs_wh = [
                min_size, min_size,
                math.sqrt(max_size * min_size),
                math.sqrt(max_size * min_size)
            ]
        else:
            prior_bboxs_wh = [min_size, min_size]
        for ratio in aspect_ratio:
            a = math.sqrt(ratio)
            w = (min_size * a)
            h = (min_size / a)
            prior_bboxs_wh.extend((w, h))
        prior_bboxs_wh = np.array(prior_bboxs_wh).reshape((-1, 2))
        num_prior_bbox = prior_bboxs_wh.shape[0]

        x = np.tile(
            np.arange(layer_size, dtype=np.float32)[:, np.newaxis],
            [1, layer_size])
        y = np.tile(
            np.arange(layer_size, dtype=np.float32)[np.newaxis, :],
            [layer_size, 1])
        xy_grid = np.concatenate([x[:, :, np.newaxis], y[:, :, np.newaxis]],
                                 axis=-1)
        xy_grid = np.tile(xy_grid[np.newaxis, :, :, np.newaxis, :],
                          [1, 1, 1, num_prior_bbox, 1])
        center_xy_grid = (xy_grid + 0.5) * stride
        center_xy_grid = center_xy_grid[..., [1, 0]]
        bbox_xy_min = (center_xy_grid - prior_bboxs_wh / 2) / input_width
        bbox_xy_max = (center_xy_grid + prior_bboxs_wh / 2) / input_height
        prior_bboxs = np.concatenate([bbox_xy_min, bbox_xy_max], axis=-1)
        return prior_bboxs

    batch_size = ssd_config.INPUT_SHAPE[0]
    layer_size = loc.shape[1]
    stride = ssd_config.STEPS[index]
    loc = np.reshape(loc, (batch_size, layer_size, layer_size, -1, 4))
    pred_xy_min = loc[..., :2]
    pred_xy_max = loc[..., 2:]

    prior_bboxs = get_prior_box(ssd_config)
    prior_xy_min, prior_xy_max = prior_bboxs[..., :2], prior_bboxs[..., 2:]
    prior_wh = prior_xy_max - prior_xy_min
    prior_center_xy = (prior_xy_min + prior_xy_max) / 2

    decode_xy = ssd_config.VARIANCE[
        0] * pred_xy_min * prior_wh + prior_center_xy
    decode_wh = np.exp(ssd_config.VARIANCE[1] * pred_xy_max) * prior_wh
    decode_bboxes = np.concatenate([decode_xy, decode_wh], axis=-1)
    decode_bboxes = np.reshape(decode_bboxes, (-1, 4))

    org_h, org_w = origin_shape
    # (x, y, w, h) --> (xmin, ymin, xmax, ymax)
    decode_bboxes = np.concatenate([
        decode_bboxes[:, :2] - decode_bboxes[:, 2:] * 0.5,
        decode_bboxes[:, :2] + decode_bboxes[:, 2:] * 0.5
    ],
                                   axis=-1)

    # (xmin, ymin, xmax, ymax) -> (xmin_org, ymin_org, xmax_org, ymax_org)
    decode_bboxes[:, 0::2] = decode_bboxes[:, 0::2] * org_w
    decode_bboxes[:, 1::2] = decode_bboxes[:, 1::2] * org_h
    return decode_bboxes


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


def draw_bboxs(image,
               bboxes,
               name='demo',
               gt_classes_index=None,
               classes=get_classes()):
    """draw the bboxes in the original image
    """
    classes = list(classes.keys())
    num_classes = len(classes)
    if len(image.shape) == 4:
        image = image[0]
    image_h, image_w, channel = image.shape
    hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(
        map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
            colors))

    fontScale = 0.5
    bbox_thick = int(0.6 * (image_h + image_w) / 600)
    for i, bbox in enumerate(bboxes):
        coor = np.array(bbox[:4], dtype=np.int32)

        if gt_classes_index == None:
            class_index = int(bbox[5])  # - 1
            score = bbox[4]
        else:
            class_index = gt_classes_index[i]  # - 1
            score = 1

        bbox_color = colors[class_index]
        c1, c2 = (coor[0], coor[1]), (coor[2], coor[3])
        cv2.rectangle(image, c1, c2, bbox_color, bbox_thick)
        classes_name = classes[class_index]
        bbox_mess = '%s: %.5f' % (classes_name, score)
        t_size = cv2.getTextSize(bbox_mess,
                                 0,
                                 fontScale,
                                 thickness=bbox_thick // 2)[0]
        cv2.rectangle(image, c1, (c1[0] + t_size[0], c1[1] - t_size[1] - 3),
                      bbox_color, -1)
        cv2.putText(image,
                    bbox_mess, (c1[0], c1[1] - 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale, (0, 0, 0),
                    bbox_thick // 2,
                    lineType=cv2.LINE_AA)
        print("{} is in the picture with confidence:{:.4f}".format(
            classes_name, score))
    cv2.imwrite("{}.jpg".format(name), image)
    return image


def postprocess_boxes(pred_location,
                      pred_prob,
                      orgin_shape,
                      background_index,
                      score_threshold=0.3):
    """post process boxes"""
    valid_scale = [0, np.inf]
    org_h, org_w = orgin_shape

    # clip the range of bbox
    pred_location = np.concatenate([
        np.maximum(pred_location[:, :2], [0, 0]),
        np.minimum(pred_location[:, 2:], [org_w - 1, org_h - 1])
    ],
                                   axis=-1)

    # drop illegal boxes whose max < min
    invalid_mask = np.logical_or((pred_location[:, 0] > pred_location[:, 2]),
                                 (pred_location[:, 1] > pred_location[:, 3]))
    pred_location[invalid_mask] = 0

    # discard invalid boxes
    bboxes_scale = np.sqrt(
        np.multiply.reduce(pred_location[:, 2:4] - pred_location[:, 0:2],
                           axis=-1))
    scale_mask = np.logical_and((valid_scale[0] < bboxes_scale),
                                (bboxes_scale < valid_scale[1]))
    # discard boxes of background
    classes_index = np.argmax(pred_prob, axis=-1)
    class_mask = classes_index != background_index
    scores = np.max(pred_prob, axis=-1)
    score_mask = scores > score_threshold
    mask = np.logical_and(scale_mask, score_mask)
    mask = np.logical_and(mask, class_mask)
    coors, scores, classes = pred_location[mask], scores[mask], classes_index[
        mask]

    valid_bboxs = np.concatenate(
        [coors, scores[:, np.newaxis], classes[:, np.newaxis]], axis=-1)

    return valid_bboxs


def nms(bboxes, iou_threshold, sigma=0.3, method='nms'):
    """
    calculate the nms for bboxes
    """
    def bboxes_iou(boxes1, boxes2):
        """calculate iou for a list of bboxes"""
        boxes1 = np.array(boxes1)
        boxes2 = np.array(boxes2)
        boxes1_area = (boxes1[..., 2] - boxes1[..., 0]) * \
                    (boxes1[..., 3] - boxes1[..., 1])
        boxes2_area = (boxes2[..., 2] - boxes2[..., 0]) * \
                    (boxes2[..., 3] - boxes2[..., 1])
        left_up = np.maximum(boxes1[..., :2], boxes2[..., :2])
        right_down = np.minimum(boxes1[..., 2:], boxes2[..., 2:])
        inter_section = np.maximum(right_down - left_up, 0.0)
        inter_area = inter_section[..., 0] * inter_section[..., 1]
        union_area = boxes1_area + boxes2_area - inter_area
        ious = np.maximum(1.0 * inter_area / union_area,
                          np.finfo(np.float32).eps)
        return ious

    classes_in_img = list(set(bboxes[:, 5]))
    best_bboxes = []
    for cls in classes_in_img:
        cls_mask = (bboxes[:, 5] == cls)
        cls_bboxes = bboxes[cls_mask]

        while len(cls_bboxes) > 0:
            max_ind = np.argmax(cls_bboxes[:, 4])
            best_bbox = cls_bboxes[max_ind]
            best_bboxes.append(best_bbox)
            cls_bboxes = np.concatenate(
                [cls_bboxes[:max_ind], cls_bboxes[max_ind + 1:]])
            iou = bboxes_iou(best_bbox[np.newaxis, :4], cls_bboxes[:, :4])

            weight = np.ones((len(iou), ), dtype=np.float32)

            assert method in ['nms', 'soft-nms']
            if method == 'nms':
                iou_mask = iou > iou_threshold
                weight[iou_mask] = 0.0
            if method == 'soft-nms':
                weight = np.exp(-(1.0 * iou**2 / sigma))

            cls_bboxes[:, 4] = cls_bboxes[:, 4] * weight
            score_mask = cls_bboxes[:, 4] > 0.
            cls_bboxes = cls_bboxes[score_mask]
    best_bboxes = np.array(best_bboxes)

    return best_bboxes
