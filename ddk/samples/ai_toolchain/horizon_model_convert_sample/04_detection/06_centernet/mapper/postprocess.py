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
import torch
import torch.nn as nn
import colorsys
from easydict import EasyDict
from PIL import Image
from matplotlib import pyplot as plt
from multiprocessing.pool import ApplyResult
from coco_metric import MSCOCODetMetric
from horizon_tc_ui.utils import tool_utils


def postprocess(model_output,
                model_hw_shape,
                origin_image=None,
                origin_img_shape=None,
                score_threshold=0.4,
                nms_threshold=0.45,
                dump_image=True):

    input_height = model_hw_shape[0]
    input_width = model_hw_shape[1]
    if origin_image is not None:
        origin_image_shape = origin_image.shape[1:3]
    else:
        origin_image_shape = origin_img_shape
    prediction_bbox = post_processor(model_output, origin_image_shape,
                                     input_height, input_width, 1.0, 4.0, 80,
                                     100)
    if dump_image and origin_image is not None:
        draw_bboxs(origin_image[0], prediction_bbox)
    return prediction_bbox


def eval_postprocess(model_output,
                     model_hw_shape,
                     entry_dict,
                     score_threshold=0.001,
                     nms_threshold=0.65):

    bboxes_pr = postprocess(model_output,
                            model_hw_shape,
                            origin_img_shape=entry_dict[0]['origin_shape'],
                            score_threshold=score_threshold,
                            nms_threshold=nms_threshold,
                            dump_image=False)
    pred_result = []
    for one_bbox in bboxes_pr:
        one_result = {'bbox': one_bbox, 'mask': False}
        pred_result.append(one_result)
    return pred_result, entry_dict[0]['image_name']


def calc_accuracy(annotation_path, accuracy):
    metric = MSCOCODetMetric(annotation_path, with_mask=False)
    if isinstance(accuracy[0], ApplyResult):
        batch_result = [i.get() for i in accuracy]
    else:
        batch_result = list(accuracy)
    total_samples = 0
    with open("centernet_eval.log", "w") as eval_log_handle:
        logging.info("start to calc eval info...")
        for pred_result, filename in batch_result:
            metric.update(pred_result, filename)
            pred_result.sort(key=lambda x: x['bbox'][4], reverse=True)
            eval_log_handle.write(f"input_image_name: {filename} ")
            for one_result in pred_result:
                det_item = one_result['bbox']
                eval_log_handle.write(
                    f"{det_item[0]:6f},{det_item[1]:6f},{det_item[2]:6f},{det_item[3]:6f},{det_item[4]:6f},{det_item[5]:0.0f} "
                )
            eval_log_handle.write("\n")
    return metric.get()


def gen_report(metric_result):
    names, values = metric_result
    summary = values[0]
    summary = summary.splitlines()
    pattern = re.compile(r'(IoU.*?) .* (.*)$')
    tool_utils.report_flag_start('MAPPER-EVAL')
    for v in summary[0:2]:
        valid_data = pattern.findall(v)[0]
        print("[%s] = %s" % (valid_data[0], valid_data[1]))
    tool_utils.report_flag_end('MAPPER-EVAL')


def post_processor(outputs,
                   origin_shape,
                   input_height,
                   input_width,
                   scale=1.0,
                   down_ratio=4.0,
                   num_classes=80,
                   topK=100):
    output_dict = {}
    output_dict['hm'] = torch.from_numpy(outputs[0])
    output_dict['wh'] = torch.from_numpy(outputs[1])
    output_dict['reg'] = torch.from_numpy(outputs[2])

    height, width = origin_shape
    new_height = int(height * scale)
    new_width = int(width * scale)
    c = np.array([new_width / 2., new_height / 2.], dtype=np.float32)
    s = max(height, width) * 1.0
    out_height = input_height // down_ratio
    out_width = input_width // down_ratio
    dets = ctdet_decode(output_dict['hm'].sigmoid_(),
                        output_dict['wh'],
                        reg=output_dict['reg'],
                        cat_spec_wh=False,
                        K=topK)
    dets = dets.detach().cpu().numpy()
    dets = dets.reshape(1, -1, dets.shape[2])
    dets = ctdet_post_process(dets.copy(), [c], [s], out_height, out_width,
                              num_classes)
    for j in range(1, num_classes + 1):
        dets[0][j] = np.array(dets[0][j], dtype=np.float32).reshape(-1, 5)
        dets[0][j][:, :4] /= scale
    nms_bboxes = []
    for cat, bboxes in dets[0].items():
        for bbox in bboxes:
            nms_bboxes.append(list(bbox) + [cat - 1])
    return np.array(nms_bboxes)


def ctdet_decode(heat, wh, reg=None, cat_spec_wh=False, K=100):
    batch, cat, height, width = heat.size()
    heat = _nms(heat)

    scores, inds, clses, ys, xs = _topk(heat, K=K)
    if reg is not None:
        reg = _transpose_and_gather_feat(reg, inds)
        reg = reg.view(batch, K, 2)
        xs = xs.view(batch, K, 1) + reg[:, :, 0:1]
        ys = ys.view(batch, K, 1) + reg[:, :, 1:2]
    else:
        xs = xs.view(batch, K, 1) + 0.5
        ys = ys.view(batch, K, 1) + 0.5
    wh = _transpose_and_gather_feat(wh, inds)
    if cat_spec_wh:
        wh = wh.view(batch, K, cat, 2)
        clses_ind = clses.view(batch, K, 1, 1).expand(batch, K, 1, 2).long()
        wh = wh.gather(2, clses_ind).view(batch, K, 2)
    else:
        wh = wh.view(batch, K, 2)
    clses = clses.view(batch, K, 1).float()
    scores = scores.view(batch, K, 1)
    bboxes = torch.cat([
        xs - wh[..., 0:1] / 2, ys - wh[..., 1:2] / 2, xs + wh[..., 0:1] / 2,
        ys + wh[..., 1:2] / 2
    ],
                       dim=2)
    bboxes = torch.clamp(bboxes, 0., 512.)
    detections = torch.cat([bboxes, scores, clses], dim=2)

    return detections


def _topk(scores, K=40):
    batch, cat, height, width = scores.size()

    topk_score, topk_ind = torch.topk(scores.view(batch, -1), K)

    topk_inds = topk_ind % (height * width)
    topk_ys = (topk_inds / width).int().float()
    topk_xs = (topk_inds % width).int().float()
    topk_clses = (topk_ind / (height * width)).int()

    return topk_score, topk_inds, topk_clses, topk_ys, topk_xs


def _nms(heat, kernel=3):
    pad = (kernel - 1) // 2

    hmax = nn.functional.max_pool2d(heat, (kernel, kernel),
                                    stride=1,
                                    padding=pad)
    keep = (hmax == heat).float()
    return heat * keep


def _gather_feat(feat, ind, mask=None):
    dim = feat.size(2)
    ind = ind.unsqueeze(2).expand(ind.size(0), ind.size(1), dim)
    feat = feat.gather(1, ind)
    if mask is not None:
        mask = mask.unsqueeze(2).expand_as(feat)
        feat = feat[mask]
        feat = feat.view(-1, dim)
    return feat


def _transpose_and_gather_feat(feat, ind):
    feat = feat.permute(0, 2, 3, 1).contiguous()
    feat = feat.view(feat.size(0), -1, feat.size(3))
    feat = _gather_feat(feat, ind)
    return feat


def ctdet_post_process(dets, c, s, h, w, num_classes):
    ret = []
    for i in range(dets.shape[0]):
        top_preds = {}
        dets[i, :, :2] = transform_preds(dets[i, :, 0:2], c[i], s[i], (w, h))
        dets[i, :, 2:4] = transform_preds(dets[i, :, 2:4], c[i], s[i], (w, h))
        classes = dets[i, :, -1]
        for j in range(num_classes):
            inds = (classes == j)
            top_preds[j + 1] = np.concatenate([
                dets[i, inds, :4].astype(np.float32),
                dets[i, inds, 4:5].astype(np.float32)
            ],
                                              axis=1).tolist()
            ret.append(top_preds)
    return ret


def transform_preds(coords, center, scale, output_size):
    target_coords = np.zeros(coords.shape)
    trans = get_affine_transform(center, scale, 0, output_size, inv=1)
    for p in range(coords.shape[0]):
        target_coords[p, 0:2] = affine_transform(coords[p, 0:2], trans)
    return target_coords


def get_affine_transform(center,
                         scale,
                         rot,
                         output_size,
                         shift=np.array([0, 0], dtype=np.float32),
                         inv=0):
    if not isinstance(scale, np.ndarray) and not isinstance(scale, list):
        scale = np.array([scale, scale], dtype=np.float32)

    scale_tmp = scale
    src_w = scale_tmp[0]
    dst_w = output_size[0]
    dst_h = output_size[1]

    rot_rad = np.pi * rot / 180
    src_dir = get_dir([0, src_w * -0.5], rot_rad)
    dst_dir = np.array([0, dst_w * -0.5], np.float32)

    src = np.zeros((3, 2), dtype=np.float32)
    dst = np.zeros((3, 2), dtype=np.float32)
    src[0, :] = center + scale_tmp * shift
    src[1, :] = center + src_dir + scale_tmp * shift
    dst[0, :] = [dst_w * 0.5, dst_h * 0.5]
    dst[1, :] = np.array([dst_w * 0.5, dst_h * 0.5], np.float32) + dst_dir

    src[2:, :] = get_3rd_point(src[0, :], src[1, :])
    dst[2:, :] = get_3rd_point(dst[0, :], dst[1, :])

    if inv:
        trans = cv2.getAffineTransform(np.float32(dst), np.float32(src))
    else:
        trans = cv2.getAffineTransform(np.float32(src), np.float32(dst))

    return trans


def affine_transform(pt, t):
    new_pt = np.array([pt[0], pt[1], 1.], dtype=np.float32).T
    new_pt = np.dot(t, new_pt)
    return new_pt[:2]


def get_3rd_point(a, b):
    direct = a - b
    return b + np.array([-direct[1], direct[0]], dtype=np.float32)


def get_dir(src_point, rot_rad):
    sn, cs = np.sin(rot_rad), np.cos(rot_rad)

    src_result = [0, 0]
    src_result[0] = src_point[0] * cs - src_point[1] * sn
    src_result[1] = src_point[0] * sn + src_point[1] * cs

    return src_result


def get_classes(class_file_name='coco_classes.names'):
    '''loads class name from a file'''
    names = {}
    with open(class_file_name, 'r') as data:
        for ID, name in enumerate(data):
            names[ID] = name.strip('\n')
    return names


def draw_bboxs(image, bboxes, gt_classes_index=None, classes=get_classes()):
    """draw the bboxes in the original image
    """
    num_classes = len(classes)
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
            class_index = int(bbox[5])
            score = bbox[4]
        else:
            class_index = gt_classes_index[i]
            score = 1
        if score < 0.3:
            continue
        bbox_color = colors[class_index]
        c1, c2 = (coor[0], coor[1]), (coor[2], coor[3])
        cv2.rectangle(image, c1, c2, bbox_color, bbox_thick)
        classes_name = classes[class_index]
        bbox_mess = '%s: %.2f' % (classes_name, score)
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
        cv2.imwrite("demo.jpg", image)
    return image
