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
import copy
import re
import time
import math
import colorsys
from easydict import EasyDict
from PIL import Image
from matplotlib import pyplot as plt
from multiprocessing.pool import ApplyResult
from horizon_tc_ui.utils import tool_utils
from coco_metric import MSCOCODetMetric


def postprocess(model_output,
                model_hw_shape,
                origin_image=None,
                origin_img_shape=None,
                score_threshold=0.5,
                nms_threshold=0.5,
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
    if origin_image is not None:
        ori_img_shape = origin_image.shape[1:-1]
    else:
        ori_img_shape = origin_img_shape

    eff_config = get_efficientdet_config(score_threshold=score_threshold,
                                         nms_threshold=nms_threshold)
    box, score, iminfo = result_seperation(model_output, ori_img_shape)
    bboxes_pr = efficientdet_post_process(box, score, iminfo, ori_img_shape,
                                          eff_config)

    if dump_image:
        print("detection item num: ", len(bboxes_pr))
        draw_bboxs(origin_image, bboxes_pr)
    return bboxes_pr


def eval_postprocess(model_output,
                     model_hw_shape,
                     entry_dict,
                     score_threshold=0.05,
                     nms_threshold=0.5):
    anno_info = entry_dict[0]
    origin_shape = anno_info['origin_shape']
    filename = anno_info['image_name']
    bboxes_pr = postprocess(model_output,
                            model_hw_shape,
                            origin_img_shape=origin_shape,
                            score_threshold=score_threshold,
                            nms_threshold=nms_threshold,
                            dump_image=False)
    pred_result = []
    for one_bbox in bboxes_pr:
        one_result = {'bbox': one_bbox, 'mask': False}
        pred_result.append(one_result)
    # eval_update(pred_result, filename, results, category_id)
    return pred_result, filename


def calc_accuracy(annotation_path, accuracy):
    metric = MSCOCODetMetric(annotation_path, with_mask=False)

    if isinstance(accuracy[0], ApplyResult):
        batch_result = [i.get() for i in accuracy]
    else:
        batch_result = list(accuracy)
    total_samples = 0
    with open("efficientdet_eval.log", "w") as eval_log_handle:
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


def result_seperation(outputs, origin_shape):
    input_height = 512
    input_width = 512
    box, scores = outputs[0:5], outputs[5:10]
    scale = min(input_width * 1.0 / origin_shape[1],
                input_height * 1.0 / origin_shape[0])
    new_h, new_w = scale * origin_shape[0], scale * origin_shape[1]
    new_h = np.ceil(new_h / 128.) * 128.
    new_w = np.ceil(new_w / 128.) * 128.
    im_info = np.array([new_h, new_w, scale]).astype(np.float32)

    return (box, scores, im_info)


def get_efficientdet_config(score_threshold, nms_threshold):
    efficientdet_config = EasyDict()
    efficientdet_config.scales = [512, 512]
    efficientdet_config.max_scale = 512
    efficientdet_config.nms_method = 'nms'
    efficientdet_config.nms_thresh = nms_threshold
    efficientdet_config.anchor_scales = [
        [4.0, 5.039684199579493, 6.3496042078727974],
        [4.0, 5.039684199579493, 6.3496042078727974],
        [4.0, 5.039684199579493, 6.3496042078727974],
        [4.0, 5.039684199579493, 6.3496042078727974],
        [4.0, 5.039684199579493, 6.3496042078727974]
    ]
    efficientdet_config.num_classes = 81
    efficientdet_config.batch_size_per_gpu = 1
    efficientdet_config.ratios = [0.5, 1, 2]
    efficientdet_config.pre_nms = 6000
    efficientdet_config.post_nms = 100
    efficientdet_config.score_thresh = -math.log(1 / score_threshold - 1)
    efficientdet_config.min_size = 0
    efficientdet_config.feature_strides = [8, 16, 32, 64, 128]
    efficientdet_config.anchors_table = []
    for i, stride in enumerate(efficientdet_config.feature_strides):
        alloc_size = int(
            np.ceil(efficientdet_config.max_scale * 1.0 /
                    efficientdet_config.feature_strides[i])) + 1
        efficientdet_config.anchors_table.append(
            anchor_generation(
                base_size=stride,
                ratios=efficientdet_config.ratios,
                scales=efficientdet_config.anchor_scales[i],
                feat_height=alloc_size,
                feat_width=alloc_size,
                feat_stride=stride,
            ))
    return efficientdet_config


def efficientdet_post_process(box, score, iminfo, origin_shape, eff_config):
    detections_lists = []
    box_scores = box
    box_regs = score
    im_info = iminfo
    det_count = 0
    for i in range(len(eff_config.feature_strides)):
        box_score = box_scores[i]  # (N, A, C, H, W)
        box_reg = box_regs[i]  # (N, A, 4, H, W)

        # model output layout is nhwc, so do nhwc2nchw before decode
        box_score = box_score.transpose((0, 3, 1, 2))
        box_reg = box_reg.transpose((0, 3, 1, 2))

        # generate anchors
        anchors = slice_anchor(eff_config.anchors_table[i],
                               box_reg)  # (1, HWA, 4)

        box_score_shape = box_score.shape
        target_box_score_shape = (box_score_shape[0], box_score_shape[1] //
                                  (eff_config.num_classes - 1),
                                  (eff_config.num_classes - 1),
                                  box_score_shape[2], box_score_shape[3])
        box_score = box_score.reshape(
            target_box_score_shape)  # (N, A, C, H, W)   # noqa
        box_score = box_score.transpose(
            (0, 3, 4, 1, 2))  # (N, H, W, A, C)  # noqa
        box_score = box_score.reshape(
            (1, -1, eff_config.num_classes - 1))  # (N, HWA, C)  # noqa

        box_reg_shape = box_reg.shape
        target_box_reg_shape = (box_reg_shape[0], box_reg_shape[1] // 4, 4,
                                box_reg_shape[2], box_reg_shape[3])

        box_reg = box_reg.reshape(
            target_box_reg_shape)  # (N, A, 4, H, W)  # noqa
        box_reg = box_reg.transpose((0, 3, 4, 1, 2))  # (N, H, W, A, 4)
        box_reg = box_reg.reshape((1, -1, 4))  # (N, HWA, 4)

        pred_boxes = pred_bbox(anchors, box_reg)  # (N, HWA*4)
        pred_boxes = pred_boxes.reshape((1, -1, 4))  # (N, HWA, 4)
        pred_boxes = box_clipper(pred_boxes, im_info)  # (N, HWA, 4)
        # get maximum scores and class ids
        max_scores = np.max(box_score, axis=-1)  # (N, HWA)
        classes = np.argmax(box_score, axis=-1) + 1  # (N, HWA)
        # concat
        mscore_shape = max_scores.shape
        max_scores = max_scores.reshape(
            (mscore_shape[0], mscore_shape[1], 1))  # (N, HWA, 1)
        classes_shape = classes.shape
        classes = classes.reshape(
            (classes_shape[0], classes_shape[1], 1))  # (N, HWA, 1)
        test_res = np.zeros(shape=(pred_boxes.shape[0] * pred_boxes.shape[1],
                                   6))

        for i in range(pred_boxes.shape[0]):
            for j in range(pred_boxes.shape[1]):
                test_res[j][0] = pred_boxes[i][j][0]
                test_res[j][1] = pred_boxes[i][j][1]
                test_res[j][2] = pred_boxes[i][j][2]
                test_res[j][3] = pred_boxes[i][j][3]
                test_res[j][4] = max_scores[i][j][0]
                test_res[j][5] = classes[i][j][0]

        test_res = select_box_by_score(test_res, eff_config.score_thresh)
        det_count += test_res.shape[0]
        detections_lists.append(test_res)

    detections = np.zeros(shape=(det_count, 6))
    det_index = 0
    for item in detections_lists:
        for item_index in range(item.shape[0]):
            detections[det_index][0] = item[item_index][0]
            detections[det_index][1] = item[item_index][1]
            detections[det_index][2] = item[item_index][2]
            detections[det_index][3] = item[item_index][3]
            detections[det_index][4] = item[item_index][4]
            detections[det_index][5] = item[item_index][5]
            det_index += 1

    output_retinanet = nms(
        detections,
        iou_threshold=eff_config.nms_thresh,
    )
    if not output_retinanet.size:
        return []
    keep = np.where(output_retinanet[:, -1] > -1)[0]
    if not keep.size:
        return []
    output_retinanet = output_retinanet[keep, :]
    output_retinanet[:, :4] /= im_info[2]
    output_retinanet[:, :4] = clip_boxes(output_retinanet[:, :4], origin_shape)
    # add 1 for height and width
    output_retinanet[:, 2:4] += 1.
    for i in range(output_retinanet.shape[0]):
        output_retinanet[i, -1] = int(output_retinanet[i, -1] - 1)
    return output_retinanet


def clip_boxes(boxes, im_shape):
    """
    Clip boxes to image boundaries.
    :param boxes: [N, 4* num_classes]
    :param im_shape: tuple of 2 (h, w)
    :return: [N, 4* num_classes]
    """
    # x1 >= 0
    boxes[:, 0::4] = np.maximum(np.minimum(boxes[:, 0::4], im_shape[1] - 1), 0)
    # y1 >= 0
    boxes[:, 1::4] = np.maximum(np.minimum(boxes[:, 1::4], im_shape[0] - 1), 0)
    # x2 < im_shape[1]
    boxes[:, 2::4] = np.maximum(np.minimum(boxes[:, 2::4], im_shape[1] - 1), 0)
    # y2 < im_shape[0]
    boxes[:, 3::4] = np.maximum(np.minimum(boxes[:, 3::4], im_shape[0] - 1), 0)
    return boxes


def get_classes(class_file_name='coco_classes.names'):
    '''loads class name from a file'''
    names = {}
    with open(class_file_name, 'r') as data:
        for ID, name in enumerate(data):
            names[ID] = name.strip('\n')
    return names


def draw_bboxs(input_image,
               bboxes,
               pic_name="demo.jpg",
               gt_classes_index=None,
               classes=get_classes()):
    """draw the bboxes in the original image
    """
    image = copy.deepcopy(input_image)
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
            class_index = int(bbox[5])
            score = bbox[4]
        else:
            class_index = gt_classes_index[i]
            score = 1

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
    cv2.imwrite(pic_name, image)
    return image


# def eval_update(pred_result, image_name, results, category_id):
#     assert isinstance(pred_result, list)
#     for pred in pred_result:
#         assert isinstance(pred, dict)
#         assert "bbox" in pred, "missing bbox for prediction"
#     if image_name in results:
#         warnings.warn("warning: you are overwriting {}".format(image_name))

#     parsed_name = image_name.strip()
#     parsed_name = parsed_name.split(".")[0]
#     image_id = int(parsed_name[-12:])
#     inst_list = []
#     for pred in pred_result:
#         coco_inst = {}
#         bbox = pred["bbox"].reshape((-1, ))
#         assert bbox.shape == (6, ), ("bbox should with shape (6,), get %s" %
#                                      bbox.shape)
#         coco_inst.update({
#             "image_id":
#                 image_id,
#             "category_id":
#                 category_id[int(bbox[5])],
#             "score":
#                 float(bbox[4]),
#             "bbox": [
#                 float(bbox[0]),
#                 float(bbox[1]),
#                 float(bbox[2] - bbox[0]),
#                 float(bbox[3] - bbox[1]),
#             ],
#         })
#         inst_list.append(coco_inst)
#     results[image_name] = inst_list
#     return results


def get_data_loader(image_path, label_path, transformers, batch_size=100):
    """Load the validation data."""
    data_loader = COCODataLoader(transformers,
                                 image_path,
                                 label_path,
                                 imread_mode='opencv')
    return data_loader


def expand_anchors(base_anchors, feat_height, feat_width, feat_stride):
    # generate proposals from shifted anchors
    shift_x = np.arange(0, feat_width) * feat_stride
    shift_y = np.arange(0, feat_height) * feat_stride
    shift_x, shift_y = np.meshgrid(shift_x, shift_y)
    shifts = np.vstack((shift_x.ravel(), shift_y.ravel(), shift_x.ravel(),
                        shift_y.ravel())).transpose()
    # add A anchors (1, A, 4) to
    # cell k shifts(k, 1, 4) to
    # get shift anchors (K, A, 4)
    A = base_anchors.shape[0]
    K = shifts.shape[0]
    base_anchors_all = base_anchors.reshape((1, A, 4)) + shifts.reshape(
        (1, K, 4)).transpose((1, 0, 2))
    base_anchors_all = base_anchors_all.reshape((K * A, 4))
    return base_anchors_all


def generate_anchors(base_size, ratios, scales):
    if isinstance(base_size, list) or isinstance(base_size, tuple):
        w = base_size[0]
        h = base_size[1]
    else:
        w = h = base_size
    size = w * h
    x_ctr = 0.5 * (w - 1.0)
    y_ctr = 0.5 * (h - 1.0)
    anchors = []
    for ratio in ratios:
        for scale in scales:
            size_ratio = math.floor(size / float(ratio))
            new_w = math.floor(math.sqrt(size_ratio) + 0.5) * scale
            new_h = math.floor(new_w / scale * ratio + 0.5) * scale
            anchors.append([
                x_ctr - 0.5 * (new_w - 1.0), y_ctr - 0.5 * (new_h - 1.0),
                x_ctr + 0.5 * (new_w - 1.0), y_ctr + 0.5 * (new_h - 1.0)
            ])
    return np.array(anchors)


def anchor_generation(base_size, ratios, scales, feat_height, feat_width,
                      feat_stride):

    base_anchors = generate_anchors(base_size, ratios, scales)
    anchors = expand_anchors(base_anchors, feat_height, feat_width,
                             feat_stride)
    # (H*W*A, 4) --> (1, 1, H, W, A*4)
    anchors = anchors.reshape((1, 1, feat_height, feat_width, -1))
    return anchors


def slice_anchor(anchor, input_x):
    input_shape = input_x.shape
    new_anchor = anchor[:, :, :input_shape[2], :input_shape[3], :]
    return new_anchor.reshape(1, -1, 4)


def select_box_by_score(boxes, score_threshold):
    score_mask = boxes[:, 4] > score_threshold
    boxes = boxes[score_mask]
    return boxes


def pred_bbox(boxes, box_deltas):
    boxes = boxes.reshape((1, -1, 4))
    x1, y1, x2, y2 = np.split(boxes, 4, axis=2)

    widths = x2 - x1 + 1.0
    heights = y2 - y1 + 1.0
    ctr_x = x1 + 0.5 * (widths - 1.0)
    ctr_y = y1 + 0.5 * (heights - 1.0)

    deltas = box_deltas.reshape((1, -1, 4))
    dx, dy, dw, dh = np.split(deltas, 4, axis=2)
    pred_ctr_x = np.add(np.multiply(dx, widths), ctr_x)
    pred_ctr_y = np.add(np.multiply(dy, heights), ctr_y)
    pred_w = np.multiply(np.exp(dw), widths)
    pred_h = np.multiply(np.exp(dh), heights)
    pred_x1 = pred_ctr_x - 0.5 * (pred_w - 1.0)
    pred_y1 = pred_ctr_y - 0.5 * (pred_h - 1.0)
    pred_x2 = pred_ctr_x + 0.5 * (pred_w - 1.0)
    pred_y2 = pred_ctr_y + 0.5 * (pred_h - 1.0)

    pred_boxes = np.zeros(shape=(1, pred_x1.shape[1], 4))
    for index in range(pred_x1.shape[1]):
        pred_boxes[0][index][0] = pred_x1[0][index][0]
        pred_boxes[0][index][1] = pred_y1[0][index][0]
        pred_boxes[0][index][2] = pred_x2[0][index][0]
        pred_boxes[0][index][3] = pred_y2[0][index][0]
    pred_boxes = pred_boxes.reshape((1, -1))
    return pred_boxes


def box_clipper(pred_boxes, im_info):
    pred_boxes = np.maximum(pred_boxes, 0.0)
    clip_info = np.array([im_info[1], im_info[0], im_info[1],
                          im_info[0]]).astype(np.float32)
    return np.minimum(pred_boxes, clip_info)


def nms(bboxes, iou_threshold, sigma=0.3, method='nms', topk=100):
    """
    calculate the nms for bboxes
    """
    # iou_threshold = 0.5
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

            for index in range(len(iou)):
                if iou[index] > iou_threshold:
                    cls_bboxes[index, 4] = -100
            # iou_mask = iou > iou_threshold
            # weight[iou_mask] = -100
            # cls_bboxes[:, 4] = cls_bboxes[:, 4] * weight

            score_mask = cls_bboxes[:, 4] > -100
            cls_bboxes = cls_bboxes[score_mask]

    if not best_bboxes:
        return np.array([])
    best_bboxes = np.array(best_bboxes)
    best_bboxes = best_bboxes[best_bboxes[:, 4].argsort()][::-1][:100]
    return best_bboxes


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
    ious = np.ones(shape=inter_area.shape)
    for index in range(len(inter_area)):
        if union_area[index] != 0:
            ious[index] = max(1.0 * inter_area[index] / union_area[index],
                              np.finfo(np.float32).eps)
    return ious
