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
from PIL import Image
from matplotlib import pyplot as plt
from multiprocessing.pool import ApplyResult
from cityscapes_metric import SegmentationMetric


def plot_image(origin_image, onnx_output):
    def get_pallete():
        pallete = [
            128,
            64,
            128,
            244,
            35,
            232,
            70,
            70,
            70,
            102,
            102,
            156,
            190,
            153,
            153,
            153,
            153,
            153,
            250,
            170,
            30,
            220,
            220,
            0,
            107,
            142,
            35,
            152,
            251,
            152,
            0,
            130,
            180,
            220,
            20,
            60,
            255,
            0,
            0,
            0,
            0,
            142,
            0,
            0,
            70,
            0,
            60,
            100,
            0,
            80,
            100,
            0,
            0,
            230,
            119,
            11,
            32,
        ]
        return pallete

    onnx_output = onnx_output[0].astype(np.uint8)
    onnx_output = np.squeeze(onnx_output)

    image_shape = origin_image.shape[:2][::-1]

    onnx_output = np.expand_dims(onnx_output, axis=2)
    onnx_output = cv2.resize(onnx_output,
                             image_shape,
                             interpolation=cv2.INTER_NEAREST)
    out_img = Image.fromarray(onnx_output)
    out_img.putpalette(get_pallete())
    plt.imshow(origin_image)
    plt.imshow(out_img, alpha=0.6)
    fig_name = 'demo.jpg'
    print(f"save predicting image with name {fig_name} ")
    plt.savefig(fig_name)


def postprocess(model_output, origin_image):
    pred_result = np.argmax(model_output[0], axis=-1)
    origin_image = np.squeeze(origin_image, axis=0)
    plot_image(origin_image, pred_result)


def eval_postprocess(model_output, label, input_shape):
    onnx_output = np.argmax(model_output[0], axis=-1)
    onnx_output = np.squeeze(onnx_output)
    onnx_output = np.expand_dims(onnx_output, axis=-1)

    pred_seg = cv2.resize(onnx_output, (input_shape[1], input_shape[0]),
                          interpolation=cv2.INTER_NEAREST)
    gt_seg = np.squeeze(label)
    return pred_seg, gt_seg


def calc_accuracy(accuracy):
    metric = SegmentationMetric(nclass=19)
    if isinstance(accuracy[0], ApplyResult):
        batch_result = [i.get() for i in accuracy]
    else:
        batch_result = list(accuracy)
    total_samples = 0
    with open("mobilenet_unet_eval.log", "w") as eval_log_handle:
        logging.info("start to calc eval info...")
        for pred_seg, gt_seg, filename in batch_result:
            metric.update(gt_seg, pred_seg)
            total_samples += 1
            if total_samples % 10 == 0:
                accu, mIoU = metric.get()
                logging.info(
                    f"pixel accuracy:{accu:.4f}, mIoU:{mIoU:.6f} - {total_samples}/{len(batch_result)}"
                )
            pred_seg = pred_seg.reshape(-1, )
            seg_count = 0
            block_count = 0
            eval_log_handle.write(
                f"input_image_name: {filename[0].split('/')[-1]}_block_{block_count}: "
            )
            for seg_item in pred_seg:
                eval_log_handle.write(f"{seg_item} ")
                seg_count += 1
                if seg_count >= 8000:
                    seg_count = 0
                    block_count += 1
                    eval_log_handle.write(
                        f"input_image_name: {filename[0].split('/')[-1]}_block_{block_count}: "
                    )
            eval_log_handle.write(f"\n")

    return metric.get()
