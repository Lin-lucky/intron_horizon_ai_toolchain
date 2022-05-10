# Copyright (c) Horizon Robotics. All rights reserved.
# Adopt from
# https://github.com/dmlc/gluon-cv/blob/master/gluoncv/utils/viz/image.py

import random
from typing import List

import numpy as np
import torch
from matplotlib import pyplot as plt
from torch import Tensor

from hdlt.visualizations.registry import VISULIZATIONS

MSCOCO_CLASSES = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]


def plot_image(img: np.array, ax=None, reverse_rgb=False):
    """Visualize image.

    Parameters
    ----------
    img : numpy.ndarray
        Image with shape `H, W, 3`.
    ax : matplotlib axes, optional
        You can reuse previous axes if provided.
    reverse_rgb : bool, optional
        Reverse RGB<->BGR orders if `True`.

    Returns
    -------
    matplotlib axes
        The ploted axes.

    Examples
    --------

    from matplotlib import pyplot as plt
    ax = plot_image(img)
    plt.show()
    """

    assert isinstance(img, np.ndarray)
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
    img = img.copy()
    if reverse_rgb:
        img[:, :, (0, 1, 2)] = img[:, :, (2, 1, 0)]
    ax.imshow(img.astype(np.uint8))
    return ax


def plot_bbox(
    img: np.array,
    bboxes: np.array,
    scores: np.array = None,
    labels: np.array = None,
    thresh: float = 0.5,
    class_names: List[str] = None,
    colors=None,
    ax=None,
    reverse_rgb=False,
    absolute_coordinates=True,
):
    """Visualize bounding boxes.

    Parameters
    ----------
    img : numpy.ndarray
        Image with shape `H, W, 3`.
    bboxes : numpy.ndarray
        Bounding boxes with shape `N, 4`. Where `N` is the number of boxes.
    scores : numpy.ndarray, optional
        Confidence scores of the provided `bboxes` with shape `N`.
    labels : numpy.ndarray, optional
        Class labels of the provided `bboxes` with shape `N`.
    thresh : float, optional, default 0.5
        Display threshold if `scores` is provided. Scores with less than
        `thresh` will be ignored in display, this is visually more elegant
        if you have a large number of bounding boxes with very small scores.
    class_names : list of str, optional
        Description of parameter `class_names`.
    colors : dict, optional
        You can provide desired colors as {0: (255, 0, 0), 1:(0, 255, 0), ...},
        otherwise random colors will be substituted.
    ax : matplotlib axes, optional
        You can reuse previous axes if provided.
    reverse_rgb : bool, optional
        Reverse RGB<->BGR orders if `True`.
    absolute_coordinates : bool
        If `True`, absolute coordinates will be considered, otherwise
        coordinates are interpreted as in range(0, 1).

    Returns
    -------
    matplotlib axes
        The ploted axes.

    """
    if labels is not None and not len(bboxes) == len(labels):
        raise ValueError(
            "The length of labels and bboxes mismatch, {} vs {}".format(
                len(labels), len(bboxes)
            )
        )
    if scores is not None and not len(bboxes) == len(scores):
        raise ValueError(
            "The length of scores and bboxes mismatch, {} vs {}".format(
                len(scores), len(bboxes)
            )
        )

    ax = plot_image(img, ax=ax, reverse_rgb=reverse_rgb)

    if len(bboxes) < 1:
        return ax

    if not absolute_coordinates:
        # convert to absolute coordinates using image shape
        height = img.shape[0]
        width = img.shape[1]
        bboxes[:, (0, 2)] *= height
        bboxes[:, (1, 3)] *= width

    # use random colors if None is provided
    if colors is None:
        colors = dict()
    for i, bbox in enumerate(bboxes):
        if scores is not None and scores.flat[i] < thresh:
            continue
        if labels is not None and labels.flat[i] < 0:
            continue
        cls_id = int(labels.flat[i]) if labels is not None else -1
        if cls_id not in colors:
            if class_names is not None:
                colors[cls_id] = plt.get_cmap("hsv")(cls_id / len(class_names))
            else:
                colors[cls_id] = (
                    random.random(),
                    random.random(),
                    random.random(),
                )
        xmin, ymin, xmax, ymax = [int(x) for x in bbox]
        rect = plt.Rectangle(
            (xmin, ymin),
            xmax - xmin,
            ymax - ymin,
            fill=False,
            edgecolor=colors[cls_id],
            linewidth=3.5,
        )
        ax.add_patch(rect)
        if class_names is not None and cls_id < len(class_names):
            class_name = class_names[cls_id]
        else:
            class_name = str(cls_id) if cls_id >= 0 else ""
        score = "{:.3f}".format(scores.flat[i]) if scores is not None else ""
        if class_name or score:
            ax.text(
                xmin,
                ymin - 2,
                "{:s} {:s}".format(class_name, score),
                bbox=dict(facecolor=colors[cls_id], alpha=0.5),
                fontsize=12,
                color="white",
            )
    return ax


@VISULIZATIONS.register_module
class CocoVisualization(object):
    def __call__(self, image: Tensor, target):
        if image.ndim == 4:
            image = image[0]
        image = image.permute(1, 2, 0)
        image = image.detach().cpu().numpy().astype(np.uint8)
        if isinstance(target, (list, tuple)):
            target = target[0][0]
        if target.ndim == 3:
            target = target[0]
        target = target.detach().cpu().numpy().astype(np.float32)
        if target.shape[-1] == 6:
            bboxes = target[:, :4]
            scores = target[:, 4]
            labels = target[:, 5]
        elif target.shape[-1] == 5:
            bboxes = target[:, :4]
            scores = None
            labels = target[:, 4]
        plot_bbox(
            img=image,
            bboxes=bboxes,
            labels=labels,
            scores=scores,
            class_names=MSCOCO_CLASSES,
        )
        plt.show()
        return self
