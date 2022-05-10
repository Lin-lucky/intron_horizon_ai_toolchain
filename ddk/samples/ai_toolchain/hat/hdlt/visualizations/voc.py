# Copyright (c) Horizon Robotics. All rights reserved.
import numpy as np
import torch
from matplotlib import pyplot as plt

from hdlt.visualizations.registry import VISULIZATIONS

from .mscoco import plot_bbox

VOC_CLASSES = (
    "aeroplane",
    "bicycle",
    "bird",
    "boat",
    "bottle",
    "bus",
    "car",
    "cat",
    "chair",
    "cow",
    "diningtable",
    "dog",
    "horse",
    "motorbike",
    "person",
    "pottedplant",
    "sheep",
    "sofa",
    "train",
    "tvmonitor",
)


@VISULIZATIONS.register_module
class VocVisualization(object):
    def __call__(
        self, image, target, thresh=0.5, absolute_coord=True, reverse_rgb=False
    ):
        # only support viz one image at each time
        if image.ndim == 4:
            image = image[0]
        if isinstance(target, (list, tuple)):
            target = target[0][0]
        elif target.ndim == 3:
            target = target[0]
        image = image.detach().cpu().numpy().astype(np.uint8)
        image = image.transpose((1, 2, 0))
        target = target.detach().cpu().numpy().astype(np.float32)

        bboxes = target[:, :4]
        labels = target[:, 4]
        scores = None  # target[:, 5]

        plot_bbox(
            img=image,
            bboxes=bboxes,
            labels=labels,
            scores=scores,
            class_names=VOC_CLASSES,
            thresh=thresh,
            reverse_rgb=reverse_rgb,
            absolute_coordinates=absolute_coord,
        )
        plt.show()
        return self
