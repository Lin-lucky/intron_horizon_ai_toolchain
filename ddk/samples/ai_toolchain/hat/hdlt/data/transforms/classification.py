# Copyright (c) Horizon Robotics. All rights reserved.

import cv2
import torch
import torch.nn as nn

from ..registry import TRANSFORMS

__all__ = ["ConvertLayout", "OneHot", "LabelSmooth"]


@TRANSFORMS.register_module
class ConvertLayout(object):
    """
    ConvertLayout is used for layout convert.

    Args:
        hwc2chw (bool): Whether to convert hwc to chw.
    """

    def __init__(self, hwc2chw=True):
        self.hwc2chw = hwc2chw

    def __call__(self, image):
        if self.hwc2chw:
            image = image.permute((2, 0, 1))
        else:
            image = image.permute((1, 2, 0))
        return image


@TRANSFORMS.register_module
class OneHot(object):
    """
    OneHot is used for convert layer to one-hot format.

    Args:
        num_classes (int): Num classes.
    """

    def __init__(self, num_classes):
        self.num_classes = num_classes

    def __call__(self, image, target):
        if len(target.shape) == 1:
            # for scatter
            target = torch.unsqueeze(target, dim=1)
        batch_size = target.shape[0]
        one_hot = torch.zeros(batch_size, self.num_classes)
        one_hot = one_hot.scatter_(1, target, 1)
        return image, one_hot


@TRANSFORMS.register_module
class LabelSmooth(object):
    """
    LabelSmooth is used for label smooth.

    Args:
        num_classes (int): Num classes.
        eta (float): Eta of label smooth.
    """

    def __init__(self, num_classes, eta=0.1):
        self.num_classes = num_classes
        self.on_value = torch.tensor([1 - eta + eta / num_classes])
        self.off_value = torch.tensor([eta / num_classes])

    def __call__(self, image, target):
        if len(target.shape) == 1:
            # for scatter
            target = torch.unsqueeze(target, dim=1)
        batch_size = target.shape[0]
        one_hot = torch.zeros(batch_size, self.num_classes)
        ont_hot = one_hot.scatter_(1, target, 1)
        target = torch.where(one_hot == 0, self.off_value, self.on_value)
        return image, target
