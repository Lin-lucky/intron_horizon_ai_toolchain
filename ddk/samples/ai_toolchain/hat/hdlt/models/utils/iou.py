import math

import torch
import torch.nn as nn
import torch.nn.functional as F

from ..registry import MODELS

__all__ = ["IOU", "GIOU", "CIOU"]


@MODELS.register_module
class IOU(nn.Module):
    r"""
    IOU.
    """

    def __init__(self):
        super(IOU, self).__init__()

    def _iou(self, pred, target, need_union=True):
        x2_min = torch.min(pred[..., 2], target[..., 2])
        y2_min = torch.min(pred[..., 3], target[..., 3])

        x1_max = torch.max(pred[..., 0], target[..., 0])
        y1_max = torch.max(pred[..., 1], target[..., 1])

        pred_w = pred[..., 2] - pred[..., 0]
        pred_h = pred[..., 3] - pred[..., 1]

        target_w = target[..., 2] - target[..., 0]
        target_h = target[..., 3] - target[..., 1]

        inter = (x2_min - x1_max).clamp(0) * (y2_min - y1_max).clamp(0)

        union = (pred_w * pred_h + target_w * target_h) - inter

        iou = inter / (union + 1e-7)
        if need_union:
            return iou, union
        else:
            return iou

    def forward(self, pred, target):
        return self._iou(pred, target, need_union=False)


@MODELS.register_module
class GIOU(IOU):
    r"""
    GIOU.
    """

    def _giou(self, pred, target):
        iou, union = self._iou(pred, target, need_union=True)
        x1_min = torch.min(pred[..., 0], target[..., 0])
        y1_min = torch.min(pred[..., 1], target[..., 1])

        x2_max = torch.max(pred[..., 2], target[..., 2])
        y2_max = torch.max(pred[..., 3], target[..., 3])

        area = (x2_max - x1_min).clamp(0) * (y2_max - y1_min).clamp(0)
        return iou - (area - union) / (area + 1e-7)

    def forward(self, pred, target):
        return self._giou(pred, target)


@MODELS.register_module
class CIOU(IOU):
    r"""
    CIOU.
    """

    def _ciou(self, pred, target):
        pred_w = (pred[..., 2] - pred[..., 0]).clamp(0)
        pred_h = (pred[..., 3] - pred[..., 1]).clamp(0) + 1e-16

        target_w = (target[..., 2] - target[..., 0]).clamp(0)
        target_h = (target[..., 3] - target[..., 1]).clamp(0) + 1e-16
        iou, _ = self._iou(pred, target, need_union=True)

        x1_min = torch.min(pred[..., 0], target[..., 0])
        y1_min = torch.min(pred[..., 1], target[..., 1])

        x2_max = torch.max(pred[..., 2], target[..., 2])
        y2_max = torch.max(pred[..., 3], target[..., 3])

        cw = x2_max - x1_min
        ch = y2_max - y1_min
        c2 = cw ** 2 + ch ** 2 + 1e-16

        pred_cx = (pred[..., 0] + pred[..., 2]) / 2
        pred_cy = (pred[..., 1] + pred[..., 3]) / 2

        target_cx = (target[..., 0] + target[..., 2]) / 2
        target_cy = (target[..., 1] + target[..., 3]) / 2
        rho2 = (target_cx - pred_cx) ** 2 + (target_cy - pred_cy) ** 2
        v = (4 / math.pi ** 2) * torch.pow(
            torch.atan(target_w / target_h) - torch.atan(pred_w / pred_h), 2
        )

        with torch.no_grad():
            alpha = v / (1 - iou + v + 1e-16)
        return iou - (rho2 / c2 + v * alpha)

    def forward(self, pred, target):
        return self._ciou(pred, target)
