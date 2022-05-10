# Copyright (c) Horizon Robotics. All rights reserved.

import numpy as np
import torch

__all__ = ["compute_ious"]


def compute_ious(bboxes_a, bboxes_b):
    """compte ious"""
    assert type(bboxes_a) == type(bboxes_b)
    tensor_type = type(bboxes_a)

    if isinstance(bboxes_a, np.ndarray):
        bboxes_a = torch.from_numpy(bboxes_a)
        bboxes_b = torch.from_numpy(bboxes_b)

    area_a = (bboxes_a[:, 2] - bboxes_a[:, 0]) * (
        bboxes_a[:, 3] - bboxes_a[:, 1]
    )
    area_b = (bboxes_b[:, 2] - bboxes_b[:, 0]) * (
        bboxes_b[:, 3] - bboxes_b[:, 1]
    )

    iw = torch.min(
        torch.unsqueeze(bboxes_a[:, 2], dim=1), bboxes_b[:, 2]
    ) - torch.max(torch.unsqueeze(bboxes_a[:, 0], dim=1), bboxes_b[:, 0])
    ih = torch.min(
        torch.unsqueeze(bboxes_a[:, 3], dim=1), bboxes_b[:, 3]
    ) - torch.max(torch.unsqueeze(bboxes_a[:, 1], dim=1), bboxes_b[:, 1])

    iw = torch.clamp(iw, min=0)
    ih = torch.clamp(ih, min=0)

    ua = torch.unsqueeze(area_a, dim=1) + area_b - iw * ih
    ua = torch.clamp(ua, min=1e-8)
    intersection = iw * ih
    IoU = intersection / ua

    if tensor_type == np.ndarray:
        IoU = IoU.numpy()
    return IoU
