# Copyright (c) Horizon Robotics. All rights reserved.

import math

import torch
import torch.nn as nn
import torch.nn.functional as F

from hdlt.common.registry import build_from_cfg
from hdlt.models.utils import IOU

from ..registry import MODELS

__all__ = ["FCOSLoss"]


@MODELS.register_module
class FCOSLoss(nn.Module):
    """
    Fcos loss.

    Args:
        cls_loss (dict):
            Cls loss setting.
        bbox_loss (dict):
            BBox loss setting.
        centerness_loss (dict):
            Centerness loss setting.
        centerness_iou (bool):
            Whether use iou as centerness target.
    """

    def __init__(
        self, cls_loss, bbox_loss, centerness_loss=None, centerness_iou=False
    ):
        super(FCOSLoss, self).__init__()
        self.bbox_loss = build_from_cfg(bbox_loss, MODELS)
        if centerness_loss is not None:
            self.centerness_loss = build_from_cfg(centerness_loss, MODELS)
        self.cls_loss = build_from_cfg(cls_loss, MODELS)
        self.centerness_iou = centerness_iou
        self.iou_func = IOU()

    def _centerness_target(self, bbox_targets):
        left_right = bbox_targets[:, [0, 2]]
        top_bottom = bbox_targets[:, [1, 3]]
        centerness_targets = (
            left_right.min(dim=-1)[0] / left_right.max(dim=-1)[0]
        ) * (top_bottom.min(dim=-1)[0] / top_bottom.max(dim=-1)[0])
        return torch.sqrt(centerness_targets)

    def _distance2bbox(self, points, distance):
        x1 = points[:, 0] - distance[:, 0]
        y1 = points[:, 1] - distance[:, 1]
        x2 = points[:, 0] + distance[:, 2]
        y2 = points[:, 1] + distance[:, 3]
        return torch.stack([x1, y1, x2, y2], -1)

    def forward(self, preds, targets):
        bbox_targets, cls_targets = targets

        num_feat = len(preds)
        if num_feat == 3:
            cls_preds, bbox_preds, centerness_preds = preds
        else:
            cls_preds, bbox_preds = preds

        nc = cls_preds[0].shape[3]
        bs = cls_preds[0].shape[0]

        num_layer = len(bbox_preds)
        total_gts = 0
        bbox_norm = torch.zeros(1, device=preds[0][0].device)

        bbox_pred_decoded_list = list()
        bbox_target_decoded_list = list()
        bbox_weight_list = list()

        centerness_pred_list = list()
        centerness_target_list = list()

        cls_pred_list = list()
        cls_target_list = list()

        for i, bbox_pred, cls_pred, bbox_target, cls_target in zip(
            range(len(bbox_preds)),
            bbox_preds,
            cls_preds,
            bbox_targets,
            cls_targets,
        ):
            index = (cls_target >= 0) & (cls_target < nc)

            bbox_target = bbox_target[index]
            num_gt = bbox_target.shape[0]
            total_gts += num_gt
            if num_gt:
                centerness_target = self._centerness_target(bbox_target)
                bbox_weight_list.append(centerness_target)

                bbox_pred = bbox_pred[index]
                points = torch.zeros(
                    (bbox_pred.shape[0], 2), device=bbox_pred.device
                )
                bbox_pred_decoded = self._distance2bbox(points, bbox_pred)
                bbox_target_decoded = self._distance2bbox(points, bbox_target)
                bbox_pred_decoded_list.append(bbox_pred_decoded)
                bbox_target_decoded_list.append(bbox_target_decoded)

                if num_feat == 3:
                    if self.centerness_iou:
                        iou = self.iou_func(
                            bbox_pred_decoded, bbox_target_decoded
                        )
                        centerness_target = iou.detach().clamp(0)
                    centerness_pred = centerness_preds[i]
                    centerness_pred = centerness_pred[index]
                    centerness_pred = centerness_pred.view(-1)
                    centerness_pred_list.append(centerness_pred)
                    centerness_target_list.append(centerness_target)
            cls_target = F.one_hot(cls_target.long(), nc + 1).float()
            cls_target = cls_target[..., :nc]
            cls_pred_list.append(cls_pred.view((-1, nc)))
            cls_target_list.append(cls_target.view((-1, nc)))

        cls_preds = torch.cat(cls_pred_list, dim=0)
        cls_targets = torch.cat(cls_target_list, dim=0)
        cls_losses = self.cls_loss(
            cls_preds, cls_targets, avg_factor=total_gts + bs
        )

        if total_gts != 0:
            bbox_preds_decoded = torch.cat(bbox_pred_decoded_list, dim=0)
            bbox_targets_decoded = torch.cat(bbox_target_decoded_list, dim=0)
            bbox_weights = torch.cat(bbox_weight_list, dim=0)

            bbox_losses = self.bbox_loss(
                bbox_preds_decoded,
                bbox_targets_decoded,
                weight=bbox_weights,
                avg_factor=bbox_weights.sum(),
            )
            if num_feat == 3:
                centerness_preds = torch.cat(centerness_pred_list, dim=0)
                centerness_targets = torch.cat(centerness_target_list, dim=0)
                centerness_losses = self.centerness_loss(
                    centerness_preds, centerness_targets
                )
        else:
            bbox_losses = torch.zeros(1, device=preds[0][0].device)
            if num_feat == 3:
                centerness_loss = torch.tensor(0, device=bbox_pred.device)
        if num_feat == 3:
            return dict(
                {
                    "cls": cls_losses,
                    "bbox": bbox_losses,
                    "centerness": centerness_losses,
                }
            )
        else:
            return dict({"cls": cls_losses, "bbox": bbox_losses})
