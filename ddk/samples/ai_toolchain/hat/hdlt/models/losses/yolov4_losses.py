# Copyright (c) Horizon Robotics. All rights reserved.

import torch
import torch.nn as nn
import torch.nn.functional as F

import math
from hdlt.common.registry import build_from_cfg
from ..registry import MODELS


__all__ = ["Yolov4Loss"]


@MODELS.register_module
class Yolov4Loss(nn.Module):
    def __init__(self, obj_loss, bbox_loss, cls_loss, iou_ratio = 1.0, balance=[4.0, 1.0, 0.4], scales=[1.0, 1.0, 1.0]):
        super(Yolov4Loss, self).__init__()
        self.balance = balance
        self.scales = scales
        self.iou_ratio = iou_ratio
        self.bbox_loss = build_from_cfg(bbox_loss, MODELS)
        self.obj_loss = build_from_cfg(obj_loss, MODELS)
        self.cls_loss = build_from_cfg(cls_loss, MODELS)

    def _decode(self, bbox_pred, anchor):
        pxy = bbox_pred[..., 0:2]
        pxy = pxy.sigmoid() * 2 - 0.5

        pwh = bbox_pred[..., 2:4]
        pwh = (pwh.sigmoid() * 2) **  2 * anchor
        return torch.cat([pxy, pwh], dim=1)
    
    def _center2bbox(self, target):
        xmin = target[..., 0] - target[..., 2] / 2
        ymin = target[..., 1] - target[..., 3] / 2
        xmax = target[..., 0] + target[..., 2] / 2
        ymax = target[..., 1] + target[..., 3] / 2
        return torch.stack([xmin, ymin, xmax, ymax], dim=-1)

 
    def forward(self, preds, targets, anchors):
        indexes, labels = targets
        layers = len(preds)
        bs = preds[0].shape[0]
        obj_losses = torch.zeros(1, device=preds[0].device)
        bbox_losses = torch.zeros(1, device=preds[0].device)
        cls_losses = torch.zeros(1, device=preds[0].device)
        for i, pred, index, label, anchor in zip(range(layers), preds, indexes, labels, anchors):
            pred_shape = pred.shape
            pred = pred.view(pred_shape[0], pred_shape[1] , pred_shape[2], anchor.shape[0],  -1)
            obj_pred = pred[..., 4] 
            obj_target = torch.zeros_like(obj_pred, device=pred.device)
            if not index.shape[0]:
                bbox_loss = torch.tensor(0, device=pred.device)
                cls_loss = torch.tensor(0, device=pred.device)
                obj_loss = self.obj_loss(obj_pred, obj_target) 
            else:
                b, a, y, x  = index.long().T
                pred = pred[b, y, x, a]

                bbox_pred = pred[..., 0:4]
                bbox_target = label[..., 0:4]
                scale = torch.tensor([pred_shape[2], pred_shape[1]], device=pred.device)
                anchor = anchor.to(pred.device) * scale
                anchor = anchor[a]
                bbox_pred = self._decode(bbox_pred, anchor)

                bbox_pred = self._center2bbox(bbox_pred)
                bbox_target = self._center2bbox(bbox_target)
                iou = self.bbox_loss(bbox_pred, bbox_target)
                cls_pred = pred[..., 5:]
                cls_target = label[..., 4]
                nc = cls_pred.shape[-1]
                cls_target = F.one_hot(cls_target.long(), nc).float()
                cls_loss = self.cls_loss(cls_pred, cls_target)
                
                obj_target[b, y, x, a] = (1.0 - self.iou_ratio) + self.iou_ratio * iou.detach().clamp(0)
                obj_loss = self.obj_loss(obj_pred, obj_target)
                bbox_loss = (1.0 - iou).mean()              
            
            obj_losses += obj_loss * self.balance[i]
            bbox_losses += bbox_loss
            cls_losses += cls_loss
        bbox_losses *= self.scales[0] *bs
        obj_losses *= self.scales[1] * bs
        cls_losses *= self.scales[2] * bs
        return dict({"obj": obj_losses , "cls": cls_losses, "bbox": bbox_losses})

