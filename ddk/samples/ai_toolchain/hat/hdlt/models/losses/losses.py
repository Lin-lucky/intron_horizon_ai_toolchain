# Copyright (c) Horizon Robotics. All rights reserved.

from typing import Sequence, Union

import torch
import torch.nn.functional as F

from hdlt.common.registry import build_from_cfg

from ...data.transforms.segmentation import _to_one_hot
from ..registry import MODELS

__all__ = [
    "CEWithLabelSmooth",
    "FocalLoss",
    "CEWithWithLogitsLoss",
    "IOULoss",
    "L1Loss",
    "SequenceFocalLoss",
    "SmoothL1Loss",
]


@MODELS.register_module
class CEWithLabelSmooth(torch.nn.CrossEntropyLoss):
    """
    The losses of cross-entropy with label smooth.

    Args:
        smooth_alpha (float): Alpha of label smooth.
    """

    def __init__(self, smooth_alpha=0.1):
        super(CEWithLabelSmooth, self).__init__()
        self.smooth_alpha = smooth_alpha

    def forward(self, input, target):
        n = input.size()[-1]
        log_pred = F.log_softmax(input, dim=-1)
        loss = -log_pred.sum(dim=-1).mean()
        nll = F.nll_loss(log_pred, target, reduction="mean")
        sa = self.smooth_alpha
        return sa * (loss / n) + (1 - sa) * nll


@MODELS.register_module
class SequenceFocalLoss(torch.nn.Module):
    r"""
    Sequence Focal Loss.

    Args:
        num_classes (int): Class number.
        alpha (float, optional): Alpha. Defaults to 0.25.
        gamma (float, optional): Gamma. Defaults to 2.0.
        reduction (str, optional):
            Specifies the reduction to apply to the output:
            ``'none'`` | ``'mean'`` | ``'sum'``. Defaults to ``'mean'``.
        weight (Union[float, Sequence], optional): Weight to be applied to
            the loss of each input. Defaults to 1.0.
    """

    def __init__(
        self,
        num_classes: int,
        alpha: float = 0.25,
        gamma: float = 2.0,
        reduction: str = "mean",
        weight: Union[float, Sequence] = 1.0,
    ):
        super(SequenceFocalLoss, self).__init__()
        self.num_classes = num_classes
        self.alpha = alpha
        self.gamma = gamma
        self.reduction = reduction
        self.weight = weight

    def focal_loss(self, logits, label):
        one_hot = _to_one_hot(label, self.num_classes)

        probs = torch.softmax(logits, dim=1)
        probs = probs + 1e-9
        ce = (-torch.log(probs)) * one_hot
        weight = torch.pow(1.0 - probs, self.gamma)

        loss = ce * weight * self.alpha
        loss = loss.sum(dim=1)

        if self.reduction == "mean":
            return loss.mean()
        elif self.reduction == "sum":
            return loss.sum()
        else:
            return loss

    def forward(self, logits, labels):
        if isinstance(logits, Sequence):
            if not isinstance(self.weight, Sequence):
                weights = [self.weight] * len(logits)
            else:
                weights = self.weight

            return [
                self.focal_loss(logit, label) * w
                for logit, label, w in zip(logits, labels, weights)
            ]
        else:
            return self.focal_loss(logits, labels) * self.weight


@MODELS.register_module
class FocalLoss(torch.nn.Module):
    r"""
    Focal Loss.

    Args:
        loss_fcn (dict): Dict for loss.
        alpha (float, optional): Alpha. Defaults to 0.25.
        gamma (float, optional): Gamma. Defaults to 2.0.
        reduction (str, optional):
            Specifies the reduction to apply to the output:
            ``'none'`` | ``'mean'`` | ``'sum'``. Defaults to ``'mean'``.
        weight (Union[float, Sequence], optional): Weight to be applied to
            the loss of each input. Defaults to 1.0.

    """

    def __init__(self, loss_fcn, gamma=1.5, alpha=0.25, reduction="mean"):
        super(FocalLoss, self).__init__()
        loss_fcn["reduction"] = "none"
        self.loss_fcn = build_from_cfg(loss_fcn, MODELS)
        self.gamma = gamma
        self.alpha = alpha
        self.reduction = reduction

    def forward(self, pred, true, weight=None, avg_factor=None):
        loss = self.loss_fcn(pred, true)
        pred_prob = torch.sigmoid(pred)  # prob from logits
        p_t = true * pred_prob + (1 - true) * (1 - pred_prob)
        alpha_factor = true * self.alpha + (1 - true) * (1 - self.alpha)
        modulating_factor = (1.0 - p_t) ** self.gamma
        loss *= alpha_factor * modulating_factor

        if weight is not None:
            loss *= weight
        if self.reduction == "mean":
            if avg_factor is not None:
                loss = loss.sum()
                return loss / avg_factor
            else:
                return loss.mean()
        elif self.reduction == "sum":
            return loss.sum()
        else:  # 'none'
            return loss


@MODELS.register_module
class CEWithWithLogitsLoss(torch.nn.Module):
    r"""
    CEWithWithLogitsLoss.

    Args:
        weights (float): Weights for positvie class.
        eps (float, optional): Eps for label smooth.
        reduction (str, optional):
            Specifies the reduction to apply to the output:
            ``'none'`` | ``'mean'`` | ``'sum'``. Defaults to ``'mean'``.
    """

    def __init__(self, weights=1.0, eps=0.0, reduction="mean"):
        super(CEWithWithLogitsLoss, self).__init__()
        self.loss = torch.nn.BCEWithLogitsLoss(
            pos_weight=torch.Tensor([weights]), reduction=reduction
        )
        self.eps = eps

    def forward(self, pred, target):
        if self.eps > 0.0:
            target = torch.where(
                target == 1.0, 1.0 - 0.5 * self.eps, 0.5 * self.eps
            )
        return self.loss(pred, target)


@MODELS.register_module
class IOULoss(torch.nn.Module):
    r"""
    IOULoss.

    Args:
        iou_cfg (dict): Dict for iou..
        reduction (str, optional):
            Specifies the reduction to apply to the output:
            ``'none'`` | ``'mean'`` | ``'sum'``. Defaults to ``'mean'``.
    """

    def __init__(self, iou_cfg, reduction="mean"):
        super(IOULoss, self).__init__()
        self.reduction = reduction
        self.iou_func = build_from_cfg(iou_cfg, MODELS)

    def forward(self, pred, target, iou=None, weight=None, avg_factor=None):
        if iou is None:
            iou = self.iou_func(pred, target)
        loss = 1.0 - iou
        if weight is not None:
            loss *= weight

        if self.reduction != "none":
            loss = loss.sum()

        if self.reduction == "mean":
            if avg_factor is None:
                loss = loss.mean()
            else:
                loss = loss / avg_factor

        return loss


@MODELS.register_module
class L1Loss(torch.nn.Module):
    """L1 loss.

    Args:
        reduction (str, optional): The method to reduce the loss.
            Options are "none", "mean" and "sum".
    """

    def __init__(self, reduction="mean"):
        super(L1Loss, self).__init__()
        self.reduction = reduction

    def forward(self, preds, targets, weights=None, sum_postive=None):
        assert preds.size() == targets.size() and targets.numel() > 0
        if sum_postive == 0:
            return preds.sum() * 0.0
        preds = preds.reshape(-1, preds.size(-1))
        targets = targets.reshape(-1, targets.size(-1))
        loss = torch.abs(preds - targets)
        if weights is not None:
            weights = weights.reshape(-1, weights.size(-1))
            loss = loss * weights
        if self.reduction == "sum":
            loss = loss.sum()
        elif self.reduction == "mean" and sum_postive > 0:
            loss = loss.sum() / sum_postive
        else:
            loss = loss.mean()
        return loss


@MODELS.register_module
class SmoothL1Loss(torch.nn.Module):
    """Smooth L1 loss.

    Args:
        beta (float, optional): The threshold in the piecewise function.
            Defaults to 1.0.
        reduction (str, optional): The method to reduce the loss.
            Options are "none", "mean" and "sum". Defaults to "mean".
    """

    def __init__(self, beta=1.0, reduction="mean"):
        super(SmoothL1Loss, self).__init__()
        assert beta > 0
        self.beta = beta
        self.reduction = reduction

    def forward(self, preds, targets, weights=None, sum_postive=None):
        assert preds.size() == targets.size() and targets.numel() > 0
        if sum_postive == 0:
            return preds.sum() * 0.0
        preds = preds.reshape(-1, preds.size(-1))
        targets = targets.reshape(-1, targets.size(-1))
        diff = torch.abs(preds - targets)
        loss = torch.where(
            diff < self.beta,
            0.5 * diff * diff / self.beta,
            diff - 0.5 * self.beta,
        )
        if weights is not None:
            weights = weights.reshape(-1, weights.size(-1))
            loss = loss * weights
        if self.reduction == "sum":
            loss = loss.sum()
        elif self.reduction == "mean" and sum_postive > 0:
            loss = loss.sum() / sum_postive
        else:
            loss = loss.mean()
        return loss
