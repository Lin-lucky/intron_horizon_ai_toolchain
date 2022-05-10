# Copyright (c) Horizon Robotics. All rights reserved.

from collections.abc import Sequence
from typing import List

import numpy
import torch

from ..data.transforms.segmentation import _to_one_hot
from .metric import EvalMetric, check_label_shapes
from .registry import METRICS

__all__ = ["MeanIoU"]


@METRICS.register_module
class MeanIoU(EvalMetric):
    """
    Compute mean IoU between predictions and targets.

    Args:
        logit_input (bool): Whether input pred is logit or class index.
        num_classes (int, optional): Pass class number by hand.
            Defaults to None.
        name (str, optional): Name of this metric instance for display.
            Defaults to "iou".
        output_names (List[str], optional): Name of predictions that should be
            used when updating with update_dict.
            By default include all predictions. Defaults to None.
        label_names (List[str], optional): Name of labels that should be used
            when updating with update_dict. By default include all labels.
            Defaults to None.
    """

    def __init__(
        self,
        logit_input: bool,
        num_classes: int,
        name: str = "iou",
        output_names: List[str] = None,
        label_names: List[str] = None,
        **kwargs
    ):
        self.logit_input = logit_input
        self.num_classes = num_classes
        super(MeanIoU, self).__init__(
            name, output_names=output_names, label_names=label_names, **kwargs
        )

    def reset(self):
        """Resets the internal evaluation result to initial state."""
        self.reset_local()
        self.global_num_inst = 0
        self.global_sum_metric = torch.zeros(
            self.num_classes, self.num_classes, dtype=torch.double
        )

    def reset_local(self):
        """Resets the local portion of the internal evaluation results
        to initial state."""
        self.num_inst = 0
        self.sum_metric = torch.zeros(
            self.num_classes, self.num_classes, dtype=torch.double
        )

    def update(self, label, preds):
        if isinstance(label, Sequence):
            label = label[0]
        if isinstance(preds, Sequence):
            preds = preds[0]

        if label.ndim < 4:
            label = label.unsqueeze(1)

        if self.logit_input:
            num_classes = preds.size(1)
            idx_preds = torch.argmax(preds, dim=1, keepdim=True)
        else:
            num_classes = int(
                max(
                    max(preds.max().item(), label.max().item()) + 1,
                    label.size(1),
                )
            )
            idx_preds = preds.to(dtype=torch.int64)

        if self.num_classes is not None:
            num_classes = self.num_classes
        one_hot_preds = _to_one_hot(idx_preds, num_classes)
        one_hot_label = _to_one_hot(label, num_classes)

        one_hot_preds = one_hot_preds.permute(1, 0, 2, 3).flatten(1, 3)
        one_hot_label = one_hot_label.permute(1, 0, 2, 3).flatten(1, 3)

        current_metrix = torch.sum(
            one_hot_label.unsqueeze(1) * one_hot_preds.unsqueeze(0), dim=2
        )

        self.sum_metric += current_metrix.detach().cpu()
        self.global_sum_metric += current_metrix.detach().cpu()

        self.num_inst += 1
        self.global_num_inst += 1

    def _confusion_to_mean_iou(self, confusion_matrix: torch.Tensor):
        sum_label = confusion_matrix.sum(dim=0)
        sum_preds = confusion_matrix.sum(dim=1)
        true_positives = confusion_matrix.diag()
        denominator = sum_label + sum_preds - true_positives
        num_valid_entries = torch.sum(denominator != 0)
        iou = torch.div(true_positives, denominator)
        mean_iou = (torch.nansum(iou) / num_valid_entries).item()
        return mean_iou

    def get(self):
        if self.num_inst == 0:
            return (self.name, float("nan"))
        else:
            return (self.name, self._confusion_to_mean_iou(self.sum_metric))

    def get_global(self):
        if self._has_global_stats:
            if self.global_num_inst == 0:
                return (self.name, float("nan"))
            else:
                return (
                    self.name,
                    self._confusion_to_mean_iou(self.global_sum_metric),
                )
        else:
            return self.get()
