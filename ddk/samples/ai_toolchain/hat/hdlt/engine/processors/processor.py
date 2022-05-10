# Copyright (c) Horizon Robotics. All rights reserved.

from collections.abc import Sequence

import torch

from hdlt.common.utils import _as_list, _to_cuda

from .registry import PROCESSORS

__all__ = ["BasicProcessor"]


@PROCESSORS.register_module
class BasicProcessor(object):
    """
    Training processor of each batch.

    Args:
        model (torch.nn.Module): The model for training.
        optimizer (torch.optim): The optimizer for training.
        gpus (int): Current device for training.
        metrics (List[hdlt.metrics]): The metrics for training data.
        apex (apex): Whether to use the apex func.
    """

    def __init__(self, model, optimizer, gpus=None, metrics=None, apex=None):
        self.model = model
        self.optimizer = optimizer
        self.gpus = gpus
        self.metrics = metrics
        self.losses = 0.0
        self.apex = apex

    def __call__(self, data, target):
        if self.gpus is not None:
            data = _to_cuda(data, self.gpus, non_blocking=True)
            target = _to_cuda(target, self.gpus, non_blocking=True)
        self.losses, preds = self.model(data, target)
        if isinstance(self.losses, dict):
            losses = [v for v in self.losses.values()]
        else:
            losses = self.losses
        loss = sum(ls.sum() for ls in losses)

        self.optimizer.zero_grad()
        if self.apex is not None:
            with self.apex.amp.scale_loss(loss, self.optimizer) as scale_loss:
                scale_loss.backward()
        else:
            loss.backward()
        self.optimizer.step()

        if self.metrics is not None:
            for metric in self.metrics:
                metric.update(target, preds)

    def get_losses(self):
        return self.losses
