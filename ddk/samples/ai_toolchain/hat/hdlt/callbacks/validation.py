# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import time

import numpy as np
import torch

from ..common.utils import _as_list
from ..data.builder import build_dataloader
from ..metrics.builder import build_metric
from .callbacks import Callback
from .registry import CALLBACKS

__all__ = ["Validation"]

logger = logging.getLogger(__name__)


@CALLBACKS.register_module
class Validation(Callback):
    """
    Validation Callback is used for getting the metrics on validation datasets.
    Validation also provides the monitor for Checkpoint and Tensorboard.

    Args:
        dataloaders (List[torch.utils.data.DataLodaer]): All dataloaders
            for validation.
        model (torch.nn.Module): The model for validation.
        metrics (list[hdlt.metrics]): The metrics on all dataloaders.
        trainer (hdlt.engine.BaseTrainer): The core trainer.
    """

    def __init__(
        self,
        dataloaders,
        freq=1,
        model=None,
        metrics=None,
        trainer=None,
    ):
        super(Validation, self).__init__(trainer=trainer)
        if int(torch.cuda.current_device()) == 0:
            self.skip = False
        else:
            self.skip = True
        self.metrics = None
        if metrics is not None:
            self.metrics = build_metric(metrics)

        dataloaders = _as_list(dataloaders)
        self.dataloaders = [build_dataloader(dl) for dl in dataloaders]
        self.datanames = ["dataset_%i" % (i) for i in range(len(dataloaders))]

        self.model = model
        self.freq = freq

    def on_epoch_end(self):
        if self.skip:
            return
        if (self.trainer.iter + 1) % self.freq != 0:
            return

        if self.model is None:
            model = self.trainer.eval_model
        else:
            model = model

        model.eval()
        metric_dict = dict()

        with torch.no_grad():
            for dataname, dataloader in zip(self.datanames, self.dataloaders):
                if self.metrics is not None:
                    for metric in self.metrics:
                        metric.reset()
                for inputs, targets in dataloader:
                    if self.trainer.gpus is not None:
                        inputs = inputs.to(self.trainer.gpus)
                    preds, _ = model(inputs)
                    for metric in self.metrics:
                        metric.update(targets, preds)

                log_info = "Epoch[%d] Eval:" % self.trainer.iter
                md = dict()
                for metric in self.metrics:
                    name, value = metric.get()
                    for k, v in zip(_as_list(name), _as_list(value)):
                        log_info += "{}={}\n".format(k, v)
                        md[k] = v

                metric_dict[dataname] = md
                if len(self.datanames) > 1:
                    log_info += "on dataset[%s]" % (dataname)
                logger.info(log_info)

            # for checkpoint monitor
            self.trainer.val_metric = metric_dict
