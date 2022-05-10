# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os
from numbers import Real
from typing import Sequence, Type

import torch
from torch import Tensor
from torch.utils.tensorboard import SummaryWriter
from torch.utils.tensorboard.summary import hparams

from ..common.utils import _as_list, _to_cuda
from .callbacks import Callback
from .registry import CALLBACKS

__all__ = ["TensorBoard"]

logger = logging.getLogger(__name__)


@CALLBACKS.register_module
class TensorBoard(Callback):
    """
    TensorBoard Callback is used for outputing some monitor objects
    to tensorboard.
    Some other functions are being added.

    Args:
        save_dir (str): The dir for saving tensorboard.
        name (str): The filename for saving tensorboard.
        update_freq (str): The freq of outputing to tensorboard.
        trainer (hdlt.engine.BaseTrainer): The core trainer.
    """

    def __init__(
        self,
        save_dir,
        name="default",
        update_freq="epoch",
        trainer=None,
        log_graph=True,
        **kwargs
    ):
        super(TensorBoard, self).__init__(trainer=trainer)
        self.save_dir = save_dir
        self.name = name
        if self.name is None or len(self.name) == 0:
            self.log_dir = self.save_dir
        else:
            self.log_dir = os.path.join(self.save_dir, self.name)

        if int(torch.cuda.current_device()) == 0:
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
            self.writer = SummaryWriter(self.log_dir, **kwargs)
        else:
            self.writer = None

        self.update_freq = update_freq
        assert self.update_freq in ["batch", "epoch"]

        self.log_graph = log_graph

        self.batch_seen = 0
        self.epoch_seen = 0

    def on_train_begin(self):
        if self.writer is None:
            return
        if self.log_graph:
            for i, data in enumerate(self.trainer.dataloader):
                data = _to_cuda(data)
                self.writer.add_graph(self.trainer.model.module, data)
                break

    def on_batch_end(self):
        self.batch_seen += 1
        if self.writer is None:
            return

        if self.update_freq == "batch":
            losses = self.trainer.batch_processor.losses
            losses = losses if isinstance(losses, dict) else {"losses": losses}
            self._log_metrics("train/batch", losses, self.batch_seen)
            for metric in self.trainer.metrics:
                name, value = metric.get()
                for k, v in zip(_as_list(name), _as_list(value)):
                    self._log_metrics("train/batch", {k: v}, self.batch_seen)

    def on_epoch_end(self):
        self.epoch_seen += 1
        if self.writer is None:
            return

        if self.trainer.val_metric is None:
            logger.warning(
                "No val_metric in trainer."
                "Sometimes due to Validation after Tensorboard."
            )
            return

        metric = self.trainer.val_metric
        self._log_metrics("val/batch", metric, self.batch_seen)
        self._log_metrics("val/epoch", metric, self.epoch_seen)

    def on_train_end(self):
        if self.writer is not None:
            self.writer.close()

    def _log_metrics(self, prefix, metrics, step):
        if isinstance(metrics, dict):
            for k, v in metrics.items():
                self._log_metrics(prefix + "/" + str(k), v, step)
            if "losses" in metrics:
                losses = _as_list(metrics["losses"])
                loss = sum(ls.sum() for ls in losses)
                self.writer.add_scalar(
                    prefix + "/total_loss", loss.item(), step
                )
        elif isinstance(metrics, Sequence):
            for i, vv in enumerate(metrics):
                self._log_metrics(prefix + "/" + str(i), vv, step)
        elif isinstance(metrics, Real):
            self.writer.add_scalar(prefix, metrics, step)
        elif isinstance(metrics, Tensor):
            if metrics.ndim == 1:
                if metrics.numel() == 1:
                    self.writer.add_scalar(prefix, metrics.item(), step)
                else:
                    self.writer.add_histogram(prefix, metrics, step)
            elif metrics.ndim == 3:
                self.writer.add_image(prefix, metrics, step)
            elif metrics.ndim == 4:
                self.writer.add_images(prefix, metrics, step)
        else:
            raise TypeError(
                "TensorBoard callback only support metric "
                + "type in Dict, Sequence, Tensor and Real."
            )
