# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os
import shutil
import time

from ..common.meter import AverageMeter
from ..common.utils import _as_list
from .callbacks import Callback
from .registry import CALLBACKS

__all__ = ["StatsMonitor", "DetectionMonitor"]

logger = logging.getLogger(__name__)


def _losses_to_str(losses):
    ret = "losses["
    losses = _as_list(losses)
    for ls in losses:
        ret += "{}, ".format(ls.sum().item())
    return ret + "] "


@CALLBACKS.register_module
class StatsMonitor(Callback):
    """
    StatsMonitor Callback is used for some monitors of training
    including epoch time, batch time and so on.

    Args:
        log_freq (int): Freq of monitor whether to output to log.
        log_time (bool): Whether output time costs.
        trainer (hdlt.engine.BaseTrainer): The core trainer.
    """

    def __init__(self, log_freq=200, log_time=False, trainer=None):
        super(StatsMonitor, self).__init__(trainer=trainer)
        self.log_freq = log_freq
        self.log_time = log_time
        self.gpu_monitor = True
        if shutil.which("nvidia-smi") is None:
            logging.warnning(
                "Cannot monitor gpus because NVIDIA driver is not installed"
            )
            self.gpu_monitor = False
        self.epoch_time = AverageMeter("epoch_time", fmt=":6.3f")
        self.batch_time = AverageMeter("batch_time", fmt=":6.3f")
        self.data_load_time = AverageMeter("data_load_time", fmt=":6.3f")
        self.batch_idx = 0

        self.batch_time_begin = 0.0
        self.batch_time_end = 0.0

    def on_batch_begin(self):
        self.batch_time_begin = time.time()

    def on_batch_end(self):
        data_load_time = self.batch_time_begin - self.batch_time_end
        self.batch_time_end = time.time()
        batch_time = self.batch_time_end - self.batch_time_begin
        self.batch_time.update(batch_time)
        self.data_load_time.update(data_load_time)
        if self.batch_idx % self.log_freq == 0:
            log_info = "Epoch[%d] Batch[%d] " % (
                self.trainer.iter,
                self.batch_idx,
            )

            if self.log_time:
                log_info += (
                    "Avg run time: %.3f s, Avg dataloading time: %.3f s "
                    % (self.batch_time.avg, self.data_load_time.avg)
                )

            if hasattr(self.trainer.batch_processor, "losses"):
                log_info += _losses_to_str(self.trainer.batch_processor.losses)

            if self.trainer.metrics is not None:
                for metric in self.trainer.metrics:
                    name, value = metric.get()
                    for k, v in zip(_as_list(name), _as_list(value)):
                        log_info += "%s[%f] " % (k, v)

            logger.info(log_info)

            self.batch_time.reset()
            self.data_load_time.reset()
        self.batch_idx += 1

    def on_epoch_begin(self):
        self.batch_time.reset()
        self.epoch_time_begin = time.time()
        self.batch_idx = 0

    def on_epoch_end(self):
        epoch_time = time.time() - self.epoch_time_begin
        self.epoch_time.update(epoch_time)
        logger.info("Epoch cost time: %s s" % (self.epoch_time.avg))
        self.epoch_time.reset()


@CALLBACKS.register_module
class DetectionMonitor(Callback):
    """
    DetectionMonitor Callback is used for some monitors of training
    including epoch time, batch time and so on.

    Args:
        log_freq (int): Freq of monitor whether to output to log.
        trainer (hdlt.engine.BaseTrainer): The core trainer.
        losses (list): List of losses for monitoring.
    """

    def __init__(self, log_freq=200, trainer=None, losses=[]):
        super(DetectionMonitor, self).__init__(trainer=trainer)
        self.log_freq = log_freq
        self.gpu_monitor = True
        if shutil.which("nvidia-smi") is None:
            logging.warning(
                "Cannot monitor gpus because NVIDIA driver is not installed"
            )
            self.gpu_monitor = False
        self.epoch_time = AverageMeter("epoch_time", fmt=":6.3f")
        self.batch_time = AverageMeter("batch_time", fmt=":6.3f")
        self.losses = dict()
        for name in losses:
            self.losses[name] = AverageMeter(name, fmt=":6.3f")
        self.losses["loss"] = AverageMeter("loss", fmt=":6.3f")
        self.batch_idx = 0

    def on_batch_begin(self):
        self.batch_time_begin = time.time()

    def on_batch_end(self):
        losses = self.trainer.get_losses()
        losses_list = [v for v in losses.values()]
        loss_sum = sum(ls.sum() for ls in losses_list)
        batch_time = time.time() - self.batch_time_begin
        self.batch_time.update(batch_time)
        for name, value in losses.items():
            self.losses[name].update(value)
        self.losses["loss"].update(loss_sum)
        if self.batch_idx % self.log_freq == 0:
            logger.info(
                "Batch average cost time: %s s" % (self.batch_time.avg)
            )
            log_info = "Epoch[%d] Batch[%d] " % (
                self.trainer.iter,
                self.batch_idx,
            )

            for name, value in self.losses.items():
                log_info += "%s[%f] " % (name, value.avg)
            logger.info(log_info)
            self.batch_time.reset()
        self.batch_idx += 1

    def on_epoch_begin(self):
        self.batch_time.reset()
        self.epoch_time_begin = time.time()
        self.batch_idx = 0

    def on_epoch_end(self):
        epoch_time = time.time() - self.epoch_time_begin
        self.epoch_time.update(epoch_time)
        logger.info("Epoch cost time: %s s" % (self.epoch_time.avg))
        self.epoch_time.reset()
        self.batch_time.reset()
        for loss in self.losses.values():
            loss.reset()
