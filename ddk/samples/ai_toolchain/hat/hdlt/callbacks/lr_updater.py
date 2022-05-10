# Copyright (c) Horizon Robotics. All rights reserved.

import logging
from math import cos, pi

from .callbacks import Callback
from .registry import CALLBACKS

__all__ = ["WarmupStepLrUpdater", "WarmupCosLrUpdater"]

logger = logging.getLogger(__name__)


@CALLBACKS.register_module
class WarmupStepLrUpdater(Callback):
    """
    Lr Updater Callback for adjusting lr with warmup and step decay.

    Args:
        base_lr (float): Base lr of optimizers.
        warmup_epoch (int): Num epoches for warming up.
        lr_decay_step (List(int)): The epoch list for lr decay.
        lr_decay_factor (float): Factor for lr decay.
        trainer (hdlt.engine.BaseTrainer): The core trainer.
    """

    def __init__(
        self,
        base_lr=0.1,
        warmup_epoch=0,
        lr_decay_step=[],
        lr_decay_factor=0.1,
        trainer=None,
    ):
        super(WarmupStepLrUpdater, self).__init__(trainer=trainer)
        self.base_lr = base_lr
        self.lr_decay_step = lr_decay_step
        self.lr_decay_factor = lr_decay_factor
        self.warmup_epoch = warmup_epoch
        self.seen_batch = 0

    def on_batch_begin(self):
        optimizer = self.trainer.optimizer
        warmup_steps = self.warmup_epoch * self.trainer.steps

        # for load checkpoint
        if self.seen_batch == 0:
            self.seen_batch += self.trainer.start_epoch * self.trainer.steps
        self.seen_batch += 1
        if self.seen_batch < warmup_steps:
            new_lr = self.base_lr / warmup_steps * self.seen_batch
            for param_group in optimizer.param_groups:
                param_group["lr"] = new_lr

    def on_epoch_begin(self):
        optimizer = self.trainer.optimizer
        last_lr = optimizer.param_groups[-1]["lr"]

        if self.trainer.iter in self.lr_decay_step:
            for param_group in optimizer.param_groups:
                param_group["lr"] *= self.lr_decay_factor
                last_lr = param_group["lr"]
        logger.info("lr changes to {}".format(last_lr))


@CALLBACKS.register_module
class WarmupCosLrUpdater(Callback):
    """
    Lr Updater Callback for adjusting lr with warmup and cos decay.

    Args:
        base_lr (float): Base lr of optimizers.
        warmup_epoch (int): Num epoches for warming up.
        epoch (int): The epoch for cos lr decay.
        trainer (hdlt.engine.BaseTrainer): The core trainer.
    """

    def __init__(self, base_lr=0.1, warmup_epoch=0, epoch=0, trainer=None):
        super(WarmupCosLrUpdater, self).__init__(trainer=trainer)
        self.base_lr = base_lr
        self.warmup_epoch = warmup_epoch
        self.epoch = epoch
        self.seen_batch = 0

    def on_batch_begin(self):
        optimizer = self.trainer.optimizer
        warmup_steps = self.warmup_epoch * self.trainer.steps
        cos_steps = (self.epoch - self.warmup_epoch) * self.trainer.steps

        # for load checkpoint
        if self.seen_batch == 0:
            self.seen_batch += self.trainer.start_epoch * self.trainer.steps
        self.seen_batch += 1

        if self.seen_batch <= warmup_steps:
            new_lr = self.base_lr / warmup_steps * self.seen_batch
        else:
            update_steps = self.seen_batch - warmup_steps
            factor = 1 + cos(pi * update_steps / cos_steps)
            new_lr = self.base_lr * factor / 2
        for param_group in optimizer.param_groups:
            param_group["lr"] = new_lr

    def on_epoch_end(self):
        optimizer = self.trainer.optimizer
        last_lr = optimizer.param_groups[-1]["lr"]
        logger.info("lr changes to %f" % (last_lr))
