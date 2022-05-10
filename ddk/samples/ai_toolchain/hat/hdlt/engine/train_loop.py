# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import weakref

import numpy as np
import torch

from ..callbacks.callbacks import Callback

__all__ = ["BaseTrainer"]

logger = logging.getLogger(__name__)


class BaseTrainer(object):
    """
    Base Trainer which contains the basic train loop.

    Args:
        model (torch.nn.Module): The model of training.
        epoch (int): The epoch of training.
        metrics (List[hdlt.metrics]): The metrics for training data.
    """

    def __init__(self, model, epoch, metrics=None):
        self.start_epoch = 0
        self.epoch = epoch

        self.model = model
        self.callbacks = []
        self.metrics = metrics
        self.val_metrics = None
        self.epoch_end_flag = False
        self.losses = 0.0

    def set_callbacks(self, callbacks):
        for cb in callbacks:
            assert isinstance(cb, Callback)
            self.callbacks += [cb]

    @property
    def eval_model(self):
        if hasattr(self, "alter_model"):
            return self.alter_model
        else:
            return self.model

    def run_step(self):
        raise NotImplementedError

    def fit(self):
        self.model.train()
        self.iter = self.start_epoch
        self.on_train_begin()
        logger.info("Start training from {} epoch".format(self.start_epoch))
        for self.iter in range(self.start_epoch, self.epoch):
            if self.metrics is not None:
                for metric in self.metrics:
                    metric.reset()

            self.model.train()
            self.on_epoch_begin()
            self.run_step()
            self.on_epoch_end()

            if self.epoch_end_flag:
                break
        self.on_train_end()

    def on_batch_begin(self):
        for cb in self.callbacks:
            cb.on_batch_begin()

    def on_batch_end(self):
        for cb in self.callbacks:
            cb.on_batch_end()

    def on_epoch_begin(self):
        for cb in self.callbacks:
            cb.on_epoch_begin()

    def on_epoch_end(self):
        for cb in self.callbacks:
            cb.on_epoch_end()

    def on_train_begin(self):
        for cb in self.callbacks:
            cb.on_train_begin()

    def on_train_end(self):
        for cb in self.callbacks:
            cb.on_train_end()
