# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os

import numpy as np
import torch
from torch.utils.data import DistributedSampler

from ..data.builder import build_dataloader
from ..metrics.builder import build_metric
from ..optimizers.builder import build_optimizer
from . import dpp_trainer
from .processors.builder import build_processor
from .registry import TRAINERS
from .train_loop import BaseTrainer

__all__ = ["ApexDDPTrainer"]

logger = logging.getLogger(__name__)


@TRAINERS.register_module
class ApexDDPTrainer(BaseTrainer):
    """
    ApexDDPTrainer is the core component for training with Apex
    and DistributedDataParallel method.
    ApexDDPTrainer must be launched by launched function below.
    ApexDDPTrainer must run in an environment with apex.

    Args:
        gpus (int): Current device for training.
        model (torch.nn.Module): The model for training.
        train_dataloader (DataLoader): The dataloader provides train data.
        optimizer (torch.optim): The optimizer for training.
        epoch (int): Training epoch.
        processor (hdlt.engine.processors): Training processor for each batch.
        metrics (List[hdlt.metrics]): The metrics for training data.
        kwargs (dict): Kwargs for apex.
    """

    def __init__(
        self,
        gpus,
        model,
        train_dataloader,
        optimizer,
        epoch,
        processor=dict(type="BasicProcessor"),
        metrics=None,
        **kwargs
    ):
        try:
            import apex
        except ImportError:
            logging.error("Cannot import apex!")

        self.dataloader = build_dataloader(train_dataloader)
        self.optimizer = build_optimizer(optimizer, dict(model=model))
        self.metrics = build_metric(metrics)
        self.steps = len(self.dataloader)

        super(ApexDDPTrainer, self).__init__(
            model=model, epoch=epoch, metrics=self.metrics
        )

        self.model.cuda(gpus)
        self.model, self.optimizer = apex.amp.initialize(
            self.model, self.optimizer, **kwargs
        )
        self.model = apex.parallel.DistributedDataParallel(
            self.model
        )
        processor["apex"] = apex

        logger.info("Training on gpu %d" % (gpus))
        self.gpus = gpus

        self.batch_processor = build_processor(dict(
            model=self.model, optimizer=self.optimizer,
            gpus=self.gpus, metrics=self.metrics,
            **processor
        ))

    def run_step(self,):
        assert self.model.training, "Model was changed to eval mode!"

        if isinstance(self.dataloader.sampler, DistributedSampler):
            self.dataloader.sampler.set_epoch(self.iter)

        for i, data in enumerate(self.dataloader):
            self.on_batch_begin()
            self.batch_processor(*data)
            self.on_batch_end()


launch = dpp_trainer.launch
