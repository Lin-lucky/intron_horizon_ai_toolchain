# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os

import numpy as np
import torch
from torch.utils.data import DistributedSampler

from hdlt.common.registry import build_from_cfg
from hdlt.common.utils import _as_list

from ..data.builder import build_dataloader
from ..metrics.builder import build_metric
from ..optimizers.builder import build_optimizer
from .processors.builder import build_processor
from .registry import TRAINERS
from .train_loop import BaseTrainer

__all__ = ["DataParallelTrainer"]

logger = logging.getLogger(__name__)


@TRAINERS.register_module
class DataParallelTrainer(BaseTrainer):
    """
    DataParallelTrainer is the core component for training
    with DataParallel method.
    DataParallelTrainer must be launched by launch function below.

    Args:
        gpus (int): Current device for training.
        model (torch.nn.Module): The model for training.
        train_dataloader (DataLoader): The dataloader provides train data.
        optimizer (torch.optim): The optimizer for training.
        epoch (int): Training epoch.
        processor (hdlt.engine.processors): Training processor for each batch.
        metrics (List[hdlt.metrics]): The metrics for training data.
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
    ):
        self.dataloader = build_dataloader(train_dataloader)
        self.optimizer = build_optimizer(optimizer, dict(model=model))
        self.metrics = build_metric(metrics)
        self.steps = len(self.dataloader)

        super(DataParallelTrainer, self).__init__(
            model=model, epoch=epoch, metrics=self.metrics
        )

        if gpus == 1:
            self.model.cuda()
        else:
            self.model = torch.nn.DataParallel(model).cuda()

        if hasattr(self.dataloader, "sampler"):
            assert not isinstance(
                self.dataloader.sampler, DistributedSampler
            ), "DataParallel cannot have DistributedSampler in dataloader."

        logger.info("Training on %s gpus" % (str(gpus)))

        self.batch_processor = build_processor(
            dict(
                model=self.model,
                optimizer=self.optimizer,
                gpus=None,
                metrics=self.metrics,
                **processor
            )
        )

    def run_step(self):
        assert self.model.training, "Model was changed to eval mode!"
        for i, data in enumerate(self.dataloader):
            self.on_batch_begin()
            self.batch_processor(*data)
            self.on_batch_end()


def launch(main_func, num_gpus, dist_url=None, args=()):
    main_func(num_gpus, *args)
