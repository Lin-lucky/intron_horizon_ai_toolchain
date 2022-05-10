# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os

import numpy as np
import torch
from torch.utils.data import DistributedSampler

from ..data.builder import build_dataloader
from ..metrics.builder import build_metric
from ..optimizers.builder import build_optimizer
from .processors.builder import build_processor
from .registry import TRAINERS
from .train_loop import BaseTrainer

__all__ = ["HorovodTrainer"]

logger = logging.getLogger(__name__)


@TRAINERS.register_module
class HorovodTrainer(BaseTrainer):
    """
    HorovodTrainer is the core component for training with horovod method.
    HorovodTrainer must be launched by launch function below.
    HorovodTrainer must run in an environment with horovod.

    Args:
        gpus (int): Current device for training.
        model (torch.nn.Module): The model for training.
        train_dataloader (DataLoader): The dataloader provides train data.
        optimizer (torch.optim): The optimizer for training.
        epoch (int): Training epoch.
        processor (hdlt.engine.processors): Training processor for each batch.
        metrics (List[hdlt.metrics]): The metrics for training data.
        compression (numpy.dtype): Compression for reducing data.
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
        compression=None,
    ):
        try:
            import horovod as hvd
        except ImportError:
            logging.error("HorovodTrainer cannot import horovod!")

        self.dataloader = build_dataloader(train_dataloader)
        self.optimizer = build_optimizer(optimizer, dict(model=model))
        self.metrics = build_metric(metrics)
        self.steps = len(self.dataloader)

        super(HorovodTrainer, self).__init__(
            model=model, epoch=epoch, metrics=self.metrics
        )

        local_rank = hvd.local_rank()
        self.model.cuda(local_rank)
        hvd.broadcast_parameters(self.model.state_dict(), root_rank=0)
        hvd.broadcast_optimizer_state(self.optimizer, root_rank=0)
        self.optimizer = hvd.DistributedOptimizer(
            self.optimizer,
            name_parameters=self.model.named_parameters(),
            compression=compression,
        )

        self.gpus = local_rank
        logger.info("Training on gpu %d" % (self.gpus))

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


def launch(main_func, num_gpus, dist_url=None, args=()):
    """
       Use horovodrun to launch horovod job.
       For example:
           $ horovodrun -np 4 -H localhost:4 python3 tools/xxxx
    """

    try:
        import horovod as hvd
    except ImportError:
        logging.error("HorovodTrainer cannot import horovod!")

    hvd.init()
    torch.cuda.set_device(hvd.local_rank())
    main_func(hvd.local_rank, *args)
