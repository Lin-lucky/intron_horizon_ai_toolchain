# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os

import numpy as np
import torch
import torch.distributed as dist
import torch.multiprocessing as mp
from torch.nn import SyncBatchNorm
from torch.utils.data import DistributedSampler

from hdlt.common.socket import find_free_port

from ..data.builder import build_dataloader
from ..metrics.builder import build_metric
from ..optimizers.builder import build_optimizer
from .processors.builder import build_processor
from .registry import TRAINERS
from .train_loop import BaseTrainer

__all__ = ["DistributedDataParallelTrainer"]

logger = logging.getLogger(__name__)


@TRAINERS.register_module
class DistributedDataParallelTrainer(BaseTrainer):
    """
    DistributedDataParallelTrainer is the core component for training
    with DistributedDataParallel method.
    DistributedDataParallelTrainer must be launched by launch function below.

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
        sync_bn=True,
    ):

        self.dataloader = build_dataloader(train_dataloader)
        self.optimizer = build_optimizer(optimizer, dict(model=model))
        self.metrics = build_metric(metrics)
        self.steps = len(self.dataloader)

        super(DistributedDataParallelTrainer, self).__init__(
            model=model, epoch=epoch, metrics=self.metrics
        )

        self.model.cuda(gpus)
        if sync_bn:
            self.model = SyncBatchNorm.convert_sync_batchnorm(self.model)
        self.model = torch.nn.parallel.DistributedDataParallel(
            self.model, find_unused_parameters=True, device_ids=[gpus]
        )

        logger.info("Training on gpu %d" % (gpus))
        self.gpus = gpus

        self.batch_processor = build_processor(
            dict(
                model=self.model,
                optimizer=self.optimizer,
                gpus=self.gpus,
                metrics=self.metrics,
                **processor
            )
        )

    def run_step(self):
        assert self.model.training, "Model was changed to eval mode!"

        if isinstance(self.dataloader.sampler, DistributedSampler):
            self.dataloader.sampler.set_epoch(self.iter)

        for i, data in enumerate(self.dataloader):
            self.on_batch_begin()
            self.batch_processor(*data)
            self.losses = self.batch_processor.losses
            self.on_batch_end()
            return 
    def get_losses(self):
        return self.batch_processor.get_losses()


def launch(main_func, num_gpus, dist_url="auto", args=()):
    if dist_url == "auto":
        port = find_free_port()
        dist_url = "tcp://localhost:{}".format(port)

    if hasattr(args[0], "launcher") and args[0].launcher == "mpi":
        _main_mpi(main_func, dist_url, num_gpus, args)
    else:
        mp.spawn(
            _main_func,
            nprocs=num_gpus,
            args=(main_func, dist_url, num_gpus, args),
        )


def _main_func(local_rank, main_func, dist_url, world_size, args):
    try:
        dist.init_process_group(
            backend="NCCL",
            init_method=dist_url,
            world_size=world_size,
            rank=local_rank,
        )
    except Exception as e:
        logger.error("Process group URL: {}".format(dist_url))
        raise e

    torch.cuda.set_device(local_rank)
    main_func(local_rank, *args)


def _main_mpi(main_func, dist_url, num_gpus, args):
    import mpi4py.MPI as MPI

    comm = MPI.COMM_WORLD
    local_rank = comm.Get_rank()
    world_size = comm.Get_size()
    try:
        dist.init_process_group(
            backend="NCCL",
            init_method=dist_url,
            world_size=world_size,
            rank=local_rank,
        )
    except Exception as e:
        logger.error("Process group URL: {}".format(dist_url))
        raise e
    torch.cuda.set_device(local_rank % num_gpus)
    main_func(local_rank % num_gpus, *args)
