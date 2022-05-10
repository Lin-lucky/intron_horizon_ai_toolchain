# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os

from hdlt.callbacks.callbacks import Callback
from hdlt.data.builder import build_dataloader
from hdlt.metrics.builder import build_metric
from hdlt.optimizers.builder import build_optimizer

from ..registry import TRAINERS

try:
    from .model_module import CustomModelModule
    from .data_module import CustomDataModule
except Exception:
    pass

__all__ = ["LightningTrainer"]

logger = logging.getLogger(__name__)


@TRAINERS.register_module
class LightningTrainer(object):
    """
    LightningTrainer is the core component for training with pytorch lightning.

    Args:
        gpus (int): Current device for training.
        model (torch.nn.Module): The model for training.
        train_dataloader (DataLoader): The dataloader provides train data.
        optimizer (torch.optim): The optimizer for training.
        metrics (List[hdlt.metrics]): The metrics for training data.
        val_dataloader (DataLoader): The dataloader provides val data.
        test_dataloader (DataLoader): The dataloader provides test data.
    """

    def __init__(
        self,
        gpus,
        model,
        train_dataloader,
        optimizer,
        lightning,
        metrics=None,
        val_dataloader=None,
        test_dataloader=None,
    ):
        try:
            import pytorch_lightning as pl
        except ImportError:
            logger.error("Cannot import pytorch_lightning!")

        self.callbacks = []
        self.train_loader = build_dataloader(train_dataloader)
        self.val_loader = build_dataloader(val_dataloader)
        self.test_loader = build_dataloader(test_dataloader)  \
            if test_dataloader is not None else self.val_loader

        self.optimizer = build_optimizer(optimizer, dict(model=model))
        self.metrics = build_metric(metrics)

        # convert basic module to pl module
        self.data_module = CustomDataModule(
            self.train_loader,
            self.val_loader,
            self.test_loader,
        )
        self.model_module = CustomModelModule(
            model, self.metrics, self.optimizer
        )

        self.trainer = pl.Trainer(**lightning)

    def set_callbacks(self, callbacks):
        for cb in callbacks:
            assert isinstance(cb, Callback)
            self.callbacks += [cb]

    def fit(self):
        self.trainer.fit(self.model_module, self.data_module)
        self.trainer.test(self.model_module, self.data_module)


def launch(main_func, num_gpus, dist_url=None, args=()):
    main_func(num_gpus, *args)
