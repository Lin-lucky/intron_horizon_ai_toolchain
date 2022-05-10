# Copyright (c) Horizon Robotics. All rights reserved.
# convert model to pl model module

from collections import OrderedDict

import torch

import pytorch_lightning as pl
from hdlt.common.utils import _as_list

__all__ = ["CustomModelModule"]


class CustomModelModule(pl.LightningModule):
    """
    Convert common training tools to ModelModule in pytorch lightning.

    Args:
        model (torch.nn.Module): The model for training.
        metrics (List[hdlt.metrics]): The metrics for training.
        optimizer (torch.optim): The optimizer for training.
    """

    def __init__(self, model, metrics, optimizer):
        super(CustomModelModule, self).__init__()
        self.save_hyperparameters()

        self.model = model
        self.metrics = metrics
        self.optimizer = optimizer

    def forward(self, image, target=None):
        return self.model.forward(image, target)

    def training_step(self, batch, batch_idx):
        self.model.train()
        loss = self.forward(*batch)
        loss = _as_list(loss)
        losses = sum(ls.sum() for ls in loss)
        tqdm_dict = {'train_loss': losses}
        output = OrderedDict({
            'loss': losses,
            'progress_bar': tqdm_dict,
            'log': tqdm_dict
        })
        return output

    def validation_step(self, batch, batch_idx):
        self.model.train()
        loss = self.forward(*batch)
        loss = _as_list(loss)
        losses = sum(ls.sum() for ls in loss)
        tqdm_dict = {'val_loss': losses}
        output = OrderedDict({
            'loss': losses,
            'progress_bar': tqdm_dict,
            'log': tqdm_dict
        })
        return output

    def test_step(self, batch, batch_idx):
        self.model.eval()
        preds = self.forward(batch)
        return preds

    def configure_optimizers(self):
        return self.optimizer
