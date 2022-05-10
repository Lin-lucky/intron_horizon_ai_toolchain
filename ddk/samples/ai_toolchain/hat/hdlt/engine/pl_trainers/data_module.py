# Copyright (c) Horizon Robotics. All rights reserved.
# convert dataloader to pl datamodule

import torch

import pytorch_lightning as pl

__all__ = ["CustomDataModule"]


class CustomDataModule(pl.LightningDataModule):
    """
    Convert train dataloader and test dataloader to DataModule
    in pytorch lightning.

    Args:
        train_loader (DataLoader): Dataloader of training data.
        val_loader (DataLoader): DataLoader of validation data.
        test_loader (dataLoader): DataLoader of test data.
    """

    def __init__(self, train_loader, val_loader=None, test_loader=None):
        super().__init__()
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.test_loader = test_loader

    @property
    def dataset(self):
        return self.dataloader.dataset

    def train_dataloader(self):
        return self.train_loader

    def val_dataloader(self):
        return self.val_loader

    def test_dataloader(self):
        return self.test_loader

    def prepare_data(self):
        pass

    def setup(self, stage=None):
        pass
