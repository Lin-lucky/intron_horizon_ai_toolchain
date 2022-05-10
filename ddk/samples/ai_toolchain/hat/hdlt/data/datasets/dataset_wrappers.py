# Copyright (c) Horizon Robotics. All rights reserved.

import torch
import torch.utils.data as data
import torchvision

from ..builder import build_dataset
from ..registry import DATASETS

__all__ = ["RepeatDataset"]


@DATASETS.register_module
class RepeatDataset(data.Dataset):
    """
    A wrapper of repeated dataset.
    Using RepeatDataset can reduce the data loading time between epochs.

    Args:
        dataset (torch.utils.data.Dataset): The datasets for repeating.
        times (int): Repeat times.
    """

    def __init__(self, dataset, times):
        self.dataset = build_dataset(dataset)
        self.times = times
        if hasattr(self.dataset, 'flag'):
            self.flag = np.tile(self.dataset.flag, times)
        self._ori_len = len(self.dataset)

    def __getitem__(self, idx):
        return self.dataset[idx % self.ori_len]

    def __len__(self):
        return self.times * self._ori_len
