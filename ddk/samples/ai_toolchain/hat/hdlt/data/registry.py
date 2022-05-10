# Copyright (c) Horizon Robotics. All rights reserved.

from ..common.registry import Registry

SAMPLERS = Registry("sampler")
TRANSFORMS = Registry("transforms")
COLLATES = Registry("collates")

DATASETS = Registry("dataset")
DATALOADERS = Registry("dataloader")
