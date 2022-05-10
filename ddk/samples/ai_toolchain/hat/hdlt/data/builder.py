# Copyright (c) Horizon Robotics. All rights reserved.

import pickle

import torch
import torch.utils.data.distributed as dist
import torchvision

from ..common.registry import build_from_cfg
from .registry import COLLATES, DATALOADERS, DATASETS, SAMPLERS, TRANSFORMS

__all__ = ["build_dataset", "build_dataloader"]


def build_dataset(cfg, default_args=None):
    if cfg is None:
        return None

    if "transforms" in cfg:
        if cfg["transforms"] is None:
            cfg["transforms"] = None
        assert isinstance(cfg["transforms"], (list, tuple))

        tmp = list()
        for ds in cfg["transforms"]:
            transform = build_from_cfg(ds, TRANSFORMS)
            tmp.append(transform)
        cfg["transforms"] = torchvision.transforms.Compose(tmp)

    dataset = build_from_cfg(cfg, DATASETS)

    # sometimes pickle dataset for supportting
    # DataParallel and DistributedDataParallel
    # at the same time.
    dataset = pickle.dumps(dataset)
    dataset = pickle.loads(dataset)

    return dataset


def build_dataloader(cfg, default_args=None):
    if cfg is None:
        return None

    assert "dataset" in cfg, "No dataset in dataloader."
    cfg["dataset"] = build_dataset(cfg["dataset"])

    if ("sampler" in cfg) and (cfg["sampler"] is not None):
        sampler_type = cfg["sampler"]["type"]
        if sampler_type == torch.utils.data.DistributedSampler:
            cfg["sampler"] = dist.DistributedSampler(cfg["dataset"])
            cfg["shuffle"] = cfg["sampler"] is None
        elif issubclass(sampler_type, torch.utils.data.Sampler):
            cfg["sampler"].pop("type")
            cfg["sampler"] = sampler_type(cfg["dataset"], **cfg["sampler"])
        else:
            cfg["sampler"] = build_from_cfg(cfg["sampler"], SAMPLERS)

    if ("collate_fn" in cfg) and (cfg["collate_fn"] is not None):
        cfg["collate_fn"] = build_from_cfg(cfg["collate_fn"], COLLATES)

    if ("transforms" in cfg) and (cfg["transforms"] is not None):
        cfg["transforms"] = [
            build_from_cfg(tr, TRANSFORMS) for tr in cfg["transforms"]
        ]

    return build_from_cfg(cfg, DATALOADERS)
