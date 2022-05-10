# Copyright (c) Horizon Robotics. All rights reserved.

from ..common.registry import build_from_cfg
from .registry import CALLBACKS


def build_callbacks(cfg, default_args=None):
    if cfg is None:
        return None
    assert isinstance(cfg, (list, tuple))
    assert "trainer" in default_args

    callbacks = list()
    for ds in cfg:
        ds.update(dict(trainer=default_args["trainer"]))
        callbacks.append(build_from_cfg(ds, CALLBACKS))
    return callbacks
