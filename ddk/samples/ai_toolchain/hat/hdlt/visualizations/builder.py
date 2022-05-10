# Copyright (c) Horizon Robotics. All rights reserved.

from ..common.registry import build_from_cfg
from .registry import VISULIZATIONS


def build_visualization(cfg, default_args=None):
    if cfg is None:
        return None
    assert "type" in cfg, "type is need in visualization"
    return build_from_cfg(cfg, VISULIZATIONS)
