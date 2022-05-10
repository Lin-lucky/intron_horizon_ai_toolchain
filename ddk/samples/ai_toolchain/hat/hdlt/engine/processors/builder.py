# Copyright (c) Horizon Robotics. All rights reserved.

from hdlt.common.registry import build_from_cfg

from .registry import PROCESSORS


def build_processor(cfg, default_args=None):
    if cfg is None:
        return None
    for df_arg in ["model", "optimizer", "gpus", "metrics"]:
        assert df_arg in cfg, "%s is not found in PROCESSORS" % (df_arg)
    return build_from_cfg(cfg, PROCESSORS)
