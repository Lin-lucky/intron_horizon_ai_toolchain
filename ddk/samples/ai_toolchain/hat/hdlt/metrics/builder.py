# Copyright (c) Horizon Robotics. All rights reserved.

from ..common.registry import build_from_cfg
from ..common.utils import _as_list
from .registry import METRICS


def build_metric(cfg, default_args=None):
    if cfg is None:
        return None

    metric_cfg = _as_list(cfg)
    metrics = list()
    for mc in metric_cfg:
        metric = build_from_cfg(mc, METRICS)
        metrics.append(metric)
    return metrics
