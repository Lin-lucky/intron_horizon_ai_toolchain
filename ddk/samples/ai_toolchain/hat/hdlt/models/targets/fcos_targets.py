import numpy as np
import torch
import torch.nn as nn

from hdlt.common.registry import build_from_cfg

from ..registry import MODELS


@MODELS.register_module
class FCOSTarget(nn.Module):
    """
    Fcos target.

    Args:
        match_cfg (dict):
            matcher setting.
    """

    def __init__(self, match_cfg):
        super(FCOSTarget, self).__init__()
        self.match_cfg = build_from_cfg(match_cfg, MODELS)

    def forward(self, features, gts, image_size):
        return self.match_cfg(features, gts, image_size)
