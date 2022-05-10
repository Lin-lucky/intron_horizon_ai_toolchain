import torch
import torch.nn as nn

import numpy as np

from ..registry import MODELS
from hdlt.common.registry import build_from_cfg

@MODELS.register_module
class Yolov4Target(nn.Module):
    def __init__(self, match_cfg):
        super(Yolov4Target, self).__init__()
        self.match_cfg = build_from_cfg(match_cfg, MODELS)
     
    def forward(self, features, anchors, gts, image_size):
        return self.match_cfg(features, anchors, gts, image_size)
             
