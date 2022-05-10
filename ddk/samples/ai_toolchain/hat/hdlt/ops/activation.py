# Copyright (c) Horizon Robotics. All rights reserved.

import torch
import torch.nn as nn

__all__ = ["Swish"]


class Swish(nn.Module):
    def __init__(self):
        super(Swish, self).__init__()

    def forward(self, x):
        x = x * torch.sigmoid(x)
        return x
