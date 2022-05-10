from collections.abc import Sequence
from typing import Union

import torch
from torch import nn
from torch.nn import Module

from ...ops import ConvModule2d
from ..registry import MODELS

__all__ = ["SegHead"]


@MODELS.register_module
class SegHead(Module):
    r"""
    Model head for segmentation tasks.
    """

    def __init__(
        self,
        input_channels: Union[int, Sequence],
        output_channels: Union[int, Sequence],
        bn_kwargs: dict = {},
        act_type: Module = None,
    ):
        super(SegHead, self).__init__()

        if isinstance(input_channels, int):
            input_channels = [input_channels]
        if isinstance(output_channels, int):
            output_channels = [output_channels]

        self.input_channels = input_channels
        self.output_channels = output_channels

        self.heads = nn.ModuleList()
        self.dequants = nn.ModuleList()

        for in_channels, out_channels in zip(input_channels, output_channels):
            self.heads.append(
                ConvModule2d(
                    in_channels,
                    out_channels,
                    kernel_size=1,
                    norm_cfg=bn_kwargs,
                    act_type=act_type,
                )
            )

    def forward(self, inputs):
        input_list = inputs if isinstance(inputs, Sequence) else [inputs]
        ret_outputs = []
        for x, head in zip(input_list, self.heads):
            x = head(x)
            if not self.training:
                x = torch.argmax(x, dim=1, keepdim=True)
            ret_outputs.append(x)

        return ret_outputs if isinstance(inputs, Sequence) else ret_outputs[0]
