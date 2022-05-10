# Copyright (c) Horizon Robotics. All rights reserved.

from typing import Tuple, Union

import torch
import torch.nn as nn

__all__ = ["ConvModule2d"]


class ConvModule2d(nn.Module):
    """
    A conv block that bundles conv/norm/activation layers.

    Args:
        in_channels (int): Same as nn.Conv2d.
        out_channels (int): Same as nn.Conv2d.
        kernel_size (int | tuple[int]): Same as nn.Conv2d.
        stride (int | tuple[int]): Same as nn.Conv2d.
        padding (int | tuple[int]): Same as nn.Conv2d.
        dilation (int | tuple[int]): Same as nn.Conv2d.
        groups (int): Same as nn.Conv2d.
        bias (bool): Same as nn.Conv2d.
        padding_mode (str): Same as nn.Conv2d.
        norm_type (nn.Module): Type of normalization layer.
        norm_cfg (dict): Dict for normalization layer.
        act_type (nn.Module): Type of activation layer.
        act_cfg (dict): Dict for activation layer.
    """

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel_size: Union[int, Tuple[int, int]],
        stride: Union[int, Tuple[int, int]] = 1,
        padding: Union[int, Tuple[int, int]] = 0,
        dilation: Union[int, Tuple[int, int]] = 1,
        groups: int = 1,
        bias: bool = True,
        padding_mode: str = "zeros",
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
    ):
        super(ConvModule2d, self).__init__()
        self.conv = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size,
            stride,
            padding,
            dilation,
            groups,
            bias,
            padding_mode,
        )
        self.norm = (
            None if norm_type is None else norm_type(out_channels, **norm_cfg)
        )
        self.act = None if act_type is None else act_type(**act_cfg)
        # conv_list = [conv, norm, act]
        # self.conv_list = [layer for layer in conv_list if layer is not None]

    def forward(self, x):
        x = self.conv(x)
        if self.norm:
            x = self.norm(x)
        if self.act:
            x = self.act(x)
        return x

    # def fuse_model(self):
    #    # for qat mappings
    #    from horizon_plugin_pytorch import quantization

    #    fuse_list = ["0", "1", "2"]
    #    self.fuse_list = fuse_list[: len(self.conv_list)]
    #    torch.quantization.fuse_modules(
    #        self,
    #        self.fuse_list,
    #        inplace=True,
    #        fuser_func=quantization.fuse_known_modules,
    #    )
