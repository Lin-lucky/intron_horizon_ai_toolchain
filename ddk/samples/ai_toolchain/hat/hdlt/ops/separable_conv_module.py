# Copyright (c) Horizon Robotics. All rights reserved.

from typing import Tuple, Union

import torch
import torch.nn as nn

from .conv_module import ConvModule2d

__all__ = ["SeparableConvModule2d"]


class SeparableConvModule2d(nn.Sequential):
    """
    Depthwise sparable convolution module.


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
        dw_norm_type (nn.Module): Type of normalization layer in dw conv.
        dw_norm_cfg (dict): Dict for normalization layer in dw conv.
        dw_act_type (nn.Module): Type of activation layer in dw conv.
        dw_act_cfg (dict): Dict for activation layer in dw conv.
        pw_norm_type (nn.Module): Type of normalization layer in pw conv.
        pw_norm_cfg (dict): Dict for normalization layer in pw conv.
        pw_act_type (nn.Module): Type of activation layer in pw conv.
        pw_act_cfg (dict): Dict for activation layer in pw conv.
    """

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel_size: Union[int, Tuple[int, int]],
        stride: Union[int, Tuple[int, int]] = 1,
        padding: Union[int, Tuple[int, int]] = 0,
        dilation: Union[int, Tuple[int, int]] = 1,
        bias: bool = True,
        padding_mode: str = "zeros",
        dw_norm_type: nn.Module = nn.BatchNorm2d,
        dw_norm_cfg: dict = dict(momentum=0.1),
        dw_act_type: nn.Module = nn.ReLU,
        dw_act_cfg: dict = dict(inplace=True),
        pw_norm_type: nn.Module = nn.BatchNorm2d,
        pw_norm_cfg: dict = dict(momentum=0.1),
        pw_act_type: nn.Module = nn.ReLU,
        pw_act_cfg: dict = dict(inplace=True),
    ):
        super(SeparableConvModule2d, self).__init__(
            ConvModule2d(
                in_channels,
                in_channels,
                kernel_size,
                stride,
                padding,
                dilation,
                in_channels,
                bias,
                padding_mode,
                dw_norm_type,
                dw_norm_cfg,
                dw_act_type,
                dw_act_cfg,
            ),
            ConvModule2d(
                in_channels,
                out_channels,
                1,
                bias=bias,
                norm_type=pw_norm_type,
                norm_cfg=pw_norm_cfg,
                act_type=pw_act_type,
                act_cfg=pw_act_cfg,
            ),
        )

    def fuse_model(self):
        # for qat mappings
        from horizon_plugin_pytorch import quantization

        getattr(self, "0").fuse_model()
        getattr(self, "1").fuse_model()
