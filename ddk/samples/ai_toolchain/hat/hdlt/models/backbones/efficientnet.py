# Copyright (c) Horizon Robotics. All rights reserved.

from typing import Union

import torch
import torch.nn as nn

from hdlt.ops import ConvModule2d

from ..registry import MODELS

__all__ = ["EfficientNetB0"]


class MBBlock(nn.Module):
    """
    A basic block for Efficientnet.

    Args:
        in_channels (int): Input channels.
        out_channels (int): Output channels.
        kernel_size (int): kernel for depthwise conv.
        pad (int): pad for depthwise conv.
        stride (int): Stride for first conv.
        expand (float): Expand for MB block.
        norm_type (nn.Module): Type of normalization layer.
        norm_cfg (dict): Dict for normalization layer.
        act_type (nn.Module): Type of activation layer.
        act_cfg (dict): Dict for activation layer.
        bias (bool): Whether to use bias in module.
    """

    def __init__(
        self,
        in_channel: int,
        out_channel: int,
        kernel_size: int = 3,
        pad: int = 1,
        stride: int = 2,
        expand: float = 2.0,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias: bool = True,
    ):
        super(MBBlock, self).__init__()
        self.expand = expand
        expand_channel = int(in_channel * expand)
        if self.expand > 1:
            self.expand_conv = ConvModule2d(
                in_channel,
                expand_channel,
                1,
                padding=0,
                stride=1,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                act_type=act_type,
                act_cfg=act_cfg,
                bias=bias,
            )
        self.depth = ConvModule2d(
            expand_channel,
            expand_channel,
            kernel_size=kernel_size,
            padding=pad,
            stride=stride,
            groups=expand_channel,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )

        self.project = ConvModule2d(
            expand_channel,
            out_channel,
            1,
            padding=0,
            stride=1,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=None,
            bias=bias,
        )

    def forward(self, x):
        out = x
        if self.expand > 1:
            out = self.expand_conv(x)
        out = self.depth(out)
        out = self.project(out)
        if out.shape == x.shape:
            out = out + x
        return out


class StemBlock(nn.Module):
    """
    A stem block for resnet.

    Args:
        out_channel (int): Output channels.
        kernel_size (int): kernel for depthwise conv.
        pad (int): pad for depthwise conv.
        norm_type (nn.Module): Type of normalization layer.
        norm_cfg (dict): Dict for normalization layer.
        act_type (nn.Module): Type of activation layer.
        act_cfg (dict): Dict for activation layer.
        bias (bool): Whether to use bias in module.
    """

    def __init__(
        self,
        out_channel: int,
        kernel_size: int,
        pad: int,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias: bool = True,
    ):
        super(StemBlock, self).__init__()
        self.stem = self.getStem(
            out_channel=out_channel,
            kernel_size=kernel_size,
            pad=pad,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )

    def getStem(
        self,
        out_channel,
        kernel_size,
        pad,
        norm_type,
        norm_cfg,
        act_type,
        act_cfg,
        bias,
    ):
        return ConvModule2d(
            3,
            out_channel,
            kernel_size,
            padding=pad,
            stride=2,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )

    def forward(self, x):
        return self.stem(x)


class StageBlock(nn.Module):
    """
    Stage block for EfficientNet.
    Args:
        in_channel (int): Input channels.
        out_channel (int): Output channels.
        kernel_size (int): kernel for depthwise conv.
        pad (int): pad for depthwise conv.
        stride (int): Stride for first conv.
        expand (float): Expand for MB block.
        repeat (int):  Repeat number of Stage.
        norm_type (nn.Module): Type of normalization layer.
        norm_cfg (dict): Dict for normalization layer.
        act_type (nn.Module): Type of activation layer.
        act_cfg (dict): Dict for activation layer.
        bias (bool): Whether to use bias in module.
    """

    def __init__(
        self,
        in_channel: int,
        out_channel: int,
        kernel_size: int,
        stride: int,
        pad: int,
        expand: float,
        repeat: int,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias: bool = True,
    ):
        super(StageBlock, self).__init__()
        self.conv = self.getBlock(
            in_channel=in_channel,
            out_channel=out_channel,
            kernel_size=kernel_size,
            pad=pad,
            stride=stride,
            expand=expand,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )
        self.repeat = repeat
        if repeat > 1:
            self.blocks = nn.Sequential(
                *(
                    self.getBlock(
                        out_channel,
                        out_channel,
                        kernel_size=kernel_size,
                        pad=pad,
                        stride=1,
                        expand=expand,
                        norm_type=norm_type,
                        norm_cfg=norm_cfg,
                        act_type=act_type,
                        act_cfg=act_cfg,
                        bias=bias,
                    )
                    for _ in range(repeat - 1)
                )
            )

    def forward(self, x):
        x = self.conv(x)
        if self.repeat > 1:
            x = self.blocks(x)
        return x

    def getBlock(
        self,
        in_channel,
        out_channel,
        kernel_size,
        pad,
        stride,
        expand,
        norm_type,
        norm_cfg,
        act_type,
        act_cfg,
        bias,
    ):
        return MBBlock(
            in_channel=in_channel,
            out_channel=out_channel,
            kernel_size=kernel_size,
            pad=pad,
            stride=stride,
            expand=expand,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )


class EfficientNet(nn.Module):
    """
    EfficientNet base structure.

    Args:
        width_coefficient (float): Width coefficient.
        depth_coefficient (float): Depth coefficient.
        norm_type (nn.Module): Type of normalization layer.
        norm_cfg (dict): Dict for normalization layer.
        act_type (nn.Module): Type of activation layer.
        act_cfg (dict): Dict for activation layer.
        bias (bool): Whether to use bias in module.
    """

    def __init__(
        self,
        width_coefficient: float = 1.0,
        depth_coefficient: float = 1.0,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias: bool = True,
    ):
        super(EfficientNet, self).__init__()
        self.channels = [
            int(x * width_coefficient)
            for x in [32, 16, 24, 40, 80, 112, 192, 320]
        ]
        self.repeats = [
            int(x * depth_coefficient) for x in [1, 2, 2, 3, 3, 4, 1]
        ]
        self.expands = [1, 6, 6, 6, 6, 6, 6]
        self.strides = [1, 2, 2, 2, 1, 2, 1]
        self.kernels = [3, 3, 5, 3, 5, 5, 3]
        self.paddings = [1, 1, 2, 1, 2, 2, 1]
        self.stem = StemBlock(
            out_channel=self.channels[0],
            kernel_size=3,
            pad=1,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )
        self.stage_blocks = nn.ModuleList(
            (
                StageBlock(
                    in_channel=self.channels[i],
                    out_channel=self.channels[i + 1],
                    kernel_size=self.kernels[i],
                    pad=self.paddings[i],
                    stride=self.strides[i],
                    repeat=self.repeats[i],
                    expand=self.expands[i],
                    norm_type=norm_type,
                    norm_cfg=norm_cfg,
                    act_type=act_type,
                    act_cfg=act_cfg,
                    bias=bias,
                )
                for i in range(len(self.repeats))
            )
        )

    def forward(self, x):
        x = self.stem(x)
        outputs = list()
        for i, stage in enumerate(self.stage_blocks):
            if self.strides[i] == 2:
                outputs.append(x)
            x = stage(x)

        outputs.append(x)
        return outputs


@MODELS.register_module
class EfficientNetB0(EfficientNet):
    """
    EfficientNet b0 structure.

    Args:
        norm_type (nn.Module): Type of normalization layer.
        norm_cfg (dict): Dict for normalization layer.
        act_type (nn.Module): Type of activation layer.
        act_cfg (dict): Dict for activation layer.
        bias (bool): Whether to use bias in module.
    """

    def __init__(
        self,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias: bool = True,
    ):
        super(EfficientNetB0, self).__init__(
            width_coefficient=1.0,
            depth_coefficient=1.0,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )
