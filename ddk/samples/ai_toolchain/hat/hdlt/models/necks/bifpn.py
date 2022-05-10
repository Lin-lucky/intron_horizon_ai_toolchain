import math
from typing import Union

import torch
import torch.nn as nn

from hdlt.common.registry import build_from_cfg
from hdlt.ops import ConvModule2d, SeparableConvModule2d

from ..registry import MODELS


@MODELS.register_module
class MaxPoolDownSample(nn.MaxPool2d):
    def __init__(
        self,
        in_channel,
        out_channel,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        bias: bool = True,
        **kwargs
    ):
        super(MaxPoolDownSample, self).__init__(**kwargs)
        if in_channel != out_channel:
            self.project = ConvModule2d(
                in_channel,
                out_channel,
                1,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                act_type=None,
                bias=bias,
            )

    def forward(self, x):
        if hasattr(self, "project"):
            x = self.project(x)
        return super(MaxPoolDownSample, self).forward(x)


@MODELS.register_module
class ConvDownsample(nn.Module):
    def __init__(
        self,
        in_channel,
        out_channel,
        kernel_size=3,
        padding=1,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        bias: bool = True,
        **kwargs
    ):
        super(ConvDownsample, self).__init__()
        self.down = SeparableConvModule2d(
            in_channels=in_channel,
            out_channels=out_channel,
            kernel_size=kernel_size,
            stride=2,
            padding=padding,
            bias=bias,
            dw_norm_type=None,
            dw_act_type=None,
            pw_norm_type=norm_type,
            pw_norm_cfg=norm_cfg,
            pw_act_type=None,
        )

    def forward(self, x):
        return self.down(x)


@MODELS.register_module
class MixConvDownsample(nn.Module):
    def __init__(
        self,
        in_channel,
        out_channel,
        kernel_sizes=[3, 5],
        paddings=[1, 2],
        dilations=[1, 1],
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        bias: bool = True,
        **kwargs
    ):
        super(MixConvDownsample, self).__init__()
        self.down = MixConvModule2d(
            in_channels=in_channel,
            out_channels=out_channel,
            kernel_sizes=kernel_sizes,
            stride=2,
            paddings=paddings,
            dilations=dilations,
            bias=bias,
            dw_norm_type=None,
            dw_act_type=None,
            pw_norm_type=norm_type,
            pw_norm_cfg=norm_cfg,
            pw_act_type=None,
        )

    def forward(self, x):
        return self.down(x)


class UpBlock(nn.Module):
    def __init__(
        self,
        in_channel: int,
        up_in_channel: int,
        out_channel: int,
        attention: bool = False,
        epsilon: float = 1e-16,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        bias: bool = False,
    ):
        super(UpBlock, self).__init__()
        self.attention = attention
        self.epsilon = epsilon
        if self.attention:
            self.w1 = nn.Parameter(
                torch.ones((2, 1, 1, 1, 1), dtype=torch.float32),
                requires_grad=True,
            )
        self.relu = nn.ReLU()
        self.upsample = nn.Upsample(scale_factor=2)
        self.conv = SeparableConvModule2d(
            in_channels=out_channel,
            out_channels=out_channel,
            kernel_size=3,
            padding=1,
            bias=bias,
            dw_norm_type=None,
            dw_act_type=None,
            pw_norm_type=norm_type,
            pw_norm_cfg=norm_cfg,
            pw_act_type=None,
        )
        if in_channel != out_channel:
            self.in_project = ConvModule2d(
                in_channels=in_channel,
                out_channels=out_channel,
                kernel_size=1,
                bias=bias,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                act_type=None,
            )
        if up_in_channel != out_channel:
            self.up_project = ConvModule2d(
                in_channels=up_in_channel,
                out_channels=out_channel,
                kernel_size=1,
                bias=bias,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                act_type=None,
            )

    def forward(self, inputs):
        x = inputs[0]
        up = inputs[1]
        if hasattr(self, "in_project"):
            x = self.in_project(x)
        if hasattr(self, "up_project"):
            up = self.up_project(up)
        up = self.upsample(up)
        if self.attention:
            w1 = self.relu(self.w1)
            weight = w1 / (torch.sum(w1, dim=0) + self.epsilon)
            x = weight[0] * x
            up = weight[1] * up
        return self.conv(self.relu(x + up))


class DownBlock(nn.Module):
    def __init__(
        self,
        in_channel: int = 64,
        down_in_channel: int = 64,
        out_channel: int = 64,
        attention: bool = False,
        epsilon: float = 1e-16,
        need_prev_in: bool = True,
        down_sample=dict(
            type="MaxPoolDownSample", kernel_size=3, stride=2, padding=1
        ),
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias: bool = False,
    ):
        super(DownBlock, self).__init__()
        self.attention = attention
        self.epsilon = epsilon
        self.need_prev_in = need_prev_in
        if self.attention:
            if self.need_prev_in:
                self.w1 = nn.Parameter(
                    torch.ones((3, 1, 1, 1, 1), dtype=torch.float32),
                    requires_grad=True,
                )
            else:
                self.w1 = nn.Parameter(
                    torch.ones((2, 1, 1, 1, 1), dtype=torch.float32),
                    requires_grad=True,
                )

        self.relu = nn.ReLU()
        down_sample["norm_type"] = norm_type
        down_sample["norm_cfg"] = norm_cfg
        down_sample["in_channel"] = out_channel
        down_sample["out_channel"] = out_channel

        self.downsample = build_from_cfg(down_sample, MODELS)

        if in_channel != out_channel:
            self.in_project = ConvModule2d(
                in_channels=in_channel,
                out_channels=out_channel,
                kernel_size=1,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                act_type=None,
                bias=bias,
            )

        if out_channel != down_in_channel:
            self.down_project = ConvModule2d(
                in_channels=down_in_channel,
                out_channels=out_channel,
                kernel_size=1,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                act_type=None,
                bias=bias,
            )

        self.conv = SeparableConvModule2d(
            in_channels=out_channel,
            out_channels=out_channel,
            kernel_size=3,
            padding=1,
            bias=bias,
            dw_norm_type=None,
            dw_act_type=None,
            pw_norm_type=norm_type,
            pw_norm_cfg=norm_cfg,
            pw_act_type=None,
        )

    def forward(self, inputs):
        x1 = inputs[0]
        x2 = inputs[1]
        down = inputs[2]
        if hasattr(self, "in_project"):
            x1 = self.in_project(x1)

        if hasattr(self, "down_project"):
            down = self.down_project(down)

        down = self.downsample(down)

        if self.attention:
            w1 = self.relu(self.w1)
            weight = w1 / (torch.sum(w1, dim=0) + self.epsilon)
            if self.need_prev_in:
                x1 = weight[0] * x1
                x2 = weight[1] * x2
                down = weight[2] * down
            else:
                x2 = weight[0] * x2
                down = weight[1] * down

        if self.need_prev_in:
            return self.conv(self.relu(x1 + x2 + down))
        else:
            return self.conv(self.relu(x2 + down))


class BiFPNBlock(nn.Module):
    def __init__(
        self,
        in_channels: list = [64, 64, 64, 64, 64],
        out_channels: list = [64, 64, 64, 64, 64],
        num_out: int = 5,
        down_sample=dict(
            type="MaxPoolDownSample", kernel_size=3, stride=2, padding=1
        ),
        attention: bool = False,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        bias: bool = False,
        **kwargs
    ):
        super(BiFPNBlock, self).__init__()
        num_up = len(in_channels) - 1
        self.up_blocks = nn.ModuleList(
            UpBlock(
                in_channel=in_channels[-i - 2],
                up_in_channel=out_channels[-i - 1],
                out_channel=out_channels[-i - 2],
                attention=attention,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                bias=bias,
            )
            for i in range(num_up)
        )
        self.down_blocks = nn.ModuleList(
            DownBlock(
                in_channel=in_channels[i + 1],
                down_in_channel=out_channels[i],
                out_channel=out_channels[i + 1],
                attention=attention,
                down_sample=down_sample,
                need_prev_in=True if i != len(out_channels) - 2 else False,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                bias=bias,
            )
            for i in range(num_out - 1)
        )

    def forward(self, inputs):
        up_in = inputs[::-1]
        down_in = inputs[1:]
        ups = [up_in[0]]
        for i, block in enumerate(self.up_blocks):
            ups.append(block([up_in[i + 1], ups[-1]]))
        outs = [ups[-1]]
        for i, block in enumerate(self.down_blocks):
            outs.append(block([down_in[i], ups[-i - 2], outs[-1]]))

        return outs


class ExtractBlock(nn.Module):
    def __init__(
        self,
        in_channel: int = 64,
        out_channels: int = [64, 64],
        down_sample=dict(
            type="MaxPoolDownSample", kernel_size=3, stride=2, padding=1
        ),
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        bias: bool = True,
    ):
        super(ExtractBlock, self).__init__()

        num_extract = len(out_channels)
        self.extract = nn.ModuleList()
        down_sample["norm_type"] = norm_type
        down_sample["norm_cfg"] = norm_cfg

        for i in range(num_extract):
            down_sample["in_channel"] = (
                in_channel if i == 0 else out_channels[i - 1]
            )
            down_sample["out_channel"] = out_channels[i]
            self.extract.append(build_from_cfg(down_sample, MODELS))

    def forward(self, x):
        outs = list()
        for ext in self.extract:
            x = ext(x)
            outs.append(x)
        return outs


@MODELS.register_module
class BiFPN(nn.Module):
    """
    BiFPN neck structure.

    P7_0 -------------------------> P7_2 -------->
       |-------------|                ↑
    P6_0 ---------> P6_1 ---------> P6_2 -------->
       |-------------|--------------↑ ↑
                     ↓                |
    P5_0 ---------> P5_1 ---------> P5_2 -------->
       |-------------|--------------↑ ↑
                     ↓                |
    P4_0 ---------> P4_1 ---------> P4_2 -------->
       |-------------|--------------↑ ↑
                     |--------------↓ |
    P3_0 -------------------------> P3_2 -------->

    Args:
        in_channels (int):
            Input channel number.
        out_channels (int):
            Output channel number.
        num_out (int):
            Number of outputs.
        repeat (int):
            Repeat numbers of bifpn block.
        down_sample (dict):
            Down sample type for bifpn block.
        attention (bool) :
            Whether use attention.
        norm_type (nn.Module):
            Type of normalization layer.
        norm_cfg (dict):
            Dict for normalization layer.
        act_type (nn.Module):
            Type of activation layer.
        act_cfg (dict):
            Dict for activation layer.
        bias (bool):
            Whether to use bias in module.
    """

    def __init__(
        self,
        in_channels: list = [40, 112, 320],
        out_channels: list = [64, 64, 64, 64, 64],
        num_out: int = 5,
        repeat: int = 3,
        down_sample=dict(
            type="MaxPoolDownSample", kernel_size=3, stride=2, padding=1
        ),
        attention: bool = False,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        bias: bool = False,
        **kwargs
    ):
        super(BiFPN, self).__init__()
        self.num_in = len(in_channels)
        if len(out_channels) > len(in_channels):
            self.extract = ExtractBlock(
                in_channel=in_channels[-1],
                out_channels=out_channels[len(in_channels) :],
                down_sample=down_sample,
                norm_type=norm_type,
                norm_cfg=norm_cfg,
                bias=bias,
            )
            fpn_inchannels = in_channels + out_channels[len(in_channels) :]

        else:
            fpn_inchannels = in_channels

        self.bifpn = nn.Sequential(
            *[
                BiFPNBlock(
                    in_channels=fpn_inchannels if i == 0 else out_channels,
                    out_channels=out_channels,
                    num_out=num_out  if i == repeat - 1 else len(out_channels),
                    attention=attention,
                    down_sample=down_sample,
                    norm_type=norm_type,
                    norm_cfg=norm_cfg,
                    bias=bias,
                )
                for i in range(repeat)
            ]
        )

    def forward(self, inputs):

        inputs = inputs[-self.num_in :]
        if hasattr(self, "extract"):
            ex_inputs = self.extract(inputs[-1])
            inputs.extend(ex_inputs)
        return self.bifpn(inputs)
