import math
from typing import Union

import numpy as np
import torch
import torch.nn as nn

from hdlt.ops import ConvModule2d, SeparableConvModule2d

from ..registry import MODELS


class Scale(nn.Module):
    """A learnable scale parameter.

    This layer scales the input by a learnable factor. It multiplies a
    learnable scale parameter of shape (1,) with input of any shape.

    Args:
        scale (float): Initial value of scale factor. Default: 1.0
    """

    def __init__(self, scale=1.0):
        super(Scale, self).__init__()
        self.scale = nn.Parameter(torch.tensor(scale, dtype=torch.float))

    def forward(self, x):
        return x * self.scale


class DecodeBlock(nn.Module):
    def __init__(
        self,
        in_channel=64,
        stacked_convs=4,
        feat_channel=64,
        shared_norm=False,
        strides=[8, 16, 32, 64, 128],
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias=True,
    ):
        super(DecodeBlock, self).__init__()
        self.shared_norm = shared_norm

        self.convs = nn.ModuleList()
        self.norm_act = nn.ModuleList()
        for i in range(stacked_convs):
            self.convs.append(
                SeparableConvModule2d(
                    in_channels=feat_channel if i != 0 else in_channel,
                    out_channels=feat_channel,
                    kernel_size=3,
                    padding=1,
                    bias=bias,
                    dw_norm_type=None,
                    dw_act_type=None,
                    pw_norm_type=None,
                    pw_act_type=None,
                )
            )

            if not self.shared_norm:
                norm_acts = nn.ModuleList()
                for j in range(len(strides)):
                    if act_type is not None:
                        norm_act = nn.Sequential(
                            norm_type(feat_channel, **norm_cfg),
                            act_type(**act_cfg),
                        )
                    else:
                        norm_act = nn.Sequential(
                            norm_type(feat_channel, **norm_cfg)
                        )
                    norm_acts.append(norm_act)
                self.norm_act.append(norm_acts)
            else:
                if act_type is not None:
                    norm_act = nn.Sequential(
                        norm_type(feat_channel, **norm_cfg),
                        act_type(**act_cfg),
                    )
                else:
                    norm_act = nn.Sequential(
                        norm_type(feat_channel, **norm_cfg)
                    )
                self.norm_act.append(norm_act)

    def forward(self, inputs):
        outs = list()
        for i, x in enumerate(inputs):
            for conv, norm_act in zip(self.convs, self.norm_act):
                x = conv(x)
                if self.shared_norm:
                    x = norm_act(x)
                else:
                    x = norm_act[i](x)
            outs.append(x)
        return outs


@MODELS.register_module
class FCOSHead(nn.Module):
    """
    Fcos head structure.

    Args:
        num_classes (int):
            Number of classes.
        in_channel (int):
            Input channel.
        stacked_convs (list)：
            Number of stack conv.
        feat_channel (int):
            Channel number for stack conv.
        strides (list):
            List of stride for ecah layer.
        shared_norm (bool):
            Whether share norm setting.
        need_centerness (bool):
            Wheter use centerness target.
        centerness_on_reg (bool):
            Wheter use centerness for reg branch.
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
        num_classes=80,
        in_channel=64,
        stacked_convs=4,
        feat_channel=64,
        strides=[8, 16, 32, 64, 128],
        shared_norm=False,
        need_centerness=True,
        centerness_on_reg=True,
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias=True,
    ):
        super(FCOSHead, self).__init__()
        self.need_centerness = need_centerness
        self.centerness_on_reg = centerness_on_reg
        self.scales = nn.ModuleList(Scale(1.0) for _ in range(len(strides)))
        self.reg_decode = DecodeBlock(
            in_channel=in_channel,
            stacked_convs=stacked_convs,
            feat_channel=feat_channel,
            shared_norm=shared_norm,
            strides=strides,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=True,
        )

        self.cls_decode = DecodeBlock(
            in_channel=in_channel,
            stacked_convs=stacked_convs,
            feat_channel=feat_channel,
            shared_norm=shared_norm,
            strides=strides,
            norm_type=norm_type,
            norm_cfg=norm_cfg,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=True,
        )

        self.cls_conv = ConvModule2d(
            feat_channel,
            num_classes,
            3,
            padding=1,
            norm_type=None,
            act_type=None,
            bias=bias,
        )
        self.reg_conv = ConvModule2d(
            feat_channel,
            4,
            3,
            padding=1,
            norm_type=None,
            act_type=act_type,
            act_cfg=act_cfg,
            bias=bias,
        )
        if self.need_centerness:
            self.centerness_conv = ConvModule2d(
                feat_channel,
                1,
                3,
                padding=1,
                norm_type=None,
                act_type=None,
                bias=bias,
            )

        self._init_weights()

    def forward(self, inputs):
        cls_pred = list()
        reg_pred = list()
        centerness_pred = list()

        regs = self.reg_decode(inputs)
        clses = self.cls_decode(inputs)

        for cls in clses:
            x = self.cls_conv(cls)
            x = x.permute(0, 2, 3, 1).contiguous()
            cls_pred.append(x)
            if self.need_centerness and not self.centerness_on_reg:
                x = self.centerness_conv(cls)
                x = x.permute(0, 2, 3, 1).contiguous()
                centerness_pred.append(x)

        for scale, reg in zip(self.scales, regs):
            x = self.reg_conv(reg)
            x = scale(x)
            x = x.permute(0, 2, 3, 1).contiguous()
            reg_pred.append(x)
            if self.need_centerness and self.centerness_on_reg:
                x = self.centerness_conv(reg)
                x = x.permute(0, 2, 3, 1).contiguous()
                centerness_pred.append(x)

        if self.need_centerness:
            return [cls_pred, reg_pred, centerness_pred]
        else:
            return [cls_pred, reg_pred]

    def _init_weights(self, prior_prob=0.01):
        """Initialize weights of the head."""
        bias = self.cls_conv.conv.bias
        with torch.no_grad():
            bias[:] += -np.log((1 - prior_prob) / prior_prob)
        self.cls_conv.conv.bias = torch.nn.Parameter(
            bias, requires_grad=bias.requires_grad
        )
        nn.init.normal_(self.cls_conv.conv.weight, 0.0, 0.01)
        nn.init.normal_(self.reg_conv.conv.weight, 0.0, 0.01)
        nn.init.normal_(self.centerness_conv.conv.weight, 0.0, 0.01)
