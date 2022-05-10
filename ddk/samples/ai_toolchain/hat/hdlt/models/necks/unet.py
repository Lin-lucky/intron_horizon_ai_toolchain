import math
from typing import Sequence

import numpy as np
import torch
from torch import nn
from torch.nn import Module
from torch.nn import functional as F

from ...ops import SeparableConvModule2d
from ..registry import MODELS

__all__ = ["Unet"]


class UpscaleAndFusion(Module):
    def __init__(
        self,
        vertical_in_channels,
        out_channels,
        horizontal_in_channels,
        bn_kwargs={},
        act_type=nn.ReLU,
        use_deconv=False,
        dw_with_relu=False,
    ):
        super(UpscaleAndFusion, self).__init__()
        self.horizontal_conv = SeparableConvModule2d(
            horizontal_in_channels,
            out_channels,
            kernel_size=3,
            padding=1,
            dw_norm_cfg=bn_kwargs,
            dw_act_type=act_type if dw_with_relu else None,
            pw_norm_cfg=bn_kwargs,
            pw_act_type=act_type,
        )

        self.vertical_conv = SeparableConvModule2d(
            vertical_in_channels,
            out_channels,
            kernel_size=3,
            padding=1,
            dw_norm_cfg=bn_kwargs,
            dw_act_type=act_type if dw_with_relu else None,
            pw_norm_cfg=bn_kwargs,
            pw_act_type=act_type,
        )

        self.fuse_conv = SeparableConvModule2d(
            out_channels,
            out_channels,
            kernel_size=3,
            padding=1,
            dw_norm_cfg=bn_kwargs,
            dw_act_type=act_type if dw_with_relu else None,
            pw_norm_cfg=bn_kwargs,
            pw_act_type=None,
        )

        self.relu = act_type()

    def forward(self, vertical_input, horizontal_input):
        horizontal_input = self.horizontal_conv(horizontal_input)

        vertical_input = self.vertical_conv(vertical_input)
        vertical_input = F.interpolate(
            vertical_input,
            size=None,
            scale_factor=2,
            mode="bilinear",
            # not pass None for align_corners to avoid warining
            align_corners=False,
            recompute_scale_factor=True,
        )

        horizontal_input = self.fuse_conv(horizontal_input)

        return self.relu(torch.add(horizontal_input, vertical_input))


@MODELS.register_module
class Unet(Module):
    """
    Unet segmentation neck structure.
    Built with separable convolution layers.

    Args:
        base_channels (int):
            Output channel number of the output layer of scale 1.
        bn_kwargs (dict, optional): Keyword arguments for BN layer.
            Defaults to {}.
        use_deconv (bool, optional): Whether user deconv for upsampling layer.
            Defaults to False.
        dw_with_relu (bool, optional):
            Whether user relu after the depthwise conv in SeparableConv.
            Defaults to False.
        output_scales (Sequence, optional): The scale of each output layer.
            Defaults to (4, 8, 16, 32, 64).
    """

    def __init__(
        self,
        base_channels: int,
        bn_kwargs: dict = {},
        act_type=nn.ReLU,
        use_deconv: bool = False,
        dw_with_relu: bool = False,
        output_scales: Sequence = (4, 8, 16, 32, 64),
        **kwargs
    ):
        super(Unet, self).__init__()
        self.base_channels = base_channels
        self.bn_kwargs = bn_kwargs
        self.act_type = act_type
        self.use_deconv = use_deconv
        self.dw_with_relu = dw_with_relu
        self.output_scales = output_scales

        self.conv64_1 = SeparableConvModule2d(
            base_channels * 32,
            base_channels * 64,
            kernel_size=3,
            stride=2,
            padding=1,
            dw_norm_cfg=self.bn_kwargs,
            dw_act_type=act_type if self.dw_with_relu else None,
            pw_norm_cfg=self.bn_kwargs,
            pw_act_type=act_type,
        )
        self.conv64_2 = SeparableConvModule2d(
            base_channels * 64,
            base_channels * 64,
            kernel_size=3,
            padding=1,
            dw_norm_cfg=self.bn_kwargs,
            dw_act_type=act_type if self.dw_with_relu else None,
            pw_norm_cfg=self.bn_kwargs,
            pw_act_type=act_type,
        )

        stage_num = int(math.log2(np.max(64 / np.array(output_scales))))

        self.stages = nn.ModuleList()
        for stage in range(stage_num):
            vertical_in_channels = int(base_channels * (2 ** (6 - stage)))
            out_channels = int(vertical_in_channels / 2)
            horizontal_in_channels = out_channels
            self.stages.append(
                self._make_upscale_and_fusion(
                    vertical_in_channels, out_channels, horizontal_in_channels
                )
            )

        self.output_layers = nn.ModuleDict()
        for output_scale in output_scales:
            channels = int(base_channels * output_scale)
            self.output_layers[
                self._out_layer_scale_to_name(output_scale)
            ] = self._make_output_layers(channels, channels)

    def _out_layer_scale_to_name(self, output_scale):
        return "out_layer_scale_%d" % output_scale

    def _make_output_layers(self, in_channels, out_channels):
        return SeparableConvModule2d(
            in_channels,
            out_channels,
            kernel_size=3,
            padding=1,
            dw_norm_cfg=self.bn_kwargs,
            dw_act_type=self.act_type if self.dw_with_relu else None,
            pw_norm_cfg=self.bn_kwargs,
            pw_act_type=self.act_type,
        )

    def _make_upscale_and_fusion(
        self, vertical_in_channels, out_channels, horizontal_in_channels,
    ):
        return UpscaleAndFusion(
            vertical_in_channels,
            out_channels,
            horizontal_in_channels,
            self.bn_kwargs,
            self.act_type,
            self.use_deconv,
            self.dw_with_relu,
        )

    def forward(self, inputs):
        ret_outputs = []
        current_scale = 64

        inputs = list(reversed(inputs))

        x = self.conv64_1(inputs[0])
        x = self.conv64_2(x)
        if current_scale in self.output_scales:
            ret_outputs.append(
                self.output_layers[
                    self._out_layer_scale_to_name(current_scale)
                ](x)
            )

        for stage, mod in enumerate(self.stages):
            x = mod(x, inputs[stage])
            current_scale /= 2

            if current_scale in self.output_scales:
                ret_outputs.append(
                    self.output_layers[
                        self._out_layer_scale_to_name(current_scale)
                    ](x)
                )

        return list(reversed(ret_outputs))
