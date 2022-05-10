# Copyright (c) Horizon Robotics. All rights reserved.

import torch.nn as nn

from hdlt.ops import ConvModule2d, SeparableConvModule2d

from ..registry import MODELS

__all__ = ["MobileNetV1"]


@MODELS.register_module
class MobileNetV1(nn.Module):
    def __init__(
        self,
        alpha=1.0,
        bias=True,
        dw_with_relu=True,
        bn_kwargs={},
        act_type=nn.ReLU,
        num_classes=1000,
        include_top=True,
        with_loss=True,
    ):
        super(MobileNetV1, self).__init__()
        self.alpha = alpha
        self.bias = bias
        self.bn_kwargs = bn_kwargs
        self.act_type = act_type
        self.num_classes = num_classes
        self.include_top = include_top
        self.with_loss = with_loss

        in_chls = [[32], [64, 128], [128, 256], [256] + [512] * 5, [512, 1024]]
        out_chls = [[64], [128, 128], [256, 256], [512] * 6, [1024, 1024]]

        self.mod1 = self._make_stage(in_chls[0], out_chls[0], True)
        self.mod2 = self._make_stage(in_chls[1], out_chls[1])
        self.mod3 = self._make_stage(in_chls[2], out_chls[2])
        self.mod4 = self._make_stage(in_chls[3], out_chls[3])
        self.mod5 = self._make_stage(in_chls[4], out_chls[4])

        self.output = nn.Sequential(
            nn.AvgPool2d(7),
            ConvModule2d(
                int(out_chls[4][-1] * alpha),
                num_classes,
                1,
                bias=self.bias,
                norm_cfg=bn_kwargs,
                act_type=None,
            ),
        )

    def _make_stage(self, in_chls, out_chls, first_layer=False):
        layers = []
        in_chls = [int(chl * self.alpha) for chl in in_chls]
        out_chls = [int(chl * self.alpha) for chl in out_chls]
        for i, in_chl, out_chl in zip(range(len(in_chls)), in_chls, out_chls):
            stride = 2 if i == 0 else 1
            if first_layer:
                layers.append(
                    ConvModule2d(
                        3,
                        in_chls[0],
                        3,
                        stride,
                        1,
                        bias=self.bias,
                        norm_cfg=self.bn_kwargs,
                        act_type=self.act_type,
                    )
                )
                stride = 1
            layers.append(
                SeparableConvModule2d(
                    in_chl,
                    out_chl,
                    3,
                    stride,
                    1,
                    bias=self.bias,
                    dw_norm_cfg=self.bn_kwargs,
                    dw_act_type=self.act_type,
                    pw_norm_cfg=self.bn_kwargs,
                    pw_act_type=self.act_type,
                )
            )
        return nn.Sequential(*layers)

    def forward(self, x):
        output = []
        for module in [self.mod1, self.mod2, self.mod3, self.mod4, self.mod5]:
            x = module(x)
            output.append(x)
        if not self.include_top:
            return output
        x = self.output(x)
        if self.with_loss:
            x = x.view(-1, self.num_classes)
        return x
