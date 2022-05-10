import torch
import torch.nn as nn
import math

from typing import Union
from ..registry import MODELS
from hdlt.ops import ConvModule2d
from hdlt.ops import SeparableConvModule2d

@MODELS.register_module
class YoloSepHead(nn.Module):
    def __init__(
        self,
        num_classes=80,
        num_anchors=[3, 3, 3],
        in_channels=[128, 256, 512],
        stacked_convs=1,
        feat_channels=[256, 512, 1024],
        strides=[8, 16, 32],
        norm_type: Union[None, nn.Module] = nn.BatchNorm2d,
        norm_cfg: dict = dict(momentum=0.1),
        act_type: Union[None, nn.Module] = nn.ReLU,
        act_cfg: dict = dict(inplace=True),
        bias = False,
        image_size = (512, 512)
    ):
        super(YoloSepHead, self).__init__()
        self.num_classes = num_classes
        self.num_anchors = num_anchors
        self.out_convs = nn.ModuleList()
        self.image_size = image_size
        self.strides = strides

        assert len(in_channels) == len(feat_channels)
        assert len(in_channels) == len(num_anchors)
    
        for i in range(len(in_channels)):
          seq_convs = nn.Sequential()

          for j in range(stacked_convs):
              chn = in_channels[i] if j == 0 else feat_channels[i]
             
              seq_convs.add_module(
                  "stack_{}_{}".format(i, j),
                  SeparableConvModule2d(
                     in_channels=chn,
                     out_channels=feat_channels[i],
                     kernel_size=3,
                     padding=1,
                     bias=bias,
                     dw_norm_type=None,
                     dw_act_type=None,
                     pw_norm_type=norm_type,
                     pw_norm_cfg=norm_cfg,
                     pw_act_type=act_type,
                     pw_act_cfg=act_cfg
                 )
              )
          seq_convs.add_module(
              "pred_{}".format(i),
              ConvModule2d(
                  feat_channels[i],
                  num_anchors[i] * (num_classes + 5),
                  1,
                  norm_type=None,
                  act_type=None,
                  bias=True
              )
          )
          self.out_convs.append(seq_convs)

    def forward(self, features):
        preds = list()
        for conv, feat in zip(self.out_convs, features):
            pred = conv(feat)
            pred = pred.permute(0, 2, 3, 1).contiguous()
            preds.append(pred)
        return preds
