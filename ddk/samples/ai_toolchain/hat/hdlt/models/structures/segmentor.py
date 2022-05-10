# Copyright (c) Horizon Robotics. All rights reserved.

import logging

import torch.nn as nn

from hdlt.common.checkpoint import load_checkpoint
from hdlt.common.registry import build_from_cfg

from ..registry import MODELS

__all__ = ["Segmentor"]

logger = logging.getLogger(__name__)


@MODELS.register_module
class Segmentor(nn.Module):
    """
    The basic structure of segmentor.

    Args:
        backbone (torch.nn.Module): Backbone module.
        neck (torch.nn.Module): Neck module.
        head (torch.nn.Module): Head module.
        losses (torch.nn.Module): Losses module.
    """

    def __init__(
        self, backbone, neck, head, losses=None, pretrained=None, export=False
    ):
        super(Segmentor, self).__init__()
        self.backbone = None
        self.neck = None
        self.head = None
        self.losses = None
        self.export = export

        self.backbone = build_from_cfg(backbone, MODELS)
        self.neck = build_from_cfg(neck, MODELS)
        self.head = build_from_cfg(head, MODELS)

        if losses is not None:
            self.losses = build_from_cfg(losses, MODELS)
        if pretrained is not None:
            load_checkpoint(self, pretrained, strict=True, logger=logger)

    def forward(self, image, target=None):
        features = self.backbone(image)
        preds = self.neck(features)
        preds = self.head(preds)

        if self.export:
            return preds
        if not self.training or self.losses is None:
            return preds, target

        losses = self.losses(preds, target)
        return losses, preds
