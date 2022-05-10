import logging

import torch
import torch.nn as nn

from hdlt.common.registry import build_from_cfg

from ...registry import MODELS
from .anchor_free import AnchorFree

logger = logging.getLogger(__name__)


@MODELS.register_module
class FCOS(AnchorFree):
    def __init__(
        self,
        backbone,
        neck=None,
        head=None,
        target=None,
        post_process=None,
        loss=None,
        pretrained=None,
    ):
        super(FCOS, self).__init__(
            backbone=backbone,
            neck=neck,
            head=head,
            target=target,
            loss=loss,
            post_process=post_process,
            pretrained=pretrained,
        )
