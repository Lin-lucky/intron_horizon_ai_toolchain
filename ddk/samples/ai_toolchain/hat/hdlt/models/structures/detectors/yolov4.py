import logging
import torch
import torch.nn as nn

from hdlt.common.registry import build_from_cfg
from ...registry import MODELS
from .anchor_base import AnchorBase

logger = logging.getLogger(__name__)


@MODELS.register_module
class Yolov4(AnchorBase):
    def __init__(
        self,
        backbone,
        neck=None,
        head=None,
        anchor=None,
        target=None,
        post_process=None,
        loss=None,
        pretrained=None,
    ):
        super(Yolov4, self).__init__(
            backbone=backbone,
            neck=neck,
            head=head,
            anchor=anchor,
            target=target,
            loss=loss,
            post_process=post_process,
            pretrained=pretrained,
        )
