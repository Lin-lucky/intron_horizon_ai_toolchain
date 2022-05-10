import logging

import torch
import torch.nn as nn

from hdlt.common.registry import build_from_cfg

from ...registry import MODELS

logger = logging.getLogger(__name__)


@MODELS.register_module
class SingleStageDetector(nn.Module):
    def __init__(
        self,
        backbone,
        neck=None,
        head=None,
        loss=None,
        pretrained=None,
    ):
        super(SingleStageDetector, self).__init__()

        if backbone is not None:
            self.backbone = build_from_cfg(backbone, MODELS)
        else:
            self.backbone = None

        if neck is not None:
            self.neck = build_from_cfg(neck, MODELS)
        else:
            self.neck = None

        if head is not None:
            self.head = build_from_cfg(head, MODELS)
        else:
            self.head = None

        if loss is not None:
            self.loss = build_from_cfg(loss, MODELS)
        else:
            self.loss = None

    def extract_feat(self, img):
        """Directly extract features from the backbone+neck."""
        x = self.backbone(img)
        if self.neck is not None:
            x = self.neck(x)
        return x
