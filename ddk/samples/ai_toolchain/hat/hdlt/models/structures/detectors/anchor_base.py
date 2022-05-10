import logging

import torch
import torch.nn as nn

from hdlt.common.registry import build_from_cfg

from ...registry import MODELS
from .single_stage import SingleStageDetector

logger = logging.getLogger(__name__)


@MODELS.register_module
class AnchorBase(SingleStageDetector):
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
        export=False,
    ):
        super(AnchorBase, self).__init__(
            backbone=backbone,
            neck=neck,
            head=head,
            loss=loss,
            pretrained=pretrained,
        )
        self.export = export
        if not self.export:
            if anchor is not None:
                self.anchor = build_from_cfg(anchor, MODELS)
            else:
                raise ValueError("anchor should be provides")
            if target is not None:
                self.target = build_from_cfg(target, MODELS)
            else:
                raise ValueError("target should be provides")
            if post_process is not None:
                self.post_process = build_from_cfg(post_process, MODELS)
            else:
                raise ValueError("post_process should be provides")

    def forward(self, images, labels=None):
        if self.training:
            labels = labels[0]
        n, c, h, w = images.size()
        features = self.extract_feat(images)
        preds = self.head(features)
        if self.export:
            return preds

        with torch.no_grad():
            anchors = self.anchor(image_size=(h, w), features=features)
        if self.training:
            with torch.no_grad():
                targets = self.target(
                    features=preds,
                    anchors=anchors,
                    gts=labels,
                    image_size=(h, w),
                )

            losses = self.loss(preds=preds, targets=targets, anchors=anchors)
            return losses, labels
        else:
            prediction = self.post_process(
                preds,
                image_shapes=[
                    (h, w),
                ]
                * n,
                anchors=anchors,
            )
            return prediction, labels
