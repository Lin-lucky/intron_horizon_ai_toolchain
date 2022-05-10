import torch
import torch.nn as nn

from ..registry import MODELS
from .detection_postprocess import DetectionPostProcess


@MODELS.register_module
class FCOSPostProcess(DetectionPostProcess):
    def __init__(
        self,
        score_threshold=0.05,
        topk_candidates=1000,
        nms_threshold=0.5,
        detections_per_img=300,
    ):
        super(FCOSPostProcess, self).__init__(
            score_threshold=score_threshold,
            topk_candidates=topk_candidates,
            nms_threshold=nms_threshold,
            detections_per_img=detections_per_img,
        )

    def _distance2bbox(self, points, distance, max_shape=None):
        x1 = points[..., 0] - distance[..., 0]
        y1 = points[..., 1] - distance[..., 1]
        x2 = points[..., 0] + distance[..., 2]
        y2 = points[..., 1] + distance[..., 3]
        if max_shape is not None:
            x1 = x1.clamp(min=0, max=max_shape[1])
            y1 = y1.clamp(min=0, max=max_shape[0])
            x2 = x2.clamp(min=0, max=max_shape[1])
            y2 = y2.clamp(min=0, max=max_shape[0])
        return torch.stack([x1, y1, x2, y2], -1)

    def _decode(self, preds):
        if len(preds) == 3:
            cls_preds, bbox_preds, centerness_preds = preds
        else:
            cls_preds, bbox_preds = preds
        nc = cls_preds.shape[2]
        grid_xy = self._make_grid(
            nx=bbox_preds.shape[1], ny=bbox_preds.shape[0]
        )
        grid_xy = grid_xy.to(device=bbox_preds.device)
        bbox_preds = self._distance2bbox(grid_xy, bbox_preds)
        cls_preds = cls_preds.sigmoid()
        if len(preds) == 3:
            centerness_preds = centerness_preds.sigmoid()
            cls_preds = torch.sqrt(cls_preds * centerness_preds)
        # print(centerness_preds.shape)
        # print(cls_preds.shape)
        scales = torch.tensor(
            [
                bbox_preds.shape[1],
                bbox_preds.shape[0],
                bbox_preds.shape[1],
                bbox_preds.shape[0],
            ],
            device=bbox_preds.device,
        )
        bbox_preds = (
            bbox_preds.view(bbox_preds.shape[0], bbox_preds.shape[1], 1, 4)
            .repeat(1, 1, nc, 1)
            .view(-1, 4)
        )

        bboxes = bbox_preds / scales
        scores = cls_preds.view(-1)
        nb = int(bbox_preds.shape[0] / nc)
        labels = (
            torch.arange(nc)
            .unsqueeze(0)
            .repeat(nb, 1)
            .view(-1)
            .to(device=bbox_preds.device)
        )
        return bboxes, scores, labels
