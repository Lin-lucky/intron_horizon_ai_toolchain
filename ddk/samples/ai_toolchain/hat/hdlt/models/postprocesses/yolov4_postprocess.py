import torch
import torch.nn as nn

from .detection_postprocess import DetectionPostProcess
from ..registry import MODELS


@MODELS.register_module
class Yolov4PostProcess(DetectionPostProcess):
    def __init__(
        self,
        score_threshold=0.05,
        topk_candidates=1000,
        nms_threshold=0.5,
        detections_per_img=300,
    ):
        super(Yolov4PostProcess, self).__init__(
            score_threshold=score_threshold,
            topk_candidates=topk_candidates,
            nms_threshold=nms_threshold,
            detections_per_img=detections_per_img,
        )

    def _center2bbox(self, target):
        xmin = target[..., 0] - target[..., 2] / 2
        ymin = target[..., 1] - target[..., 3] / 2
        xmax = target[..., 0] + target[..., 2] / 2
        ymax = target[..., 1] + target[..., 3] / 2
        return torch.stack([xmin, ymin, xmax, ymax], dim=1)

    def _decode(self, feat, anchor):
       
        feat_t = feat.reshape(feat.shape[0], feat.shape[1], anchor.shape[0], -1)
        grid_y, grid_x = torch.meshgrid(torch.arange(feat.shape[0]), torch.arange(feat.shape[1]))
        grid_xy = torch.stack([grid_x, grid_y], dim=2).view(feat.shape[0], feat.shape[1], 1, 2).repeat(1, 1, anchor.shape[0], 1).float().to(device=feat.device)
        obj = feat_t[..., 4].sigmoid()
        index = obj > self.score_threshold
        pxy = feat_t[..., 0:2]
        pxy = pxy.sigmoid() * 2 - 0.5
        scales = torch.tensor([feat.shape[1], feat.shape[0]], device=feat.device)
        
        pxy = (pxy + grid_xy) / scales
        pxy = pxy[index]
        pwh = feat_t[..., 2:4]
        a_t = anchor.reshape(1, 1, -1, 2).repeat(feat.shape[0], feat.shape[1], 1,  1).to(device=feat.device)
        pwh = (pwh.sigmoid() * 2) **  2 * a_t
        pwh = pwh[index]
         
        cls = feat_t[..., 5:].sigmoid()
        nc = cls.shape[-1]
        cls = cls[index]        
        obj = obj[index].unsqueeze(-1)
        nb = obj.shape[0]
 
        bboxes = torch.cat([pxy, pwh], dim=1)
        bboxes = self._center2bbox(bboxes)
        scores = (obj * cls).view(-1)
        labels = torch.arange(nc).unsqueeze(0).repeat(nb, 1).view(-1).to(device=feat.device)
        bboxes = bboxes.unsqueeze(1).repeat(1, nc, 1).view(-1, 4)
        return bboxes, scores, labels
