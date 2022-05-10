import numpy as np
import torch
import torch.nn as nn

from ..registry import MODELS


@MODELS.register_module
class BaseMatcher(nn.Module):
    def __init__(self):
        super(BaseMatcher, self).__init__()

    def _bbox2center(self, bboxes, image_size):

        image_size = torch.tensor(
            [image_size[1], image_size[0], image_size[1], image_size[0]],
            device=bboxes.device,
        )
        bboxes[:, :, 0:4] = bboxes[:, :, 0:4] / image_size
        cls = bboxes[:, :, 4]
        w = bboxes[:, :, 2] - bboxes[:, :, 0]
        h = bboxes[:, :, 3] - bboxes[:, :, 1]

        xc = (bboxes[:, :, 0] + bboxes[:, :, 2]) / 2
        yc = (bboxes[:, :, 1] + bboxes[:, :, 3]) / 2
        return torch.stack([xc, yc, w, h, cls], dim=2)


@MODELS.register_module
class ThresholdMatcher(BaseMatcher):
    def __init__(self, threshold=4.0):
        super(ThresholdMatcher, self).__init__()
        self.threshold = threshold

    def forward(self, features, anchors, gts, image_size):
        indexes = [list() for _ in range(len(features))]
        targets = [list() for _ in range(len(features))]
        gts = self._bbox2center(gts, image_size)
        anchors = anchors.to(features[0].device)

        batch_num = gts.shape[0]
        g = 0.5
        off = torch.tensor([[0, 0],
                        [1, 0], [0, 1], [-1, 0], [0, -1],  # j,k,l,m
                        ], device=features[0].device).float() * g

        for i in range(batch_num):
            gt = gts[i]
            index = gt[..., 4] >= 0
            gt = gt[index]
            num_gt = gt.shape[0]
            if not num_gt:
                continue
            for nl, feat, anchor in zip(range(len(features)),features, anchors):
                num_anchor = anchor.shape[0]
                scale = torch.tensor([feat.shape[2], feat.shape[1], feat.shape[2], feat.shape[1]], device=features[0].device)
                anchor_idx = torch.arange(num_anchor, device=features[0].device).float().view(num_anchor, -1).repeat(1, num_gt)
                gt_feat = torch.cat([gt.view(1, num_gt, -1).repeat(num_anchor, 1, 1), anchor_idx[:, :, None]], 2)

                scale = torch.tensor([feat.shape[2], feat.shape[1]], device=features[0].device)
                anchor = anchor * scale

                gt_feat[..., 0:2] = gt_feat[..., 0:2] * scale
                gt_feat[..., 2:4] = gt_feat[..., 2:4] * scale

                ratio = gt_feat[..., 2:4] / anchor[:, None]
                idx = torch.max(ratio, 1/ratio).max(2)[0] < self.threshold
                gt_feat = gt_feat[idx]
                if not gt_feat.shape[0]:
                    continue
                gxy = gt_feat[:, 0:2]

                gxi = scale - gxy
                j, k = ((gxy % 1. < g) & (gxy > 1.)).T
                l, m = ((gxi % 1. < g) & (gxi > 1.)).T
                j  = torch.stack((torch.ones_like(j), j, k, l, m))
                gt_feat = gt_feat.repeat((5, 1, 1))[j]
                offsets = (torch.zeros_like(gxy)[None] + off[:, None])[j]
                cls = gt_feat[..., 4].long()
                gxy = gt_feat[..., 0:2]
                gwh = gt_feat[..., 2:4]
                gij = (gxy - offsets).long()
                #print(gij)
                gi, gj = gij.T
                ai = gt_feat[..., 5].long()
                indexes[nl].append(torch.stack((torch.ones_like(ai)* i, ai, gj.clamp_(0, feat.shape[1]), gi.clamp_(0, feat.shape[2])), 1))
                targets[nl].append(torch.cat((gxy - gij, gwh, cls[:, None]), 1))

        for i in range(len(features)):
           if len(indexes[i]) == 0:
               indexes[i] = torch.tensor([], device=features[0].device)
               targets[i] = torch.tensor([], device=features[0].device)
               continue
           indexes[i] = torch.cat(indexes[i])
           targets[i] = torch.cat(targets[i])
        return indexes, targets


@MODELS.register_module
class PointsMatcher(BaseMatcher):
    def __init__(
        self,
        regress_ranges=(
            (-1, 64),
            (64, 128),
            (128, 256),
            (256, 512),
            (512, 1e16),
        ),
        center_sample_radius=0.0,
        num_classes=80,
    ):
        super(PointsMatcher, self).__init__()
        self.regress_ranges = regress_ranges
        self.center_sample_radius = center_sample_radius
        self.num_classes = num_classes

    def forward(self, features, gts, image_size):
        features = features[0]
        bbox_targets = [list() for _ in range(len(features))]
        labels = [list() for _ in range(len(features))]
        batch_num = gts.shape[0]
        INF = 1e16
        for i in range(batch_num):
            gt = gts[i]
            index = gt[..., 4] >= 0
            gt = gt[index]
            num_gt = gt.shape[0]
            for j, feat, regress_range in zip(
                range(len(features)), features, self.regress_ranges
            ):
                bbox_target = torch.zeros(
                    (feat.shape[1], feat.shape[2], 4), device=feat.device
                )
                label = (
                    torch.ones(
                        (feat.shape[1], feat.shape[2]), device=feat.device
                    )
                    * self.num_classes
                )
                if num_gt:
                    cls = gt[:, 4]

                    areas = (gt[:, 2] - gt[:, 0]) * (gt[:, 3] - gt[:, 1])
                    areas = areas.view(1, 1, -1).repeat(
                        feat.shape[1], feat.shape[2], 1
                    )

                    grid_y, grid_x = torch.meshgrid(
                        torch.arange(feat.shape[1]),
                        torch.arange(feat.shape[2]),
                    )
                    grid_xy = (
                        torch.stack([grid_x, grid_y], dim=2)
                        .float()
                        .to(device=feat.device)
                    )

                    gt_bbox = gt[:, 0:4]
                    gt_bbox = gt_bbox.view(1, 1, -1, 4).repeat(
                        feat.shape[1], feat.shape[2], 1, 1
                    )
                    grid_xy = grid_xy.view(
                        feat.shape[1], feat.shape[2], 1, 2
                    ).repeat(1, 1, num_gt, 1)

                    s = image_size[0] / feat.shape[1]

                    scale = torch.tensor([s, s, s, s], device=feat.device)

                    gt_bbox[..., 0:4] = gt_bbox[..., 0:4] / scale

                    bbox_target = bbox_target.view(
                        feat.shape[1], feat.shape[2], 1, 4
                    ).repeat(1, 1, num_gt, 1)
                    bbox_target[..., 0:2] = grid_xy - gt_bbox[..., 0:2]
                    bbox_target[..., 2:4] = gt_bbox[..., 2:4] - grid_xy

                    if self.center_sample_radius > 0.0:
                        radius = self.center_sample_radius

                        cx = (gt_bbox[..., 0] + gt_bbox[..., 2]) / 2
                        cy = (gt_bbox[..., 1] + gt_bbox[..., 3]) / 2
                        xmin = torch.max(cx - radius, gt_bbox[..., 0])
                        ymin = torch.max(cy - radius, gt_bbox[..., 1])
                        xmax = torch.min(cx + radius, gt_bbox[..., 2])
                        ymax = torch.min(cy + radius, gt_bbox[..., 3])
                        center_bbox = torch.stack(
                            [xmin, ymin, xmax, ymax], dim=-1
                        )

                        center_bbox_target = torch.zeros_like(bbox_target)
                        center_bbox_target[..., 0:2] = (
                            grid_xy - center_bbox[..., 0:2]
                        )
                        center_bbox_target[..., 2:4] = (
                            center_bbox[..., 2:4] - grid_xy
                        )
                        inside_gt_bbox_mask = center_bbox_target.min(-1)[0] > 0
                    else:
                        inside_gt_bbox_mask = bbox_target.min(-1)[0] > 0

                    max_regress_distance = bbox_target.max(-1)[0]
                    regress_range_min = regress_range[0] / s
                    regress_range_max = regress_range[1] / s

                    inside_regress_range = (
                        max_regress_distance >= regress_range_min
                    ) & (max_regress_distance <= regress_range_max)

                    areas[inside_gt_bbox_mask == 0] = INF
                    areas[inside_regress_range == 0] = INF

                    min_area, min_area_inds = areas.min(dim=2)

                    label = cls[min_area_inds]
                    label[min_area == INF] = self.num_classes

                    bbox_target = bbox_target[grid_y, grid_x, min_area_inds]
                bbox_targets[j].append(bbox_target)
                labels[j].append(label)

        for i in range(len(features)):
            labels[i] = torch.stack(labels[i])
            bbox_targets[i] = torch.stack(bbox_targets[i])
        return bbox_targets, labels
