import numpy as np
import torch
import torch.nn as nn
from torch.nn.functional import pad
from torchvision.ops import boxes as box_ops

__all__ = ["DetectionPostProcess"]


class DetectionPostProcess(nn.Module):
    def __init__(
        self,
        score_threshold=0.05,
        topk_candidates=1000,
        nms_threshold=0.5,
        detections_per_img=300,
    ):
        super(DetectionPostProcess, self).__init__()
        self.score_threshold = score_threshold
        self.topk_candidates = topk_candidates
        self.nms_threshold = nms_threshold
        self.detections_per_img = detections_per_img

    def _clip_coords(self, boxes, img_shape):
        # Clip bounding xyxy bounding boxes to image shape (height, width)
        boxes[:, 0].clamp_(0, img_shape[1])  # x1
        boxes[:, 1].clamp_(0, img_shape[0])  # y1
        boxes[:, 2].clamp_(0, img_shape[1])  # x2
        boxes[:, 3].clamp_(0, img_shape[0])  # y2

    def _make_grid(self, nx=20, ny=20):
        yv, xv = torch.meshgrid([torch.arange(ny), torch.arange(nx)])
        return torch.stack((xv, yv), 2).float()

    def forward(self, feats, image_shapes, anchors=None):
        if isinstance(feats[0], list):
            level_num = len(feats[0])
            batch_size = feats[0][0].shape[0]
        else:
            level_num = len(feats)
            batch_size = feats[0].shape[0]

        if anchors is not None:
            assert len(anchors) == level_num, "Mismatched input numbers."
        assert (
            len(image_shapes) == batch_size
        ), "Mismatched image shapes and batch size."

        boxes_list = list()
        scores_list = list()
        labels_list = list()
        for i in range(batch_size):
            bboxes = []
            scores = []
            labels = []

            for j in range(level_num):
                if isinstance(feats[0], list):
                    preds = [pred[j][i] for pred in feats]
                else:
                    preds = feats[j][i]
                if anchors is not None:
                    anchor = anchors[j]
                    bbox, score, label = self._decode(preds, anchor)
                else:
                    bbox, score, label = self._decode(preds)
                keep_idxs = score > self.score_threshold
                score = score[keep_idxs]
                topk_idxs = torch.where(keep_idxs)[0]

                # keep only topk scoring predictions
                num_topk = min(self.topk_candidates, topk_idxs.shape[0])
                score, idxs = score.topk(num_topk)
                topk_idxs = topk_idxs[idxs]
                image_shape = image_shapes[i]
                scales = torch.tensor(
                    [
                        image_shape[1],
                        image_shape[0],
                        image_shape[1],
                        image_shape[0],
                    ]
                ).to(device=bbox.device)
                bbox = bbox[topk_idxs] * scales
                label = label[topk_idxs]

                self._clip_coords(bbox, image_shape)
                bboxes.append(bbox)
                scores.append(score)
                labels.append(label)

            bboxes = torch.cat(bboxes, dim=0)
            scores = torch.cat(scores, dim=0)
            labels = torch.cat(labels, dim=0)
            # non-maximum suppression
            keep = box_ops.batched_nms(
                bboxes, scores, labels, self.nms_threshold
            )
            keep = keep[: self.detections_per_img]

            boxes_list.append(bboxes[keep])
            scores_list.append(scores[keep])
            labels_list.append(labels[keep])

        predictions = [
            torch.cat(
                [boxes, scores.unsqueeze(-1), labels.unsqueeze(-1)], dim=-1
            )
            for boxes, scores, labels in zip(
                boxes_list, scores_list, labels_list
            )
        ]
        return predictions
