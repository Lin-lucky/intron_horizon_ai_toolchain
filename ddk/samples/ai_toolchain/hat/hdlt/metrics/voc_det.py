# Copyright (c) Horizon Robotics. All rights reserved.

from collections import defaultdict

import numpy as np
import torch

from .metric import EvalMetric
from .registry import METRICS

__all__ = ["VOCMApMetric", "VOC07MApMetric"]


@METRICS.register_module
class VOCMApMetric(EvalMetric):
    """Calculate mean AP for object detection task

    Args:
        iou_thresh (float): IOU overlap threshold for TP
        class_names (List[str]): if provided, will print out AP for each class
    """

    def __init__(self, iou_thresh=0.5, class_names=None):
        super(VOCMApMetric, self).__init__("MeanAP")
        if class_names is None:
            self.num = None
        else:
            assert isinstance(class_names, (list, tuple))
            for name in class_names:
                assert isinstance(name, str), "must provide names as str"
            num = len(class_names)
            self.name = list(class_names) + ["mAP"]
            self.num = num + 1
        self.reset()
        self.iou_thresh = iou_thresh
        self.class_names = class_names

    def reset(self):
        """Clear the internal statistics to initial state."""
        if getattr(self, "num", None) is None:
            self.num_inst = 0
            self.sum_metric = 0.0
        else:
            self.num_inst = [0] * self.num
            self.sum_metric = [0.0] * self.num
        self._n_pos = defaultdict(int)
        self._score = defaultdict(list)
        self._match = defaultdict(list)

    def get(self):
        self._update()  # update metric at this time
        if self.num is None:
            if self.num_inst == 0:
                return (self.name, float("nan"))
            else:
                return (self.name, self.sum_metric / self.num_inst)
        else:
            names = ["%s" % (self.name[i]) for i in range(self.num)]
            values = [
                x / y if y != 0 else float("nan")
                for x, y in zip(self.sum_metric, self.num_inst)
            ]
            return (names, values)

    def update(self, targets, outputs):
        def as_numpy(a):
            """Convert a (list of) torch.Tensor into numpy.ndarray"""
            if isinstance(a, (list, tuple)):
                out = [
                    x.cpu().detach().numpy()
                    if isinstance(x, torch.Tensor)
                    else x
                    for x in a
                ]
                return out
            elif isinstance(a, torch.Tensor):
                a = a.cpu().detach().numpy()
            return a

        gt_bboxes = targets[:, :, :4]
        gt_labels = targets[:, :, 4]
        gt_difficults = targets[:, :, 5]
        pred_bboxes = [pred[:, :4] for pred in outputs]
        pred_labels = [pred[:, 4] for pred in outputs]
        pred_scores = [pred[:, 5] for pred in outputs]

        if gt_difficults is None:
            gt_difficults = [None for _ in as_numpy(gt_labels)]

        for (
            pred_bbox,
            pred_label,
            pred_score,
            gt_bbox,
            gt_label,
            gt_difficult,
        ) in zip(
            *[
                as_numpy(x)
                for x in [
                    pred_bboxes,
                    pred_labels,
                    pred_scores,
                    gt_bboxes,
                    gt_labels,
                    gt_difficults,
                ]
            ]
        ):
            # strip padding -1 for pred and gt
            valid_pred = np.where(pred_label.flat >= 0)[0]
            pred_bbox = pred_bbox[valid_pred, :]
            pred_label = pred_label.flat[valid_pred].astype(int)
            pred_score = pred_score.flat[valid_pred]
            valid_gt = np.where(gt_label.flat >= 0)[0]
            gt_bbox = gt_bbox[valid_gt, :]
            gt_label = gt_label.flat[valid_gt].astype(int)

            if gt_difficult is None:
                gt_difficult = np.zeros(gt_bbox.shape[0])
            else:
                gt_difficult = gt_difficult.flat[valid_gt]

            for l in np.unique(
                np.concatenate((pred_label, gt_label)).astype(int)
            ):
                pred_mask_l = pred_label == l
                pred_bbox_l = pred_bbox[pred_mask_l]
                pred_score_l = pred_score[pred_mask_l]
                # sort by score
                order = pred_score_l.argsort()[::-1]
                pred_bbox_l = pred_bbox_l[order]
                pred_score_l = pred_score_l[order]

                gt_mask_l = gt_label == l
                gt_bbox_l = gt_bbox[gt_mask_l]
                gt_difficult_l = gt_difficult[gt_mask_l]

                self._n_pos[l] += np.logical_not(gt_difficult_l).sum()
                self._score[l].extend(pred_score_l)

                if len(pred_bbox_l) == 0:
                    continue
                if len(gt_bbox_l) == 0:
                    self._match[l].extend((0,) * pred_bbox_l.shape[0])
                    continue

                # VOC evaluation follows integer typed bounding boxes.
                bbox_a = pred_bbox_l.copy()
                bbox_a[:, 2:] += 1
                bbox_b = gt_bbox_l.copy()
                bbox_b[:, 2:] += 1

                tl = np.maximum(bbox_a[:, None, :2], bbox_b[:, :2])
                br = np.minimum(bbox_a[:, None, 2:4], bbox_b[:, 2:4])

                area_i = np.prod(br - tl, axis=2) * (tl < br).all(axis=2)
                area_a = np.prod(bbox_a[:, 2:4] - bbox_a[:, :2], axis=1)
                area_b = np.prod(bbox_b[:, 2:4] - bbox_b[:, :2], axis=1)
                iou = area_i / (area_a[:, None] + area_b - area_i)

                gt_index = iou.argmax(axis=1)
                gt_index[iou.max(axis=1) < 0.35] = -1
                del iou

                selec = np.zeros(gt_bbox_l.shape[0], dtype=bool)
                for gt_idx in gt_index:
                    if gt_idx >= 0:
                        if gt_difficult_l[gt_idx]:
                            self._match[l].append(-1)
                        else:
                            if not selec[gt_idx]:
                                self._match[l].append(1)
                            else:
                                self._match[l].append(0)
                        selec[gt_idx] = True
                    else:
                        self._match[l].append(0)

    def _update(self):
        """ update num_inst and sum_metric """
        aps = []
        recall, precs = self._recall_prec()
        for l, rec, prec in zip(range(len(precs)), recall, precs):
            ap = self._average_precision(rec, prec)
            aps.append(ap)
            if self.num is not None and l < (self.num - 1):
                self.sum_metric[l] = ap
                self.num_inst[l] = 1
        if self.num is None:
            self.num_inst = 1
            self.sum_metric = np.nanmean(aps)
        else:
            self.num_inst[-1] = 1
            self.sum_metric[-1] = np.nanmean(aps)

    def _recall_prec(self):
        """ get recall and precision from internal records """
        n_fg_class = max(self._n_pos.keys()) + 1
        prec = [None] * n_fg_class
        rec = [None] * n_fg_class

        for l in self._n_pos.keys():
            score_l = np.array(self._score[l])
            match_l = np.array(self._match[l], dtype=np.int32)

            order = score_l.argsort()[::-1]
            match_l = match_l[order]

            tp = np.cumsum(match_l == 1)
            fp = np.cumsum(match_l == 0)

            # If an element of fp + tp is 0,
            # the corresponding element of prec[l] is nan.
            with np.errstate(divide="ignore", invalid="ignore"):
                prec[l] = tp / (fp + tp)
            # If n_pos[l] is 0, rec[l] is None.
            if self._n_pos[l] > 0:
                rec[l] = tp / self._n_pos[l]

        return rec, prec

    def _average_precision(self, rec, prec):
        """calculate average precision

        Args:
            rec (numpy.array): cumulated recall
            prec (numpy.array): cumulated precision
        Returns:
            ap as float
        """

        if rec is None or prec is None:
            return np.nan

        # append sentinel values at both ends
        mrec = np.concatenate(([0.0], rec, [1.0]))
        mpre = np.concatenate(([0.0], np.nan_to_num(prec), [0.0]))

        # compute precision integration ladder
        for i in range(mpre.size - 1, 0, -1):
            mpre[i - 1] = np.maximum(mpre[i - 1], mpre[i])

        # look for recall value changes
        i = np.where(mrec[1:] != mrec[:-1])[0]

        # sum (\delta recall) * prec
        ap = np.sum((mrec[i + 1] - mrec[i]) * mpre[i + 1])
        return ap


@METRICS.register_module
class VOC07MApMetric(VOCMApMetric):
    """ Mean average precision metric for PASCAL V0C 07 dataset

    Args:
        iou_thresh (float): IOU overlap threshold for TP
        class_names (List[str]): if provided, will print out AP for each class
    """

    def __init__(self, *args, **kwargs):
        super(VOC07MApMetric, self).__init__(*args, **kwargs)

    def _average_precision(self, rec, prec):
        """calculate average precision, override the default one,
           special 11-point metric

        Args:
            rec (numpy.array): cumulated recall
            prec (numpy.array): cumulated precision
        Returns:
            ap as float
        """

        if rec is None or prec is None:
            return np.nan
        ap = 0.0
        for t in np.arange(0.0, 1.1, 0.1):
            if np.sum(rec >= t) == 0:
                p = 0
            else:
                p = np.max(np.nan_to_num(prec)[rec >= t])
            ap += p / 11.0
        return ap
