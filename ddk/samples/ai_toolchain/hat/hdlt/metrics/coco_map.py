# Copyright (c) Horizon Robotics. All rights reserved.

import io
import json
import os
import sys
import warnings

import numpy as np
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval

from hdlt.metrics.registry import METRICS

__all__ = ["CocoMetrics"]


@METRICS.register_module
class CocoMetrics(object):
    def __init__(
        self,
        anno_file,
        score_thresh=0.0,
        num_classes=80,
    ):
        save_filename = "tmp_file.json"
        anno_file = os.path.abspath(os.path.expanduser(anno_file))
        self.score_thresh = score_thresh
        self.num_classes = num_classes
        self._coco_anno = COCO(anno_file)
        self._contiguous_id_to_json = {}
        self._contiguous_id_to_name = {}
        class_cat = self._coco_anno.dataset["categories"]
        class_names = [cat["name"] for cat in class_cat]
        name2jsonID = {}
        for (i, cat) in enumerate(class_cat):
            name2jsonID[cat["name"]] = cat["id"]
        self._with_bg = False
        for (i, name) in enumerate(class_names):
            name = name.split("|")[-1]
            if name not in ["background", "__background__"]:
                assert name in name2jsonID
                self._contiguous_id_to_json[i] = name2jsonID[name]
                self._contiguous_id_to_name[i] = name
            else:
                self._with_bg = True
        self._filename = os.path.abspath(os.path.expanduser(save_filename))
        if os.path.exists(self._filename):
            os.remove(self._filename)
        self._results = {}
        self.IoU_lo_thresh = 0.5
        self.IoU_hi_thresh = 0.95

    def reset(self):
        self._results = {}

    def update_coco(self, pred_result, image_id):
        assert isinstance(pred_result, list)
        image_id = int(image_id)
        for pred in pred_result:
            assert isinstance(pred, dict)
            assert "bbox" in pred, "missing bbox for prediction"
        image_name = "%012d" % image_id + ".jpg"
        if image_name in self._results:
            warnings.warn("warning: you are overwriting {}".format(image_name))
        inst_list = []
        for pred in pred_result:
            coco_inst = {}
            bbox = pred["bbox"]
            bbox = bbox.reshape((-1,))
            assert bbox.shape == (6,), (
                "bbox should with shape (6,), get %s" % bbox.shape
            )
            coco_inst.update(
                {
                    "image_id": image_id,
                    "category_id": int(bbox[5])
                    if self.num_classes == 91
                    else self._contiguous_id_to_json[int(bbox[5])],
                    "score": float(bbox[4]),
                    "bbox": [
                        float(bbox[0]),
                        float(bbox[1]),
                        float(bbox[2] - bbox[0]),
                        float(bbox[3] - bbox[1]),
                    ],
                }
            )
            inst_list.append(coco_inst)
        self._results[image_name] = inst_list

    def update(self, labels, preds):
        gt_bboxes, scales, image_ids, hw = labels
        preds = [
            pred.detach().cpu().numpy().astype("float32") for pred in preds
        ]
        scales = scales.detach().cpu().numpy().astype("float32")
        image_ids = image_ids.detach().cpu().numpy().astype("int32")
        N = len(preds)
        for i in range(N):
            pred = preds[i]
            scale = scales[i]
            image_id = image_ids[i]
            valid_box = pred[..., 4] > self.score_thresh
            pred = pred[valid_box, :]
            pred[:, 0:4] = pred[:, 0:4] / scale
            num_bbox = pred.shape[0]
            pred_results = list()
            for j in range(num_bbox):
                pred_results.append({"bbox": pred[j]})
            self.update_coco(pred_results, image_id)

    def _dump_json(self):
        recorded_size = len(self._results)
        anno_size = len(self._coco_anno.getImgIds())
        if not anno_size == recorded_size:
            warnings.warn(
                "Recorded {} out of {} validation images, "
                "incompelete results".format(recorded_size, anno_size)
            )
        try:
            ret = []
            for (_, v) in self._results.items():
                ret.extend(v)
            with open(self._filename, "w") as f:
                json.dump(ret, f)
        except IOError as e:
            raise RuntimeError(
                "Unable to dump json file, ignored. What(): {}".format(str(e))
            )

    def get(self):
        self._dump_json()
        try:
            bbox_names, bbox_values = self._update("bbox")
        except Exception:
            bbox_names = [
                "~~~~ MeanAP @ IoU=[{:.2f},{:.2f} for {} ~~~~".format(
                    self.IoU_lo_thresh, self.IoU_hi_thresh, "bbox"
                )
            ]
            bbox_values = ["0.0"]
            return (bbox_names, bbox_values)
        return (bbox_names, bbox_values)

    def _update(self, anno_type):
        def _get_thr_ind(coco_eval, thr):
            ind = np.where(
                (coco_eval.params.iouThrs > thr - 1e-5)
                & (coco_eval.params.iouThrs < thr + 1e-5)
            )[0][0]
            iou_thr = coco_eval.params.iouThrs[ind]
            assert np.isclose(iou_thr, thr)
            return ind

        pred = self._coco_anno.loadRes(self._filename)
        coco_eval = COCOeval(self._coco_anno, pred, anno_type)
        coco_eval.evaluate()
        coco_eval.accumulate()
        ind_lo = _get_thr_ind(coco_eval, self.IoU_lo_thresh)
        ind_hi = _get_thr_ind(coco_eval, self.IoU_hi_thresh)
        precision = coco_eval.eval["precision"][
            ind_lo : (ind_hi + 1), :, :, 0, 2
        ]
        ap_default = np.mean(precision[precision > -1])
        names, values = ([], [])
        names.append("~~~~ Summary {} metrics ~~~~".format(anno_type))
        # catch coco print string, don't want directly print here
        _stdout = sys.stdout
        sys.stdout = io.StringIO()
        coco_eval.summarize()
        coco_summary = sys.stdout.getvalue()
        sys.stdout = _stdout
        values.append(str(coco_summary).strip())
        # collect MAP for each class
        for (cls_ind, cls_name) in self._contiguous_id_to_name.items():
            precision = coco_eval.eval["precision"][
                ind_lo : (ind_hi + 1), :, cls_ind - int(self._with_bg), 0, 2
            ]
            ap = np.mean(precision[precision > -1])
            names.append(cls_name)
            values.append("{:.1f}".format(100 * ap))
        # put mean AP at last, for comparing perf
        names.append(
            "~~~~ MeanAP @ IoU=[{:.2f},{:.2f} for {} ~~~~".format(
                self.IoU_lo_thresh, self.IoU_hi_thresh, anno_type
            )
        )
        values.append("{:.1f}".format(100 * ap_default))
        return (names, values)
