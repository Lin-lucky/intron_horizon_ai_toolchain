# Copyright (c) Horizon Robotics. All rights reserved.

import numpy
import torch

from .metric import EvalMetric, check_label_shapes
from .registry import METRICS

__all__ = ["Accuracy", "TopKAccuracy"]


@METRICS.register_module
class Accuracy(EvalMetric):
    """Computes accuracy classification score.

    Args:
        axis (int): The axis that represents classes
        name (str):  Name of this metric instance for display.
        output_names (List[str], None): Name of predictions that should
            be used when updating with update_dict.
            By default include all predictions.
        label_names (lLst[str], None): Name of labels that should be used
            when updating with update_dict.
            By default include all labels.
    """

    def __init__(
        self, axis=1, name="accuracy", output_names=None, label_names=None
    ):
        super(Accuracy, self).__init__(
            name, axis=axis, output_names=output_names,
            label_names=label_names, has_global_stats=True
        )
        self.axis = axis

    def update(self, labels, preds):
        labels, preds = check_label_shapes(labels, preds, True)

        for label, pred_label in zip(labels, preds):
            if pred_label.shape != label.shape:
                pred_label = torch.argmax(pred_label, -1)
            pred_label = pred_label.cpu().numpy().astype("int32")
            label = label.cpu().numpy().astype("int32")

            # flatten before checking shapes to avoid shape miss match
            label = label.flat
            pred_label = pred_label.flat

            check_label_shapes(label, pred_label)

            num_correct = (pred_label == label).sum()
            self.sum_metric += num_correct
            self.global_sum_metric += num_correct
            pred_label_num = (label >= 0).sum()
            self.num_inst += pred_label_num
            self.global_num_inst += pred_label_num


@METRICS.register_module
class TopKAccuracy(EvalMetric):
    """Computes top k predictions accuracy.

    `TopKAccuracy` differs from Accuracy in that it considers the prediction
    to be ``True`` as long as the ground truth label is in the top K
    predicated labels.

    If `top_k` = ``1``, then `TopKAccuracy` is identical to `Accuracy`.

    Args:
        top_k (int): Whether targets are in top k predictions.
        name (str):  Name of this metric instance for display.
        output_names (List[str], None): Name of predictions that should
            be used when updating with update_dict.
            By default include all predictions.
        label_names (lLst[str], None): Name of labels that should be used
            when updating with update_dict.
            By default include all labels.
    """

    def __init__(
        self, top_k, name="top_k_accuracy", output_names=None, label_names=None
    ):
        super(TopKAccuracy, self).__init__(
            name, top_k=top_k,
            output_names=output_names, label_names=label_names,
            has_global_stats=True,
        )
        self.top_k = top_k
        assert(self.top_k > 1), "Please use Accuracy if top_k=1"
        self.name += "_%d" % self.top_k

    def update(self, labels, preds):
        labels, preds = check_label_shapes(labels, preds, True)

        for label, pred_label in zip(labels, preds):
            assert(len(pred_label.shape) <= 2), \
                'Predictions should be no more than 2 dims'
            # Using argpartition here instead of argsort is safe because
            # we do not care about the order of top k elements. It is
            # much faster, which is important since that computation is
            # single-threaded due to Python GIL.
            pred_label = numpy.argpartition(
                pred_label.detach().cpu().numpy().astype('float32'),
                -self.top_k
            )
            label = label.detach().cpu().numpy().astype('int32')
            check_label_shapes(label, pred_label)
            num_samples = pred_label.shape[0]
            num_dims = len(pred_label.shape)
            if num_dims == 1:
                self.sum_metric += (pred_label.flat == label.flat).sum()
            elif num_dims == 2:
                num_classes = pred_label.shape[1]
                top_k = min(num_classes, self.top_k)
                for j in range(top_k):
                    num_correct = (
                        pred_label[:, num_classes - 1 - j].flat == label.flat
                    ).sum()
                    self.sum_metric += num_correct
                    self.global_sum_metric += num_correct
            self.num_inst += num_samples
            self.global_num_inst += num_samples
