# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.
"""Evaluation Metrics for Semantic Segmentation"""
import threading
import numpy as np


class TFIOUMetric(object):
    def __init__(self, num_classes=19):
        import tensorflow as tf
        self.score = 0.0
        self.shape = (1, 1024, 2048)
        self.label = tf.placeholder(dtype=tf.int32, shape=self.shape)
        self.pred = tf.placeholder(dtype=tf.int32, shape=self.shape)
        self.weight = tf.placeholder(dtype=tf.int32, shape=self.shape)
        self.metric, self.metric_update = tf.metrics.mean_iou(
            self.label,
            self.pred,
            weights=self.weight,
            num_classes=num_classes,
        )
        self.metric_sess = tf.Session()
        self.metric_sess.run(tf.local_variables_initializer())

    def update(self, preds, label):
        pred = np.expand_dims(preds, axis=0)
        label = np.expand_dims(label, axis=0)
        weights = (label != -1) + 0
        label[label == -1] = 0
        pred = pred * weights

        weights = weights.astype(np.int32)
        pred = pred.astype(np.int32)
        label = label.astype(np.int32)

        feed_dict = {self.label: label, self.pred: pred, self.weight: weights}
        self.metric_sess.run(self.metric_update, feed_dict=feed_dict)
        self.score = self.metric_sess.run(self.metric)

    def get(self):
        res = self.score
        return res


class SegmentationMetric(object):
    """Computes pixAcc and mIoU metric scores
    """
    def __init__(self, nclass, with_index=True):
        self.nclass = nclass
        self.with_index = with_index
        self.lock = threading.Lock()
        self.reset()

    def update(self, labels, preds):
        """Updates the internal evaluation result.
        Parameters
        ----------
        labels : 'NDArray' or list of `NDArray`
            The labels of the data.
        preds : 'NDArray' or list of `NDArray`
            Predicted values.
        """
        def evaluate_worker(self, label, pred):
            if self.with_index is True:
                pred = pred + 1
            else:
                pred = np.argmax(output, 1).astype('int64') + 1
            label = label.astype('int64') + 1

            correct, labeled = batch_pix_accuracy(pred, label)
            inter, union = batch_intersection_union(pred, label, self.nclass)
            with self.lock:
                self.total_correct += correct
                self.total_label += labeled
                self.total_inter += inter
                self.total_union += union

        if isinstance(preds, (list, tuple)):
            threads = [
                threading.Thread(
                    target=evaluate_worker,
                    args=(self, label, pred),
                ) for (label, pred) in zip(labels, preds)
            ]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
        else:
            evaluate_worker(self, labels, preds)

    def get(self):
        """Gets the current evaluation result.
        Returns
        -------
        metrics : tuple of float
            pixAcc and mIoU
        """
        pixAcc = 1.0 * self.total_correct / (np.spacing(1) + self.total_label)
        IoU = 1.0 * self.total_inter / (np.spacing(1) + self.total_union)
        mIoU = IoU.mean()
        return pixAcc, mIoU

    def reset(self):
        """Resets the internal evaluation result to initial state."""
        self.total_inter = 0
        self.total_union = 0
        self.total_correct = 0
        self.total_label = 0


def batch_pix_accuracy(predict, target):
    """PixAcc"""
    # inputs are NDarray, output 4D, target 3D
    # the category -1 is ignored class, typically for background / boundary
    pixel_labeled = np.sum(target > 0)
    pixel_correct = np.sum((predict == target) * (target > 0))

    assert pixel_correct <= pixel_labeled, "Correct area should be smaller than Labeled"
    return pixel_correct, pixel_labeled


def batch_intersection_union(predict, target, nclass):
    """mIoU"""
    # inputs are NDarray, output 4D, target 3D
    # the category -1 is ignored class, typically for background / boundary
    mini = 1
    maxi = nclass
    nbins = nclass

    predict = predict * (target > 0).astype(predict.dtype)
    intersection = predict * (predict == target)
    # areas of intersection and union
    area_inter, _ = np.histogram(intersection, bins=nbins, range=(mini, maxi))
    area_pred, _ = np.histogram(predict, bins=nbins, range=(mini, maxi))
    area_lab, _ = np.histogram(target, bins=nbins, range=(mini, maxi))
    area_union = area_pred + area_lab - area_inter
    assert (area_inter <= area_union).all(), \
        "Intersection area should be smaller than Union area"
    return area_inter, area_union


def pixelAccuracy(imPred, imLab):
    """
    This function takes the prediction and label of a single image, returns pixel-wise accuracy
    To compute over many images do:
    for i = range(Nimages):
         (pixel_accuracy[i], pixel_correct[i], pixel_labeled[i]) = \
            pixelAccuracy(imPred[i], imLab[i])
    mean_pixel_accuracy = 1.0 * np.sum(pixel_correct) / (np.spacing(1) + np.sum(pixel_labeled))
    """
    # Remove classes from unlabeled pixels in gt image.
    # We should not penalize detections in unlabeled portions of the image.
    pixel_labeled = np.sum(imLab > 0)
    pixel_correct = np.sum((imPred == imLab) * (imLab > 0))
    pixel_accuracy = 1.0 * pixel_correct / pixel_labeled
    return (pixel_accuracy, pixel_correct, pixel_labeled)


def intersectionAndUnion(imPred, imLab, numClass):
    """
    This function takes the prediction and label of a single image,
    returns intersection and union areas for each class
    To compute over many images do:
    for i in range(Nimages):
        (area_intersection[:,i], area_union[:,i]) = intersectionAndUnion(imPred[i], imLab[i])
    IoU = 1.0 * np.sum(area_intersection, axis=1) / np.sum(np.spacing(1)+area_union, axis=1)
    """
    # Remove classes from unlabeled pixels in gt image.
    # We should not penalize detections in unlabeled portions of the image.
    imPred = imPred * (imLab > 0)

    # Compute area intersection:
    intersection = imPred * (imPred == imLab)
    (area_intersection, _) = np.histogram(intersection,
                                          bins=numClass,
                                          range=(1, numClass))

    # Compute area union:
    (area_pred, _) = np.histogram(imPred, bins=numClass, range=(1, numClass))
    (area_lab, _) = np.histogram(imLab, bins=numClass, range=(1, numClass))
    area_union = area_pred + area_lab - area_intersection
    return (area_intersection, area_union)
