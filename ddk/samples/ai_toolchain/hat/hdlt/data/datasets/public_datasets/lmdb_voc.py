# Copyright (c) Horizon Robotics. All rights reserved.

import copy
import json
import os
import sys

import cv2
import lmdb
import msgpack
import msgpack_numpy
import numpy as np
import torch
import torch.utils.data as data
import torchvision
from PIL import Image

from hdlt.data.registry import DATASETS

__all__ = ["VOCDetection2lmdb", "PascalVOCFromLMDB"]


_PASCAL_VOC_LABELS = {
    "aeroplane": (0, "Vehicle"),
    "bicycle": (1, "Vehicle"),
    "bird": (2, "Animal"),
    "boat": (3, "Vehicle"),
    "bottle": (4, "Indoor"),
    "bus": (5, "Vehicle"),
    "car": (6, "Vehicle"),
    "cat": (7, "Animal"),
    "chair": (8, "Indoor"),
    "cow": (9, "Animal"),
    "diningtable": (10, "Indoor"),
    "dog": (11, "Animal"),
    "horse": (12, "Animal"),
    "motorbike": (13, "Vehicle"),
    "person": (14, "Person"),
    "pottedplant": (15, "Indoor"),
    "sheep": (16, "Animal"),
    "sofa": (17, "Indoor"),
    "train": (18, "Vehicle"),
    "tvmonitor": (19, "Indoor"),
}


@DATASETS.register_module
class PascalVOCFromLMDB(data.Dataset):
    """
    PascalVOCFromLMDB provides the method of reading voc data
    from LMDB format.

    Args:
        data_path (str): The path of LMDB file.
        transforms (list): Transforms of voc before using.
        num_samples (int): The lenght of keys saved in LMDB file.
    """

    def __init__(
        self, data_path, shape=(1.0, 1.0), transforms=None, num_samples=1
    ):
        self.root = data_path
        self.shape = shape
        self.transforms = transforms
        self.num_samples = num_samples
        num = num_samples
        self.samples = [
            u"{}".format(idx).encode("ascii") for idx in range(num)
        ]
        self.txn = None

    def __getstate__(self):
        state = self.__dict__
        state["txn"] = None
        return state

    def __setstate__(self, state):
        self.__dict__ = state
        env = lmdb.open(
            self.root,
            readonly=True,
            lock=False,
            readahead=False,
            meminit=False,
        )
        self.txn = env.begin(write=False)
        samples = self.txn.get(b"__keys__")
        length = self.txn.get(b"__len__")

    def __getitem__(self, index):
        raw_data = self.txn.get(self.samples[index])
        raw_data = msgpack.unpackb(raw_data, object_hook=msgpack_numpy.decode)
        sample = raw_data["image"].astype(np.uint8)
        sample = torch.from_numpy(sample)
        sample = sample.permute((2, 0, 1)).contiguous()

        labels = copy.deepcopy(raw_data["label"])
        labels[:, :4] *= self.shape * 2
        data = dict(image=sample, label=labels)
        if self.transforms is not None:
            data = self.transforms(data)

        return data["image"], data["label"]

    def __len__(self):
        return len(self.samples)

    def __repr__(self):
        return "PascalVOCFromLMDB"


def VOCDetection2lmdb(lmdb_path, directory, split_name, num_workers):
    """
    VOCDetection2lmdb is used for converting PascalVOC dataset
    in torchvision to LMDB format.

    Args:
        lmdb_path (str): Path for LMDB file.
        directory (str): The dir of original voc det data.
        split_name (str): Split name of data, such as train, val and so on.
        num_workers (int): Num workers for reading data using multiprocessing.
    """

    def transforms(image, target):
        return np.array(image), target

    if split_name == "trainval":
        ds_2007 = torchvision.datasets.VOCDetection(
            root=directory,
            year="2007",
            image_set=split_name,
            transforms=transforms,
        )
        ds_2012 = torchvision.datasets.VOCDetection(
            root=directory,
            year="2012",
            image_set=split_name,
            transforms=transforms,
        )
        dataset = data.dataset.ConcatDataset([ds_2007, ds_2012])
    elif split_name == "test":
        dataset = torchvision.datasets.VOCDetection(
            root=directory,
            year="2007",
            image_set="test",
            transforms=transforms,
        )
    else:
        raise NameError(
            "split name must be trainval or test, but get %s" % (split_name)
        )

    data_loader = data.DataLoader(dataset, num_workers=num_workers)

    db = lmdb.open(
        lmdb_path, map_size=1099511627776 * 2, meminit=False, map_async=True
    )
    txn = db.begin(write=True)
    for idx, datas in enumerate(data_loader):
        image, label = datas
        image = image.numpy()[0]

        h, w, c = image.shape
        label = label["annotation"]
        labels = []
        for obj in label["object"]:
            labels.append(
                [
                    (float(obj["bndbox"]["xmin"][0]) - 1) / float(w),
                    (float(obj["bndbox"]["ymin"][0]) - 1) / float(h),
                    (float(obj["bndbox"]["xmax"][0]) - 1) / float(w),
                    (float(obj["bndbox"]["ymax"][0]) - 1) / float(h),
                    float(_PASCAL_VOC_LABELS[obj["name"][0]][0]),
                    float(obj["difficult"][0]),
                ]
            )
        labels = np.array(labels)

        txn.put(
            u"{}".format(idx).encode("ascii"),
            msgpack.packb(
                {"image": image, "label": labels}, default=msgpack_numpy.encode
            ),
        )

        if idx % 500 == 0:
            print("[%d/%d]" % (idx, len(data_loader)))
            txn.commit()
            txn = db.begin(write=True)

    txn.commit()
    keys = [u"{}".format(k).encode("ascii") for k in range(idx + 1)]
    with db.begin(write=True) as txn:
        txn.put(
            b"__keys__",
            msgpack.packb(np.array(keys).tobytes(), use_bin_type=True),
        )
        txn.put(
            b"__len__",
            msgpack.packb(np.array(len(keys)).tobytes(), use_bin_type=True),
        )

    db.sync()
    db.close()
