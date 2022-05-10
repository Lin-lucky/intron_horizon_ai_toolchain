from __future__ import absolute_import, print_function

import os
import shutil
import warnings
from collections.abc import Sequence

import cv2
import lmdb
import msgpack
import numpy as np
import torch
import torch.utils.data as data
import torchvision
from PIL import Image
from torch import nn
from torchvision import transforms
from torchvision.transforms.transforms import RandomAffine

from hdlt.data.registry import DATASETS
from hdlt.data.transforms.segmentation import SegOneHot, SegPILToTensor

__all__ = ["CityscapesFromLMDB", "Cityscapes2lmdb"]


class Datum(object):
    def __init__(self, shape=None, image=None, label=None):
        self.shape = shape
        self.image = image
        self.label = label

    def SerializeToString(self):
        shape_data = np.asarray(self.shape, dtype=np.uint16).tobytes()
        image_data = cv2.imencode(".png", self.image)[1].tobytes()
        label_data = np.asarray(self.label, dtype=np.uint8).tobytes()
        return msgpack.packb(
            shape_data + image_data + label_data, use_bin_type=True
        )

    @classmethod
    def ParseFromString(cls, raw_data):
        shape_data = raw_data[: 2 * 2]
        shape = np.frombuffer(shape_data, dtype=np.uint16).astype(np.int64)
        raw_image_data = raw_data[2 * 2 :]
        total_data = np.frombuffer(raw_image_data, dtype=np.uint8)
        label_size = shape[0] * shape[1]
        image = total_data[:-label_size]
        label = total_data[-label_size:]
        image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        label = label.reshape((shape[0], shape[1]))
        return cls(shape, image, label)


@DATASETS.register_module
class CityscapesFromLMDB(data.Dataset):
    """
    CityscapesFromLMDB provides the method of reading cityscapes data
    from LMDB format.

    Args:
        data_path (str): The path of LMDB file.
        transfroms (list): Transfroms of cityscapes before using.
        num_samples (int): The length of keys saved in LMDB file.
    """

    def __init__(
        self, data_path: str, transforms: list = None, num_samples: int = 2975,
    ):
        # num_samples = {"train": 2975, "val": 500, "test": 1525}
        self.root = data_path
        self.origin_transforms = transforms
        self.samples = [
            u"{}".format(idx).encode("ascii") for idx in range(num_samples)
        ]

        self.__setstate__(self.__dict__)

    def __getstate__(self):
        state = self.__dict__
        state["txn"] = None
        state["transforms"] = None
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

        if self.origin_transforms is not None:
            self.transforms = []

            if isinstance(
                self.origin_transforms,
                torchvision.transforms.transforms.Compose,
            ):
                self.origin_transforms = self.origin_transforms.transforms
            elif not isinstance(self.origin_transforms, Sequence):
                self.origin_transforms = [self.origin_transforms]

            for transform in self.origin_transforms:
                if "torchvision" in transform.__class__.__module__:

                    def wrapper(data: dict):
                        data["image"] = transform(data["image"])
                        return data

                    self.transforms.append(wrapper)
                else:
                    self.transforms.append(transform)
        else:
            self.transforms = None

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, item):
        raw_data = self.txn.get(self.samples[item])
        raw_data = msgpack.unpackb(raw_data, raw=False)
        datum = Datum.ParseFromString(raw_data)
        image, label = (
            Image.fromarray(datum.image),
            Image.fromarray(datum.label),
        )

        data = {"image": image, "label": label}
        if self.transforms is not None:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                for transform in self.transforms:
                    data = transform(data)

        return data["image"], data["label"]


def Cityscapes2lmdb(
    lmdb_path: str, directory: str, split_name: str, num_workers: int
):
    """
    Cityscapes2lmdb is used for converting cityscapes dataset
    in torchvision to LMDB format.

    Args:
        lmdb_path (str): Path for LMDB file.
        directory (str): The dir of original cityscapes data.
        split_name (str): Split name of data, such as train, val and so on.
        num_workers (int): Num workers for reading data using multiprocessing.
    """

    def transforms(image, label):
        # put as much work in here to make it parallel
        image = np.array(image)
        label = np.array(label)
        shape = label.shape
        base_data = Datum(shape, image, label)

        return base_data.SerializeToString(), torch.empty(0)

    if os.path.exists(lmdb_path):
        shutil.rmtree(lmdb_path)
    os.makedirs(lmdb_path, exist_ok=True)

    dataset = torchvision.datasets.Cityscapes(
        root=directory,
        split=split_name,
        mode="fine",
        target_type="semantic",
        transforms=transforms,
    )
    data_loader = data.DataLoader(dataset, num_workers=num_workers)
    db = lmdb.open(
        lmdb_path, map_size=(1024 ** 3) * 15, meminit=False, map_async=True,
    )
    txn = db.begin(write=True)

    for idx, (bytes, _) in enumerate(data_loader):
        txn.put(
            u"{}".format(idx).encode("ascii"), bytes[0],
        )
        if idx % 500 == 0:
            print("[%d/%d]" % (idx, len(dataset)))
            txn.commit()
            txn = db.begin(write=True)
    idx += 1

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

    print("%d instances packed, files located in:" % idx)
    print(lmdb_path)

    return idx
