# Copyright (c) Horizon Robotics. All rights reserved.

import os
import sys

import cv2
import lmdb
import msgpack
import numpy as np
import torch
import torch.utils.data as data
import torchvision
from PIL import Image

from hdlt.data.registry import DATASETS

__all__ = ["ImageNetFromLMDB", "ImageNet2lmdb"]


@DATASETS.register_module
class ImageNetFromLMDB(data.Dataset):
    """
    ImageNetFromLMDB provides the method of reading imagenet data
    from LMDB format.

    Args:
        data_path (str): The path of LMDB file.
        transfroms (list): Transfroms of imagenet before using.
        num_samples (int): The length of keys saved in LMDB file.
    """

    def __init__(
        self,
        data_path,
        transforms=None,
        num_samples=1,
    ):
        self.root = data_path
        self.transforms = transforms
        self.num_samples = num_samples
        num = num_samples
        self.samples = [
            u'{}'.format(idx).encode('ascii') for idx in range(num)
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
            meminit=False
        )
        self.txn = env.begin(write=False)
        samples = self.txn.get(b'__keys__')
        length = self.txn.get(b'__len__')

        # self.samples = np.frombuffer(samples, dtype=np.int32)

    def __getitem__(self, index):
        raw_data = self.txn.get(self.samples[index])
        raw_data = msgpack.unpackb(raw_data, raw=False)
        img_data = raw_data[:-2]
        img = np.frombuffer(img_data, dtype=np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        sample = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

        label_data = raw_data[-2:]
        target = np.frombuffer(label_data, dtype=np.int16).astype(np.int64)
        target = target.squeeze()

        if self.transforms is not None:
            sample = self.transforms(sample)

        return sample, target

    def __len__(self):
        return len(self.samples)

    def __repr__(self):
        return "ImageNetFromLMDB"


def ImageNet2lmdb(lmdb_path, directory, split_name, num_workers):
    """
    ImageNet2lmdb is used for converting ImageNet dataset
    in torchvision to LMDB format.

    Args:
        lmdb_path (str): Path for LMDB file.
        directory (str): The dir of original imagenet data.
        split_name (str): Split name of data, such as train, val and so on.
        num_workers (int): Num workers for reading data using multiprocessing.
    """

    directory = os.path.join(directory, split_name)
    dataset = torchvision.datasets.ImageNet(
        root=directory,
        split=split_name,
        loader=loader
    )
    data_loader = data.DataLoader(dataset, num_workers=num_workers)

    db = lmdb.open(
        lmdb_path,
        map_size=1099511627776 * 2,
        meminit=False,
        map_async=True
    )
    txn = db.begin(write=True)
    for idx, datas in enumerate(data_loader):
        image, label = datas
        image = image[0]
        label = np.uint16(label).tobytes()
        txn.put(
            u'{}'.format(idx).encode('ascii'),
            msgpack.packb(image+label, use_bin_type=True)
        )

        if idx % 500 == 0:
            print("[%d/%d]" % (idx, len(data_loader)))
            txn.commit()
            txn = db.begin(write=True)

    txn.commit()
    keys = [u'{}'.format(k).encode('ascii') for k in range(idx + 1)]
    with db.begin(write=True) as txn:
        txn.put(
            b'__keys__',
            msgpack.packb(np.array(keys).tobytes(), use_bin_type=True)
        )
        txn.put(
            b'__len__',
            msgpack.packb(np.array(len(keys)).tobytes(), use_bin_type=True)
        )

    db.sync()
    db.close()


def loader(data_path):
    img = cv2.imread(data_path)
    img = cv2.imencode(".JPEG", img)[1]
    img = np.asarray(img).astype(np.uint8).tobytes()
    return img
