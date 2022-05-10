# Copyright (c) Horizon Robotics. All rights reserved.

import mxnet as mx
import mxnet.gluon as gluon
import numpy as np
import torch.utils.data as data
from PIL import Image

from hdlt.data.registry import DATASETS

__all__ = ["ImageNetFromRecord"]


@DATASETS.register_module
class ImageNetFromRecord(data.Dataset):
    """
    ImageNetFromRecord provides the method of reading imagenet data
    from MXRecord format.

    Args:
        rec_path (str): The path of mxrecord file.
        idx_path (str): The path of idx for mxrecord.
        transfroms (list): Transfroms of imagenet before using.
    """

    def __init__(
        self,
        rec_path,
        idx_path,
        transforms=None,
    ):
        self.rec_path = rec_path
        self.idx_path = idx_path
        self.transforms = transforms

        self.samples = len(open(self.idx_path, 'r').readlines())

    def __getstate__(self):
        state = self.__dict__
        state["record"] = None
        return state

    def __setstate__(self, state):
        self.__dict__ = state
        self.record = mx.recordio.MXIndexedRecordIO(
            self.idx_path, self.rec_path, 'r'
        )

    def __len__(self):
        return self.samples

    def __getitem__(self, index):
        record = self.record.read_idx(index + 1)

        header, img = mx.recordio.unpack(record)
        img = mx.image.imdecode(img).asnumpy()
        sample = Image.fromarray(img)

        label_data = header.label
        target = np.array([label_data]).astype(np.int64)
        target = target.squeeze()

        if self.transforms is not None:
            sample = self.transforms(sample)
        return sample, target

    def __repr__(self):
        return "ImageNetFromRecord"
