# Copyright (c) Horizon Robotics. All rights reserved.

import os

import cv2
import lmdb
import msgpack
import numpy as np
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision.datasets.vision import VisionDataset

from hdlt.data.registry import DATASETS

__all__ = ["CocoDetection2lmdb", "CocoFromLMDB"]


COCO_LABLE_TO_LABEL = {
    1: 0,
    2: 1,
    3: 2,
    4: 3,
    5: 4,
    6: 5,
    7: 6,
    8: 7,
    9: 8,
    10: 9,
    11: 10,
    13: 11,
    14: 12,
    15: 13,
    16: 14,
    17: 15,
    18: 16,
    19: 17,
    20: 18,
    21: 19,
    22: 20,
    23: 21,
    24: 22,
    25: 23,
    27: 24,
    28: 25,
    31: 26,
    32: 27,
    33: 28,
    34: 29,
    35: 30,
    36: 31,
    37: 32,
    38: 33,
    39: 34,
    40: 35,
    41: 36,
    42: 37,
    43: 38,
    44: 39,
    46: 40,
    47: 41,
    48: 42,
    49: 43,
    50: 44,
    51: 45,
    52: 46,
    53: 47,
    54: 48,
    55: 49,
    56: 50,
    57: 51,
    58: 52,
    59: 53,
    60: 54,
    61: 55,
    62: 56,
    63: 57,
    64: 58,
    65: 59,
    67: 60,
    70: 61,
    72: 62,
    73: 63,
    74: 64,
    75: 65,
    76: 66,
    77: 67,
    78: 68,
    79: 69,
    80: 70,
    81: 71,
    82: 72,
    84: 73,
    85: 74,
    86: 75,
    87: 76,
    88: 77,
    89: 78,
    90: 79,
}


def transforms(image, target):
    return np.array(image), target


class CocoDetection(VisionDataset):
    """Coco Detection Dataset

    Args:
        root (string): Root directory where images are downloaded to.
        annFile (string): Path to json annotation file.
        num_classes (int): The number of classes of coco. 80 or 91.
        transform (callable, optional): A function/transform that  takes in an
            PIL image and returns a transformed version.
            E.g, ``transforms.ToTensor``
        target_transform (callable, optional): A function/transform that takes
            in the target and transforms it.
        transforms (callable, optional): A function/transform that takes input
            sample and its target as entry and returns a transformed version.
    """

    def __init__(
        self,
        root,
        annFile,
        num_classes=80,
        transform=None,
        target_transform=None,
        transforms=None,
    ):
        self.num_classes = num_classes
        super(CocoDetection, self).__init__(
            root, transforms, transform, target_transform
        )
        from pycocotools.coco import COCO

        self.coco = COCO(annFile)
        self.ids = list(sorted(self.coco.imgs.keys()))

    def _load_image(self, id):
        path = self.coco.loadImgs(id)[0]["file_name"]
        return Image.open(os.path.join(self.root, path)).convert("RGB")

    def _load_target(self, id):
        return self.coco.loadAnns(self.coco.getAnnIds(id, iscrowd=False))

    def __getitem__(self, index):
        id = self.ids[index]
        image = self._load_image(id)
        target = self._load_target(id)
        if self.transforms is not None:
            image, target = self.transforms(image, target)
        label = np.zeros((0, 5))
        if len(target) == 0:
            print("image {} has no bounding boxes".format(id))
            return image, label, id
        for _, tar in enumerate(target):
            if tar["bbox"][2] < 1 or tar["bbox"][3] < 1:
                continue
            anno = np.zeros((1, 5))
            anno[0, :4] = tar["bbox"]
            if self.num_classes == 91:
                anno[0, 4] = tar["category_id"]  # no mapping
            else:
                anno[0, 4] = COCO_LABLE_TO_LABEL[tar["category_id"]]
            label = np.append(label, anno, axis=0)
        label[:, 2] = label[:, 0] + label[:, 2]
        label[:, 3] = label[:, 1] + label[:, 3]
        return image, label, id

    def __len__(self) -> int:
        return len(self.ids)


class Datum(object):
    """
    Args:
        image_id: str
        image: numpy.array, h*w*c
        label: numpy.array, n*5
    """

    def __init__(self, image_id=None, image=None, label=None):
        self.image_id = image_id
        self.image = image
        self.label = label

    def SerializeToString(self):
        self.image_id = np.asarray(self.image_id, dtype=np.int64).tobytes()
        self.num_bboxes = self.label.shape[0]
        num_bboxes_data = np.asarray(self.num_bboxes, dtype=np.uint8).tobytes()
        self.image = cv2.imencode(".jpg", self.image)[1].tobytes()
        self.label = np.asarray(self.label, dtype=np.float).tobytes()
        return msgpack.packb(
            self.image_id + num_bboxes_data + self.label + self.image
        )

    def ParseFromString(self, raw_data):
        image_id = raw_data[:8]
        self.image_id = np.frombuffer(image_id, dtype=np.int64)
        num_bboxes_data = raw_data[8:9]
        self.num_bboxes = np.frombuffer(num_bboxes_data, dtype=np.uint8)
        self.num_bboxes = self.num_bboxes[0]
        label_data = raw_data[9 : 9 + self.num_bboxes * 5 * 8]
        self.label = np.frombuffer(label_data, dtype=np.float)
        self.label = self.label.reshape((self.num_bboxes, 5))
        image_data = raw_data[9 + self.num_bboxes * 5 * 8 :]
        self.image = np.frombuffer(image_data, dtype=np.uint8)
        self.image = cv2.imdecode(self.image, cv2.IMREAD_COLOR)
        return self.image_id, self.image, self.label


def CocoDetection2lmdb(
    lmdb_path,
    directory,
    split_name,
    num_workers,
    num_classes=80,
    shuffle=False,
):
    """CocoDetection2lmdb is used for converting coco dataset
    to LMDB format.

    Args:
        lmdb_path (str): Path for LMDB file.
        directory (str): The dir of original coco data.
        split_name (str): Split name of data, such as train, val and so on.
        num_workers (int): The num workers for reading data
            using multiprocessing.
        num_classes (int): The num of classes produced
        shuffle (bool): whether shuffle the original coco data
    """
    if not os.path.exists(lmdb_path):
        os.mkdir(lmdb_path)
    root = os.path.join(directory, split_name)
    annFile = os.path.join(
        directory, "annotations", "instances_{}.json".format(split_name)
    )
    dataset = CocoDetection(
        root=root,
        annFile=annFile,
        num_classes=num_classes,
        transforms=transforms,
    )
    data_loader = DataLoader(dataset, num_workers=num_workers, shuffle=shuffle)
    db = lmdb.open(
        lmdb_path,
        map_size=1099511627776 * 2,
        meminit=False,
        map_async=True,
    )
    txn = db.begin(write=True)
    for idx, datas in enumerate(data_loader):
        image, label, image_id = datas
        image_id = image_id[0].numpy()
        image = image[0].numpy()
        label = label[0].numpy()
        base_data = Datum(image_id=image_id, image=image, label=label)
        txn.put(
            u"{}".format(idx).encode("ascii"),
            base_data.SerializeToString(),
        )
        if idx % 500 == 0:
            print("[%d/%d]" % (idx, len(dataset)))
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


@DATASETS.register_module
class CocoFromLMDB(Dataset):
    """CocoFromLMDB provides the method of reading coco data
    from LMDB format.

    Args:
        data_path (str): The path of LMDB file.
        transforms (list): Transfroms of data before using.
        num_samples (int): The length of keys saved in LMDB file.
    """

    def __init__(
        self,
        data_path,
        transforms=None,
        num_samples=118287,
    ):
        self.root = data_path
        self.transforms = transforms
        self.samples = [
            u"{}".format(idx).encode("ascii") for idx in range(num_samples)
        ]
        self.txn = None
        self.datum = Datum()

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

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, item):
        raw_data = self.txn.get(self.samples[item])
        raw_data = msgpack.unpackb(raw_data, raw=True)
        image_id, image, label = self.datum.ParseFromString(raw_data)
        image_id = image_id.copy()
        image = image.copy()
        label = label.copy()
        scale = np.array([1.0])
        data = {
            "image_id": image_id,
            "image": image,
            "label": label,
            "scale": scale,
        }
        if self.transforms is not None:
            data = self.transforms(data)
        return data
