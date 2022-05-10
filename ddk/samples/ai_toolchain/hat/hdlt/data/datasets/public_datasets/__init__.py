# Copyright (c) Horizon Robotics. All rights reserved.

# import third party for custom futures
# the register key may not found if failed
#try:
import lmdb
import msgpack

from .lmdb_cityscapes import Cityscapes2lmdb, CityscapesFromLMDB
from .lmdb_imagenet import ImageNet2lmdb, ImageNetFromLMDB
from .lmdb_mscoco import CocoDetection2lmdb, CocoFromLMDB
from .lmdb_voc import PascalVOCFromLMDB, VOCDetection2lmdb
#except Exception e:
#    pass

try:
    import mxnet as mx

    from .mxrecord_imagenet import ImageNetFromRecord
except Exception:
    pass
