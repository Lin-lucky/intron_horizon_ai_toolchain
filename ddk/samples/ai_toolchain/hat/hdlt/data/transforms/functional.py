# Copyright (c) Horizon Robotics. All rights reserved.

import numpy as np
import torch
from PIL import Image

__all__ = ["to_tensor"]


def to_tensor(pic, to_chw=True, norm=False):
    """Convert a ``Image`` or ``numpy.array``to a tensor.

    Args:
        pic (Image or numpy.array): Image or numpy.array
            to be converted to tensor.
        to_chw (Boolen): Whether to convert to CHW.
        norm (Boolen): Whether to div 255.

    Returns:
        Tensor: Converted image.
    """

    # handle numpy array
    if isinstance(pic, np.ndarray):
        if pic.ndim == 2:
            pic = pic[:, :, None]
        if to_chw:
            pic = pic.transpose((2, 0, 1))
        img = torch.from_numpy(pic).contiguous()
        if norm:
            img = img.float().div(255.)
        return img

    if not(isinstance(pic, Image.Image)):
        raise TypeError(
            'pic should be PIL Image. Got {}'.format(type(pic))
        )

    # handle PIL Image
    img = torch.as_tensor(np.array(pic))
    img = img.view(pic.size[1], pic.size[0], len(pic.getbands()))
    if to_chw:
        # put it from HWC to CHW format
        img = img.permute((2, 0, 1))
    if norm:
        img = img.float().div(255)
    return img
