# Copyright (c) Horizon Robotics. All rights reserved.

import math
from collections.abc import Sequence
from numbers import Number, Real
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision
from PIL import Image
from torch import Tensor
from torch.nn import functional as torchF
from torchvision.transforms import functional as F

from ..registry import TRANSFORMS

__all__ = [
    "LabelRemap",
    "Scale",
    "SegBgrToYuv444",
    "SegNormalize",
    "SegOneHot",
    "SegRandomAffine",
    "SegPILToTensor",
    "SegTensorToPIL",
    "CITYSCAPES_LABLE_MAPPINGS",
]

CITYSCAPES_LABLE_MAPPINGS = (
    (-1, -1, -1, -1, -1, -1, -1, 0, 1, -1, -1)
    + (2, 3, 4, -1, -1, -1, 5, -1, 6, 7, 8, 9, 10)
    + (11, 12, 13, 14, 15, -1, -1, 16, 17, 18)
)


def _to_one_hot(data: Tensor, num_classes: Optional[int] = None) -> Tensor:
    r"""
    Convert data to one hot format.
    Negative values will be converted to all zero vectors.

    Args:
        data (Tensor): Input data of shape (N, 1/None, H, W).
        num_classes (Optional[int], optional): Specify class number, pass None
            to infer clss number from input.
            Defaults to None.

    Returns:
        Tensor: One hot output.
    """
    if num_classes is None:
        num_classes = int(data.max().item()) + 1

    if data.ndim < 3:
        data = data.unsqueeze(1)

    if data.size(1) == 1:
        N, _, H, W = data.shape
        data = (data + 1).clamp(0, num_classes)
        one_hot = torch.zeros(
            (N, num_classes + 1, H, W),
            dtype=torch.int,
            device=data.device,
        )
        one_hot = one_hot.scatter_(
            1,
            data.expand_as(one_hot).to(dtype=torch.long),
            1,
        )
        one_hot = one_hot[:, 1:]
    else:
        one_hot = data

    return one_hot


class Transform(object):
    def __init__(self, key: str):
        self.key = key

    def one_ele_transform(self, data):
        raise NotImplementedError

    def __call__(self, input: Dict[str, Union[Tensor, Sequence]]):
        if isinstance(input[self.key], Sequence):
            input[self.key] = [
                self.one_ele_transform(data) for data in input[self.key]
            ]
        else:
            input[self.key] = self.one_ele_transform(input[self.key])

        return input


class TensorBasedTransform(Transform):
    r"""
    Transform implemented in Tensor but can deal with
    ONE PIL Image or batched Tensor.
    """

    def batched_worker(self, data: Tensor):
        if data.ndim < 4:
            ret = self.one_ele_transform(data.unsqueeze(0))
            if isinstance(ret, Sequence):
                return [r.squeeze(0) for r in ret]
            else:
                return ret.squeeze(0)
        else:
            return self.one_ele_transform(data)

    def image_input_worker(self, data: Union[Tensor, Image.Image]):
        if isinstance(data, Image.Image):
            data = F.pil_to_tensor(data)
            data = self.batched_worker(data)
            return F.to_pil_image(data)
        else:
            return self.batched_worker(data)

    def __call__(self, input: Dict[str, Union[Tensor, Sequence]]):
        if isinstance(input[self.key], Sequence):
            input[self.key] = [
                self.image_input_worker(data) for data in input[self.key]
            ]
        else:
            input[self.key] = self.image_input_worker(input[self.key])

        return input


@TRANSFORMS.register_module
class LabelRemap(TensorBasedTransform):
    r"""
    Remap labels.

    Args:
        mapping (Sequence): Mapping from input to output.
    """

    def __init__(self, mapping: Sequence = CITYSCAPES_LABLE_MAPPINGS):
        super(LabelRemap, self).__init__("label")
        if not isinstance(mapping, Sequence):
            raise TypeError(
                "mapping should be a sequence. Got {}".format(type(mapping))
            )
        self.mapping = torch.tensor(mapping)

    def one_ele_transform(self, label: Tensor):
        self.mapping = self.mapping.to(dtype=label.dtype, device=label.device)
        return self.mapping[label.to(dtype=torch.long)]


@TRANSFORMS.register_module
class Scale(TensorBasedTransform):
    r"""
    Scale input according to a scale list.

    Args:
        scales (Union[Real, Sequence]): The scales to apply on input.
        key (str, optional): Specify which data to manipulate of input dict.
            Defaults to "label".
    """

    def __init__(self, scales: Union[Real, Sequence], key: str = "label"):

        super(Scale, self).__init__(key)
        if isinstance(scales, Real):
            self.scales = [scales]
        elif isinstance(scales, Sequence):
            self.scales = scales
        else:
            raise TypeError(
                "scales should be number or sequence. Got {}".format(
                    type(scales)
                )
            )

    def one_ele_transform(self, data: Tensor):
        scaled_data = []
        for scale in self.scales:
            scaled_data.append(
                torchF.interpolate(
                    data.to(dtype=torch.float),
                    scale_factor=scale,
                    mode="bilinear",
                    align_corners=False,
                    recompute_scale_factor=True,
                ).to(dtype=data.dtype)
            )
        return scaled_data


@TRANSFORMS.register_module
class SegBgrToYuv444(TensorBasedTransform):
    r"""
    BgrToYuv444 is used for color format convert.

    Args:
        rgb_input (bool): The input is rgb input or not.
    """

    def __init__(self, rgb_input: bool = False):
        super(SegBgrToYuv444, self).__init__("image")
        self.rgb_input = rgb_input

    def _bgr_to_yuv444(self, images):
        device = images.device
        images = images.permute((0, 2, 3, 1)).contiguous()
        images_np = images.cpu().detach().numpy().astype(np.uint8)

        if self.rgb_input:
            images_np = images_np[:, :, :, ::-1]

        for i, image in enumerate(images_np):
            img_h, img_w = image.shape[:2]
            uv_start_idx = img_h * img_w
            v_size = int(img_h * img_w / 4)

            # bgr -> yuv420sp
            img_yuv420sp = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420)
            img_yuv420sp = img_yuv420sp.flatten()

            # yuv420sp -> yuv444
            img_y = img_yuv420sp[:uv_start_idx].reshape((img_h, img_w, 1))

            uv_end_idx = uv_start_idx + v_size
            img_u = img_yuv420sp[uv_start_idx:uv_end_idx]
            img_u = img_u.reshape(
                int(math.ceil(img_h / 2.0)), int(math.ceil(img_w / 2.0)), 1
            )
            img_u = np.repeat(img_u, 2, axis=0)
            img_u = np.repeat(img_u, 2, axis=1)

            v_start_idx = uv_start_idx + v_size
            v_end_idx = uv_start_idx + 2 * v_size
            img_v = img_yuv420sp[v_start_idx:v_end_idx]
            img_v = img_v.reshape(
                int(math.ceil(img_h / 2.0)), int(math.ceil(img_w / 2.0)), 1
            )
            img_v = np.repeat(img_v, 2, axis=0)
            img_v = np.repeat(img_v, 2, axis=1)
            img_yuv444 = np.concatenate((img_y, img_u, img_v), axis=2)

            img_yuv444 = img_yuv444.astype(np.float32)
            images[i] = torch.from_numpy(img_yuv444)
        images = images.permute((0, 3, 1, 2)).contiguous()
        return images

    def one_ele_transform(self, image: Tensor):
        return self._bgr_to_yuv444(image)


@TRANSFORMS.register_module
class SegNormalize(TensorBasedTransform):
    r"""
    Normalize a tensor image with mean and standard deviation.

    Args:
        mean (Union[float, Sequence]): Means for each channel.
        std (Union[float, Sequence]): Standard deviations for each channel.
        inplace (bool, optional): Bool to make this operation inplace.
            Defaults to False.
    """

    def __init__(
        self,
        mean: Union[float, Sequence],
        std: Union[float, Sequence],
        inplace: bool = False,
    ):
        super(SegNormalize, self).__init__("image")
        self.mean = mean
        self.std = std
        self.inplace = inplace

    def one_ele_transform(self, image: Tensor):
        return F.normalize(
            image.to(dtype=torch.float), self.mean, self.std, self.inplace
        )


@TRANSFORMS.register_module
class SegOneHot(TensorBasedTransform):
    r"""
    OneHot is used for convert layer to one-hot format.

    Args:
        num_classes (int): Num classes.
    """

    def __init__(self, num_classes: int):
        super(SegOneHot, self).__init__("label")
        self.num_classes = num_classes

    def one_ele_transform(self, label):
        return _to_one_hot(label, self.num_classes).to(dtype=torch.float)


@TRANSFORMS.register_module
class SegPILToTensor(Transform):
    r"""
    Convert PIL Image to Tensor.

    Args:
        key (str, optional): Specify which data to manipulate of input dict.
            Defaults to "image".
        to_cuda (bool, optional): Whether transfer data to current default
            cuda device. Defaults to False.
    """

    def __init__(self, key: str = "image", to_cuda: bool = False):
        super(SegPILToTensor, self).__init__(key)
        self.to_cuda = to_cuda

    def one_ele_transform(self, data: Image.Image):
        if isinstance(data, Image.Image):
            data = F.pil_to_tensor(data)
            if self.to_cuda:
                data = data.cuda()

        return data


@TRANSFORMS.register_module
class SegTensorToPIL(Transform):
    """
    Convert Tensor to PIL Image.

    Args:
        key (str, optional): Specify which data to manipulate of input dict.
            Defaults to "image".
    """

    def __init__(self, key: str = "image"):

        super(SegTensorToPIL, self).__init__(key)

    def one_ele_transform(self, data: Tensor):
        if isinstance(data, Tensor):
            data = F.to_pil_image(data)

        return data


@TRANSFORMS.register_module
class SegRandomAffine(torchvision.transforms.RandomAffine):
    """
    Apply random for both image and label.
    Please refer to :class:`~torchvision.transforms.RandomAffine` for details.

    Args:
        label_fill_value (tuple or int, optional): Fill value for label.
            Defaults to 0.
    """

    def __init__(
        self,
        degrees: Union[Sequence, float] = 0,
        translate: Tuple = None,
        scale: Tuple = None,
        shear: Union[Sequence, float] = None,
        resample: int = Image.NEAREST,
        fillcolor: Union[tuple, int] = 0,
        label_fill_value: Union[tuple, int] = 0,
    ):
        super(SegRandomAffine, self).__init__(
            degrees, translate, scale, shear, resample, fillcolor
        )

        self.label_fill_value = label_fill_value

    def __call__(self, data: Dict[str, Tensor]):
        img = data["image"]
        img_size = F._get_image_size(img)

        ret = self.get_params(
            self.degrees, self.translate, self.scale, self.shear, img_size
        )
        data["image"] = F.affine(
            img, *ret, resample=self.resample, fillcolor=self.fillcolor
        )

        if "label" in data:
            data["label"] = F.affine(
                data["label"],
                *ret,
                resample=self.resample,
                fillcolor=self.label_fill_value
            )

        return data

@TRANSFORMS.register_module
class SegResize(torchvision.transforms.Resize):
    def __init__(self, size, interpolation=Image.BILINEAR):
        super(SegResize, self).__init__(size, interpolation)

    def forward(self, data):
        data["image"] = super(SegResize, self).forward(data["image"])
        data["label"] = super(SegResize, self).forward(data["label"])

        return data
