# Copyright (c) Horizon Robotics. All rights reserved.

import copy
import math
import random

import cv2
import numpy as np
import torch
import torchvision

from hdlt.data.registry import TRANSFORMS
from hdlt.data.transforms import functional as cF
from hdlt.ops.bbox import compute_ious

__all__ = [
    "BgrToYuv444",
    "Normalizer",
    "ToTensor",
    "FlipLeftRight",
    "ResizerMinMax",
    "RandomCrop",
    "AugmentHSV",
    "PadwithDivisor",
    "MinIoURandomCrop",
    "RandomExpand",
    "DetResize",
    "DetColorAug",
    "Mosaic"
]

@TRANSFORMS.register_module
class RandomPerspective(object):
    def __init__(self, degrees=10, translate=.1, scale=.1, shear=10, perspective=0.0, border=(0, 0)):
        self.degrees = degrees
        self.translate = translate
        self.scale = scale
        self.shear = shear
        self.perspective = perspective
        self.border= border

    def _box_candidates(self, box1, box2, wh_thr=2, ar_thr=20, area_thr=0.2):
        w1, h1 = box1[2] - box1[0], box1[3] - box1[1]
        w2, h2 = box2[2] - box2[0], box2[3] - box2[1]
        ar = np.maximum(w2 / (h2 + 1e-16), h2 / (w2 + 1e-16))
        return (w2 > wh_thr) & (h2 > wh_thr) & (w2 * h2 / (w1 * h1 + 1e-16) > area_thr) & (ar < ar_thr)

    def __call__(self, data):
        image = data["image"]
        labels = data["label"]
        height = image.shape[0] + self.border[0] * 2 
        width = image.shape[1] + self.border[1] * 2
        # Center
        C = np.eye(3)
        C[0, 2] = -image.shape[1] / 2  # x translation (pixels)
        C[1, 2] = -image.shape[0] / 2  # y translation (pixels)

        # Perspective
        P = np.eye(3)
        P[2, 0] = random.uniform(-self.perspective, self.perspective)  # x perspective (about y)
        P[2, 1] = random.uniform(-self.perspective, self.perspective)  # y perspective (about x)

        # Rotation and Scale
        R = np.eye(3)
        a = random.uniform(-self.degrees, self.degrees)
        s = random.uniform(1.0, 1 + self.scale)
        direction = random.uniform(0, 1)
        s = s if direction > 0.5 else 1 / s
        #s = random.uniform(1 - self.scale, 1 + self.scale)
        R[:2] = cv2.getRotationMatrix2D(angle=a, center=(0, 0), scale=s)

        # Shear
        S = np.eye(3)
        S[0, 1] = math.tan(random.uniform(-self.shear, self.shear) * math.pi / 180)  # x shear (deg)
        S[1, 0] = math.tan(random.uniform(-self.shear, self.shear) * math.pi / 180)  # y shear (deg)

        # Translation
        T = np.eye(3)
        T[0, 2] = random.uniform(0.5 - self.translate, 0.5 + self.translate) * width  # x translation (pixels)
        T[1, 2] = random.uniform(0.5 - self.translate, 0.5 + self.translate) * height  # y translation (pixels)

        # Combined rotation matrix
        M = T @ S @ R @ P @ C  # order of operations (right to left) is IMPORTANT
        if (self.border[0] != 0) or (self.border[1] != 0) or (M != np.eye(3)).any():  # image changed
            if self.perspective:
                image = cv2.warpPerspective(image, M, dsize=(width, height), borderValue=(114, 114, 114))
            else:  # affine
                image = cv2.warpAffine(image, M[:2], dsize=(width, height), borderValue=(114, 114, 114))

        # Transform label coordinates
        n = len(labels)
        if n:
            xy = np.ones((n * 4, 3))
            xy[:, :2] = labels[:, [0, 1, 2, 3, 0, 3, 2, 1]].reshape(n * 4, 2)  # x1y1, x2y2, x1y2, x2y1
            xy = xy @ M.T  # transform
            if self.perspective:
                xy = (xy[:, :2] / xy[:, 2:3]).reshape(n, 8)  # rescale
            else:  # affine
                xy = xy[:, :2].reshape(n, 8)

            # create new boxes
            x = xy[:, [0, 2, 4, 6]]
            y = xy[:, [1, 3, 5, 7]]
            xy = np.concatenate((x.min(1), y.min(1), x.max(1), y.max(1))).reshape(4, n).T


            # clip boxes
            xy[:, [0, 2]] = xy[:, [0, 2]].clip(0, width)
            xy[:, [1, 3]] = xy[:, [1, 3]].clip(0, height)

            # filter candidates
            i = self._box_candidates(box1=labels[:, :4].T * s, box2=xy.T)
            labels = labels[i]
            labels[:, :4] = xy[i]
            
        data["image"] = image
        data["label"] = labels
        data["scale"] = np.asarray([s])
        return data


@TRANSFORMS.register_module
class Mosaic(object):
    def __init__(self, image_size=512,  degrees=10, translate=.1, scale=.1, shear=10, perspective=0.0):
        self.image_size = image_size
        self.random_rerspective = RandomPerspective(
                                      degrees=degrees,
                                      translate=translate,
                                      scale=scale,
                                      shear=shear,
                                      perspective=perspective,
                                      border=[-image_size // 2, -image_size // 2])

    def _fix_labels(self, labels, offsets):
        new_labels = labels.copy()
        new_labels[:, 0] = labels[:, 0]+ offsets[1]
        new_labels[:, 1] = labels[:, 1]+ offsets[0]
        new_labels[:, 2] = labels[:, 2]+ offsets[1]
        new_labels[:, 3] = labels[:, 3]+ offsets[0]
        return new_labels

    @property
    def images_needed(self):
        return 4

    def __call__(self, data_list):
        _, _, c  = data_list[0]["image"].shape
        s = self.image_size
        img4 = np.full((s * 2, s * 2, c), 114, dtype=np.uint8)
        # top left
        data1 = data_list[0].copy()
        image = data1["image"]
        oh, ow, _ = image.shape
        labels = data1["label"]

        h = min(oh, s)
        w = min(ow, s)
        img4[s-h:s, s-w:s, :] = image[oh-h:oh, ow-w:ow, :]
        new_labels = self._fix_labels(labels, ((s - oh), (s - ow)))

        #top right
        data2 = data_list[1].copy()
        image = data2["image"]
        oh, ow, _ = image.shape
        labels = data2["label"]
        h = min(oh, s)
        w = min(ow, s)

        img4[s-h:s, s:s+w, :] = image[oh-h:oh, :w, :]
        labels = self._fix_labels(labels, ((s-oh), s))
        new_labels = np.concatenate((new_labels, labels),axis=0)

        #bottom left
        data3 = data_list[2].copy()
        image = data3["image"]
        oh, ow, _ = image.shape
        labels = data3["label"]
        h = min(oh, s)
        w = min(ow, s)
        img4[s:s + h, s-w:s, :] = image[:h, ow-w:ow, :]
        labels = self._fix_labels(labels, (s, s-ow))
        new_labels = np.concatenate((new_labels, labels),axis=0)

         #bottom right
        data4 = data_list[3].copy()
        image = data4["image"]
        oh, ow, _ = image.shape
        labels = data4["label"]
        h = min(oh, s)
        w = min(ow, s)

        img4[s:s + h, s:s+w, :] = image[:h, :w, :]
        labels = self._fix_labels(labels, (s, s))
        new_labels = np.concatenate((new_labels, labels),axis=0)

        new_labels[:, 0:4] = np.clip(new_labels[:, 0:4], 0, 2 * s)
        data1["image"] = img4
        data1["label"] = new_labels
        data1 = self.random_rerspective(data1)
        return data1

@TRANSFORMS.register_module
class BgrToYuv444(object):
    """
    BgrToYuv444 is used for color format convert.

     Args:
        rgb_input (bool): Whether rgb input.

    """

    def __init__(self, rgb_input=False):
        self.rgb_input = rgb_input

    def _rgb_to_yuv444(self, image):
        image = image.astype(np.uint8)
        if self.rgb_input:
            image = image[:, :, ::-1]
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

        return img_yuv444

    def __call__(self, data):
        if isinstance(data, dict) and "image" in data.keys():
            img = data["image"]
            img = self._rgb_to_yuv444(img)
            data["image"] = img
        else:
            img = self._rgb_to_yuv444(data)
        return data


@TRANSFORMS.register_module
class Normalizer(object):
    r"""
    Normalize for det, mainly for dict input.

    Args:
        mean (float): Mean of norm.
        std (float): std of norm.
        inplace (float): whether to inplace.

    """

    def __init__(self, mean, std, inplace=False):
        self.normalize = torchvision.transforms.Normalize(
            mean=mean, std=std, inplace=inplace
        )

    def __call__(self, data):
        assert "image" in data.keys()
        img = data["image"]
        img = self.normalize(img)
        data["image"] = img
        return data


@TRANSFORMS.register_module
class ToTensor(object):
    r"""
    ToTensor for det, mainly for dict input.

    Args:
        to_chw (bool): whether to convert to chw.
        norm (bool): whether to norm output.

    """

    def __init__(self, to_chw=True, norm=False):
        self.to_chw = to_chw
        self.norm = norm

    def __call__(self, data):
        if isinstance(data, dict) and "image" in data.keys():
            img = data["image"]
            img = cF.to_tensor(img, self.to_chw, self.norm)
            data["image"] = img
        else:
            data = cF.to_tensor(data, self.to_chw, self.norm)
        return data


@TRANSFORMS.register_module
class FlipLeftRight(object):
    r"""
    Random flip left to right  for det, mainly for dict input.

    Args:
        absolute_coord (bool): Whether labels with absolute coord.
        flip_ratio (bool): The ratio to flip.

    """

    def __init__(self, absolute_coord=True, flip_ratio=0.5):
        self.flip_ratio = flip_ratio
        self.abs_c = absolute_coord

    def __call__(self, data):
        if np.random.rand() < self.flip_ratio:
            assert "image" in data.keys()
            assert "label" in data.keys()
            image = copy.deepcopy(data["image"])
            label = copy.deepcopy(data["label"])

            if self.abs_c:
                # hwc
                image = image[:, ::-1, :]
                h, w, c = image.shape
                x1 = label[:, 0].copy()
                x2 = label[:, 2].copy()
                label[:, 0] = w - x2
                label[:, 2] = w - x1
            else:
                # chw
                image = image.flip(-1)
                x1 = copy.deepcopy(label[:, 0])
                x2 = copy.deepcopy(label[:, 2])
                label[:, 0] = 1.0 - x2
                label[:, 2] = 1.0 - x1

            data["image"] = image
            data["label"] = label
        return data


@TRANSFORMS.register_module
class RandomCrop(object):
    r"""
    Random crop.

    Args:
        ratio_range (list): Scale range.
        crop_size (int): The size to crop.

    """

    def __init__(self, ratio_range=(0.5, 2.0), crop_size=512):
        self.ratio_range = ratio_range
        self.crop_size = crop_size

    def __random_scale(self, image, label):
        min_ratio, max_ratio = self.ratio_range
        assert min_ratio <= max_ratio
        scale = random.uniform(min_ratio, max_ratio)
        max_side = self.crop_size * scale

        h, w, _ = image.shape
        largest_side = max(h, w)
        scale = max_side / largest_side
        image = cv2.resize(
            image,
            (int(round(w * scale)), int(round((h * scale)))),
            interpolation=cv2.INTER_LINEAR,
        )

        label[:, :4] *= scale
        return image, label

    def __crop(self, img, labels):
        margin_h = max(img.shape[0] - self.crop_size, 0)
        margin_w = max(img.shape[1] - self.crop_size, 0)
        offset_h = np.random.randint(0, margin_h + 1)
        offset_w = np.random.randint(0, margin_w + 1)
        crop_y1, crop_y2 = offset_h, offset_h + self.crop_size
        crop_x1, crop_x2 = offset_w, offset_w + self.crop_size
        # crop the image
        crop_img = img[crop_y1:crop_y2, crop_x1:crop_x2, :]
        bbox_offset = np.array(
            [offset_w, offset_h, offset_w, offset_h], dtype=np.float32
        )
        org_labels = labels.copy()
        labels[:, :4] -= bbox_offset
        labels[:, [0, 2]] = labels[:, [0, 2]].clip(0, crop_img.shape[1])
        labels[:, [1, 3]] = labels[:, [1, 3]].clip(0, crop_img.shape[0])

        valid_ind = (labels[:, 2] > labels[:, 0]) & (
            labels[:, 3] > labels[:, 1]
        )
        labels = labels[valid_ind]
        return crop_img, labels

    def __call__(self, data):
        image, label = data["image"], data["label"]
        image, label = self.__random_scale(image, label)
        image, label = self.__crop(image, label)
        data["image"] = image
        data["label"] = label
        return data


@TRANSFORMS.register_module
class AugmentHSV(object):
    r"""
    Augment for color.

    Args:
        hgain (float): Gain for h.
        sgain (float): Gain for s.
        vgain (float): Gain for v.

    """

    def __init__(self, hgain=0.5, sgain=0.5, vgain=0.5):
        self.hgain = hgain
        self.sgain = sgain
        self.vgain = vgain

    def __call__(self, data):
        img = data["image"]
        r = (
            np.random.uniform(-1, 1, 3) * [self.hgain, self.sgain, self.vgain]
            + 1
        )  # random gains
        hue, sat, val = cv2.split(cv2.cvtColor(img, cv2.COLOR_RGB2HSV))
        dtype = img.dtype  # uint8

        x = np.arange(0, 256, dtype=np.int16)
        lut_hue = ((x * r[0]) % 180).astype(dtype)
        lut_sat = np.clip(x * r[1], 0, 255).astype(dtype)
        lut_val = np.clip(x * r[2], 0, 255).astype(dtype)

        img_hsv = cv2.merge(
            (
                cv2.LUT(hue, lut_hue),
                cv2.LUT(sat, lut_sat),
                cv2.LUT(val, lut_val),
            )
        ).astype(dtype)
        img = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2RGB)  # no return needed
        data["image"] = img
        data["scale"] = np.asarray([1.0])
        return data


@TRANSFORMS.register_module
class PadwithDivisor(object):
    r"""Padding the image to fixed shape

    Args:
        size_divisor (int):  The divisor for resize.
    """

    def __init__(self, size_divisor=512):
        self.size_divisor = size_divisor

    def __call__(self, data):
        assert "image" in data.keys()
        assert "label" in data.keys()
        image, label = data["image"], data["label"]
        h, w, c = image.shape
        h_padded = math.ceil(h / self.size_divisor) * self.size_divisor
        w_padded = math.ceil(w / self.size_divisor) * self.size_divisor

        new_image = np.zeros((h_padded, w_padded, c), dtype=np.uint8)
        new_image[:h, :w, :] = image.astype(np.uint8)
        data["image"] = new_image
        data["hw"] = np.asarray([h, w])
        return data


@TRANSFORMS.register_module
class ResizerMinMax(object):
    r"""
    Resize with fixed ratio, which has limits of short side and
    long side. For det, mainly for dict input.

    Args:
        min_side (int): The limit of short side.
        max_side (int): The limit of long side.

    """

    def __init__(self, min_side=800, max_side=1024):
        self.min_side = min_side
        self.max_side = max_side

    def __call__(self, data):
        assert "image" in data.keys()
        assert "label" in data.keys()
        image, label = data["image"], data["label"]
        h, w, c = image.shape
        smallest_side = min(h, w)
        scale = self.min_side / smallest_side
        largest_side = max(h, w)
        if largest_side * scale > self.max_side:
            scale = self.max_side / largest_side
        image = cv2.resize(
            image, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR
        )

        label[:, :4] *= scale
        data["image"] = image.astype(np.uint8)
        data["label"] = label
        data["scale"] = np.asarray([scale])
        return data


@TRANSFORMS.register_module
class Padding(object):
    r"""Padding the image to fixed shape

    Args:
        size (tuple):  The target shape for resize.
    """

    def __init__(self, size):
        self.size = size

    def __call__(self, data):
        assert "image" in data.keys()
        image = data["image"]
        h, w, c = image.shape
        assert h <= self.size[0] and w <= self.size[1]
        padding_image = np.zeros(
            (self.size[0], self.size[1], c), dtype=np.uint8
        )
        padding_image[:h, :w, :] = image.astype(np.uint8)
        data["image"] = padding_image
        data["hw"] = np.asarray([h, w])
        return data


@TRANSFORMS.register_module
class RandomExpand(object):
    r"""
    Random expand for det, mainly for dict input.

    Args:
        ratio_range (tuple): The ratio of expand.
        prob (float): The prob for expand.
        fill_value (float): The value for expand area.

    """

    def __init__(self, ratio_range=(1, 4), prob=0.5, fill_value=128.0):
        self.min_ratio, self.max_ratio = ratio_range
        self.prob = prob
        self.fill_value = fill_value

    def __call__(self, data):
        if random.uniform(0, 1) > self.prob:
            return data

        img = data["image"]
        label = copy.deepcopy(data["label"])

        c, h, w = img.shape
        ratio = random.uniform(self.min_ratio, self.max_ratio)

        expand_img = torch.empty(
            c, int(h * ratio), int(w * ratio), dtype=img.dtype
        )
        expand_img.fill_(self.fill_value)

        left = int(random.uniform(0, w * ratio - w))
        top = int(random.uniform(0, h * ratio - h))
        expand_img[:, top : top + h, left : left + w] = img
        data["image"] = expand_img

        label[:, 0] = (label[:, 0] * w + left) / int(w * ratio)
        label[:, 1] = (label[:, 1] * h + top) / int(h * ratio)
        label[:, 2] = (label[:, 2] * w + left) / int(w * ratio)
        label[:, 3] = (label[:, 3] * h + top) / int(h * ratio)
        data["label"] = label

        return data


@TRANSFORMS.register_module
class MinIoURandomCrop(object):
    r"""
    Random crop by min iou for det, mainly for dict input.

    Args:
        min_ious (tuple): The min ious for random crop.
        min_crop_size (float): The min crop size for random crop.
        bbox_clip_border (bool): Whether to clip border of bbox.

    """

    def __init__(
        self,
        min_ious=(0.1, 0.3, 0.5, 0.7, 0.9),
        min_crop_size=0.3,
        bbox_clip_border=True,
    ):
        self.min_ious = min_ious
        self.sample_mode = (1, *min_ious, 0)
        self.min_crop_size = min_crop_size
        self.bbox_clip_border = bbox_clip_border

    def __call__(self, data):
        img = data["image"]
        label = copy.deepcopy(data["label"])

        c, h, w = img.shape
        label[:, 0] *= w
        label[:, 1] *= h
        label[:, 2] *= w
        label[:, 3] *= h

        while True:
            mode = random.choice(self.sample_mode)
            self.mode = mode
            if mode == 1:
                return data

            min_iou = mode
            for i in range(50):
                new_w = random.uniform(self.min_crop_size * w, w)
                new_h = random.uniform(self.min_crop_size * h, h)

                # h / w in [0.5, 2]
                if new_h / new_w < 0.5 or new_h / new_w > 2:
                    continue

                left = random.uniform(0, w - new_w)
                top = random.uniform(0, h - new_h)

                patch = np.array(
                    [int(left), int(top), int(left + new_w), int(top + new_h)]
                )
                if patch[2] == patch[0] or patch[3] == patch[1]:
                    continue
                overlaps = compute_ious(patch.reshape(-1, 4), label[:, :4])

                if len(overlaps) > 0 and overlaps.min() < min_iou:
                    continue

                if len(overlaps) > 0:

                    def is_center_of_bboxes_in_patch(boxes, patch):
                        center = (boxes[:, :2] + boxes[:, 2:]) / 2
                        mask = (
                            (center[:, 0] > patch[0])
                            * (center[:, 1] > patch[1])
                            * (center[:, 0] < patch[2])
                            * (center[:, 1] < patch[3])
                        )
                        return mask

                    mask = is_center_of_bboxes_in_patch(label[:, :4], patch)

                    if not mask.any():
                        continue

                    label = label[mask]
                    if self.bbox_clip_border:
                        label[:, 2:4] = label[:, 2:4].clip(max=patch[2:])
                        label[:, :2] = label[:, :2].clip(min=patch[:2])
                    label[:, :4] -= np.tile(patch[:2], 2)

                img = img[:, patch[1] : patch[3], patch[0] : patch[2]]
                new_h = patch[3] - patch[1]
                new_w = patch[2] - patch[0]
                label[:, 0] /= new_w
                label[:, 1] /= new_h
                label[:, 2] /= new_w
                label[:, 3] /= new_h

                area = (label[:, 3] - label[:, 1]) * (
                    label[:, 2] - label[:, 0]
                )
                area_mask = (area > 0.001) * (area < 0.9)
                len_mask = ((label[:, 3] - label[:, 1]) > 0.025) * (
                    (label[:, 2] - label[:, 0]) > 0.025
                )
                mask = area_mask * len_mask
                label = label[mask]

                data["image"] = img
                data["label"] = label
                return data


@TRANSFORMS.register_module
class DetColorAug(torchvision.transforms.ColorJitter):
    """Randomly change the brightness, contrast, saturation and
    hue of an image. For det and dict input are the main differences
    with ColorJitter in torchvision.

    Args:
        brightness (float or tuple of float (min, max)):
            How much to jitter brightness.
        contrast (float or tuple of float (min, max)):
            How much to jitter contrast.
        saturation (float or tuple of float (min, max)):
            How much to jitter saturation.
        hue (float or tuple of float (min, max)):
            How much to jitter hue.
    """

    def __init__(
        self,
        brightness=0.5,
        contrast=(0.5, 1.5),
        saturation=(0.5, 1.5),
        hue=0.1,
    ):
        super(DetColorAug, self).__init__(
            brightness, contrast, saturation, hue
        )

    def __call__(self, data):
        assert "image" in data.keys()
        img = data["image"]
        img = super().__call__(img)
        data["image"] = img
        return data


@TRANSFORMS.register_module
class DetResize(object):
    r"""
    Resize with fixed shape for det, mainly for dict input.

    Args:
        size (tuple or int):  The target shape for resize.
        interp (int): The interp for resize.

    """

    def __init__(self, size, interp=2):
        self.size = size
        self.interp = interp

    def __call__(self, data):
        assert "image" in data.keys()
        img = data["image"]
        img = torchvision.transforms.functional.resize(
            img, self.size, self.interp
        )
        data["image"] = img
        return data
