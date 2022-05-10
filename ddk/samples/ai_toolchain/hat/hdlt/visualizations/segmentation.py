from typing import Sequence, Union

import cv2
import numpy as np
import torch
from torch import Tensor

from hdlt.visualizations.registry import VISULIZATIONS

_cityscapes_colormap = torch.tensor(
    [
        [128, 64, 128],
        [244, 35, 232],
        [70, 70, 70],
        [102, 102, 156],
        [190, 153, 153],
        [153, 153, 153],
        [250, 170, 30],
        [220, 220, 0],
        [107, 142, 35],
        [152, 251, 152],
        [70, 130, 180],
        [220, 20, 60],
        [255, 0, 0],
        [0, 0, 142],
        [0, 0, 70],
        [0, 60, 100],
        [0, 80, 100],
        [0, 0, 230],
        [119, 11, 32],
    ]
)


def _tensor_to_img_format(image: Tensor) -> Tensor:
    return image.permute(1, 2, 0)


def _onehot_to_idx(onehot: Tensor):
    if onehot.size(0) > 1:
        return torch.argmax(onehot, dim=0, keepdim=True)
    else:
        return onehot


def _colormap(gray_image: np.ndarray, colormap=cv2.COLORMAP_RAINBOW, scale=1):
    return cv2.applyColorMap(gray_image * scale, colormap)


def _tensor_colormap(gray_image, colormap=_cityscapes_colormap):
    return colormap.to(device=gray_image.device)[
        gray_image.to(dtype=torch.int64)
    ].squeeze()


def _default_image_process(image: Tensor, prefix):
    if image.ndim == 4:
        image = image.squeeze(0)
    image = _tensor_to_img_format(image)
    cv2.imshow(
        prefix,
        cv2.cvtColor(
            image.detach().cpu().numpy().astype(np.uint8), cv2.COLOR_RGB2BGR
        ),
    )


def _seg_target_process(target: Tensor, prefix):
    if target.ndim == 4:
        target = target.squeeze(0)
    target = _onehot_to_idx(target)
    target = _tensor_to_img_format(target)
    target = _tensor_colormap(target, _cityscapes_colormap)
    cv2.imshow(prefix, target.detach().cpu().numpy().astype(np.uint8))


@VISULIZATIONS.register_module
class SegVisualization(object):
    def __init__(
        self,
        data_process: dict = {
            "image": _default_image_process,
            "target": _seg_target_process,
        },
    ):
        super(SegVisualization, self).__init__()
        self.data_process = data_process

    def constructed_show(self, data, prefix, process):
        if isinstance(data, dict):
            for k, v in data.items():
                self.constructed_show(v, prefix + "_" + str(k), process)
        elif isinstance(data, Sequence):
            for i, v in enumerate(data):
                self.constructed_show(v, prefix + "_" + str(i), process)
        elif isinstance(data, Tensor):
            process(data, prefix)
        else:
            raise TypeError(
                "Visualization only accept dict/Sequence of Tensors"
            )

    def __call__(self, image: Tensor, target: Tensor):
        self.constructed_show(image, "image", self.data_process["image"])
        self.constructed_show(target, "target", self.data_process["target"])
        key = cv2.waitKey(0)
        if key == 27:
            cv2.destroyAllWindows()
            return None
        return self
