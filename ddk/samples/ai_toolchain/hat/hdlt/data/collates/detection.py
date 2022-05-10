# Copyright (c) Horizon Robotics. All rights reserved.

import torch
from torch.utils.data._utils.collate import default_convert

from hdlt.data.registry import COLLATES

__all__ = ["CoCoCollater"]


@COLLATES.register_module
class CoCoCollater(object):
    def __call__(self, data_list):
        assert isinstance(data_list, list) and len(data_list) > 0
        assert "image" in data_list[0].keys()
        assert "label" in data_list[0].keys()
        assert "scale" in data_list[0].keys()
        assert "image_id" in data_list[0].keys()
        assert "hw" in data_list[0].keys()

        image = [data["image"] for data in data_list]
        label = [data["label"] for data in data_list]
        scale = [data["scale"] for data in data_list]
        image_id = [data["image_id"] for data in data_list]
        hw = [data["hw"] for data in data_list]

        # Converts each NumPy array data field into a tensor
        image = default_convert(image)

        label = default_convert(label)
        scale = default_convert(scale)
        image_id = default_convert(image_id)
        hw = default_convert(hw)
        hw = [t.unsqueeze(0) for t in hw]

        scale = torch.cat(scale)
        image_id = torch.cat(image_id)
        hw = torch.cat(hw)

        heights = [int(s.shape[1]) for s in image]
        widths = [int(s.shape[2]) for s in image]
        batch_size = len(image)

        max_height = max(heights)
        max_width = max(widths)
        padded_images = torch.zeros(
            batch_size, 3, max_height, max_width, dtype=torch.float32
        )
        for i in range(batch_size):
            img = image[i]
            padded_images[i, :, : heights[i], : widths[i]] = img
        max_num_annot = max(annot.shape[0] for annot in label)
        if max_num_annot > 0:
            annot_padded = (
                torch.ones(len(label), max_num_annot, 5, dtype=torch.float32)
                * -1
            )
            for idx, annot in enumerate(label):
                if annot.shape[0] > 0:
                    annot_padded[idx, : annot.shape[0], :] = annot
        else:
            annot_padded = torch.ones(len(label), 1, 5) * -1
        return padded_images, [annot_padded, scale, image_id, hw]


@COLLATES.register_module
class VocCollater(object):
    def __call__(self, data_list):
        image = [data[0] for data in data_list]
        label = [data[1] for data in data_list]

        # Converts each NumPy array data field into a tensor
        image = default_convert(image)
        image = [img.unsqueeze(0) for img in image]
        label = default_convert(label)

        images = torch.cat(image, dim=0)
        max_num_annot = max(annot.shape[0] for annot in label)
        if max_num_annot > 0:
            annot_padded = (
                torch.ones(len(label), max_num_annot, 6, dtype=torch.float32)
                * -1
            )
            for idx, annot in enumerate(label):
                if annot.shape[0] > 0:
                    annot_padded[idx, : annot.shape[0], :] = annot
        else:
            annot_padded = torch.ones(len(label), 1, 6) * -1
        return images, annot_padded
