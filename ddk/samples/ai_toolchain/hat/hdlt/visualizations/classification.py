# Copyright (c) Horizon Robotics. All rights reserved.
import numpy as np
import torch
from matplotlib import pyplot as plt

from .mscoco import plot_image
from .registry import VISULIZATIONS


@VISULIZATIONS.register_module
class ClassVisualization(object):
    def __call__(self, image, target):
        if image.ndim == 4:
            image = image[0]
        image = image.permute(1, 2, 0)
        image = image.detach().cpu().numpy().astype(np.uint8)
        plot_image(image)
        print(torch.max(target[0], dim=0))
        plt.show()
        return self
