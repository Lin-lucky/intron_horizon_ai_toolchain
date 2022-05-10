import torch
import torch.nn as nn

import numpy as np
from ..registry import MODELS

@MODELS.register_module
class RelativeAnchors(nn.Module):
   def __init__(self, anchors):
       super(RelativeAnchors, self).__init__()
       anchors = np.array(anchors, dtype=np.float32)
       self.anchors = torch.from_numpy(anchors)
       
   
   def forward(self, features, image_size):
       image_size = torch.from_numpy(np.array(image_size))
       return self.anchors / image_size

@MODELS.register_module
class GridAnchors(nn.Module):
   def __init__(self):
       super(GridAnchors, self).__init__()

   def _make_grid(self, nx=20, ny=20):
        yv, xv = torch.meshgrid([torch.arange(ny), torch.arange(nx)])
        return torch.stack((xv, yv), 2).float()

   def forward(self, features, image_size):
       stride = image_size[0] / features.shape[1]
       grid_xy = self._make_grid(features.shape[2], features.shape[1])
       grid_xy =  grid_xy * stride
       bbox = torch.stack([grid_xy[..., 0] - stride, grid_xy[..., 1] - stride, grid_xy[..., 0] + stride, grid_xy[..., 1] + stride], dim = -1)
       return bbox.unsqueeze(dim = 2) 
