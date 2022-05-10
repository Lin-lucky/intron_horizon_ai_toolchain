from __future__ import absolute_import, print_function

import logging
import math
import time
from copy import deepcopy

import numpy as np
import torch
import torch.nn as nn

from ..common.utils import _as_list
from .callbacks import Callback
from .registry import CALLBACKS

logger = logging.getLogger(__name__)


@CALLBACKS.register_module
class ExponentialMovingAverage(Callback):
    def __init__(
        self,
        decay=0.9999,
        model=None,
        updates=0,
        trainer=None,
    ):
        super(ExponentialMovingAverage, self).__init__(trainer=trainer)
        self.updates = updates
        self.decay = lambda x: decay * (1 - math.exp(-x / 2000))

        if int(torch.cuda.current_device()) == 0:
            if model is None:
                model = self.trainer.model
            self.ema = deepcopy(
                model.module if self._is_parallel(model) else model
            ).eval()
            for p in self.ema.parameters():
                p.requires_grad_(False)
            self.trainer.alter_model = self.ema

    def _is_parallel(self, model):
        return type(model) in (
            nn.parallel.DataParallel,
            nn.parallel.DistributedDataParallel,
        )

    def on_batch_end(self):
        if int(torch.cuda.current_device()) == 0:
            model = self.trainer.model
            with torch.no_grad():
                self.updates += 1
                d = self.decay(self.updates)
                msd = (
                    model.module.state_dict()
                    if self._is_parallel(model)
                    else model.state_dict()
                )  # model state_dict
                for k, v in self.ema.state_dict().items():
                    if v.dtype.is_floating_point:
                        v *= d
                        v += (1.0 - d) * msd[k].detach()
