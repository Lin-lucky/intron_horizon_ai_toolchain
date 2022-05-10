# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os
import shutil

import numpy as np
import torch

from .callbacks import Callback
from .registry import CALLBACKS

__all__ = ["Checkpoint"]

logger = logging.getLogger(__name__)


@CALLBACKS.register_module
class Checkpoint(Callback):
    """
    Checkpoint Callback is used for saving model after training
    and resume model before training as the same times.

    Args:
        monitor (str): The objects to be supervised for saving model.
        monitor_dataset (str): The objects of unique dataset to be supervised
            for saving model while metrics is on multi datasets.
        mode (str): State of monitor for saving model.
        save_path (str): Path for saving model after training.
        resume (str): Path for resuming model before training.
        resume_model_only (bool): Whether to only resume model before training.
        strict (bool): Whether to check model while loading state dict.
        trainer (hdlt.engine.BaseTrainer): The core trainer.
    """

    def __init__(
        self,
        monitor,
        monitor_dataset=None,
        mode="auto",
        freq=1,
        save_path=None,
        resume=None,
        resume_model_only=False,
        strict=True,
        trainer=None,
    ):
        super(Checkpoint, self).__init__(trainer=trainer)
        if int(torch.cuda.current_device()) == 0:
            self.skip = False
        else:
            self.skip = True
        self.save_path = save_path
        self.resume = resume
        self.resume_model_only = resume_model_only
        self.strict = strict
        self.monitor = monitor
        self.monitor_dataset = monitor_dataset
        self.freq = freq

        if mode not in ["auto", "min", "max"]:
            logger.warning(
                "current mode %s is unknown, fallback to auto mode", mode
            )
        if mode == "min":
            self.monitor_op = np.less
            self.best = np.Inf
        elif mode == "max":
            self.monitor_op = np.greater
            self.best = -np.Inf
        else:
            if "loss" in monitor:
                self.monitor_op = np.less
                self.best = np.Inf
            else:
                self.monitor_op = np.greater
                self.best = -np.Inf

    def on_train_begin(self):
        if self.resume is not None:
            checkpoint = torch.load(
                self.resume, map_location=self.trainer.model.device
            )
            self.trainer.model.load_state_dict(
                checkpoint["state_dict"], strict=self.strict
            )
            if self.resume_model_only:
                logger.info("load model only from %s", self.resume)
                return
            self.trainer.optimizer.load_state_dict(checkpoint["optimizer"])
            self.trainer.start_epoch = checkpoint["epoch"]
            logger.info(
                "load checkpoint from %s (epoch %s)",
                self.resume,
                checkpoint["epoch"],
            )

    def on_epoch_end(self):
        if self.skip:
            return
        if (self.trainer.iter + 1) % self.freq != 0:
            return
        assert (
            self.trainer.val_metric is not None
        ), "You should call Validation before Checkpoint."

        if self.save_path is None:
            raise NameError("save path: %s is None" % (self.save_path))

        try:
            os.makedirs(self.save_path)
        except Exception as e:
            pass
        checkpoint_file = os.path.join(self.save_path, "checkpoint.pth.tar")
        best_file = os.path.join(self.save_path, "model_best.pth.tar")

        state = {
            "epoch": self.trainer.iter,
            "state_dict": self.trainer.eval_model.state_dict(),
            "optimizer": self.trainer.optimizer.state_dict(),
        }
        torch.save(state, checkpoint_file)

        if self.monitor_dataset is None:
            assert len(self.trainer.val_metric.keys()) == 1
            monitor_res = float(
                list(self.trainer.val_metric.values())[0][self.monitor]
            )
        else:
            monitor_res = self.trainer.val_metric[self.monitor_dataset][
                self.monitor
            ]
        is_best = self.monitor_op(monitor_res, self.best)
        if is_best:
            logger.info("best:%f, current:%f" % (self.best, monitor_res))
            self.best = monitor_res
            shutil.copyfile(checkpoint_file, best_file)
        logger.info("Save checkpoint to %s" % (self.save_path))
