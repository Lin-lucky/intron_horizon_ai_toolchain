# Copyright (c) Horizon Robotics. All rights reserved.

__all__ = ["Callback"]


class Callback(object):
    """
    Create a callback use Callback.

    Args:
        trainer (hdlt.engine.BaseTrainer): The core trainer.
    """

    def __init__(self, trainer):
        self.trainer = trainer

    def on_train_begin(self):
        pass

    def on_train_end(self):
        pass

    def on_batch_begin(self):
        pass

    def on_batch_end(self):
        pass

    def on_epoch_begin(self):
        pass

    def on_epoch_end(self):
        pass
