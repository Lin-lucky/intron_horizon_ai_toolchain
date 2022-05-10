# Copyright (c) Horizon Robotics. All rights reserved.

from ..common.registry import build_from_cfg
from . import apex_trainer, dp_trainer, dpp_trainer, horovod_trainer
from .pl_trainers import pl_trainer
from .registry import TRAINERS


def build_trainer(cfg, default_args=None):
    if cfg is None:
        return None
    assert "type" in cfg, "type is need in trainer"
    return build_from_cfg(cfg, TRAINERS)


# different trainers need different launcher
def build_launcher(trainer):
    return LaunchMap[trainer["type"]]


LaunchMap = {
    "DistributedDataParallelTrainer": dpp_trainer.launch,
    "DataParallelTrainer": dp_trainer.launch,
    "HorovodTrainer": horovod_trainer.launch,
    "ApexDDPTrainer": apex_trainer.launch,
    "LightningTrainer": pl_trainer.launch,
}
