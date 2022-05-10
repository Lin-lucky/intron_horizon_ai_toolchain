"""train tools"""
import argparse
import copy
import logging
import os
import random
import time
import warnings

import numpy as np
import torch
import torch.backends.cudnn as cudnn
import torchvision

from hdlt.callbacks.builder import build_callbacks
from hdlt.common import Config, init_logger
from hdlt.engine.builder import build_launcher, build_trainer
from hdlt.models.builder import build_model


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        required=True,
        help="train config file path",
    )
    parser.add_argument(
        "--dist-url",
        type=str,
        default="tcp://localhost:8000",
        help="dist url for init process",
    )
    parser.add_argument(
        "--launcher",
        type=str,
        choices=["torch", "mpi"],
        default="",
        help="job launcher for multi machines",
    )
    return parser.parse_args()


def main(gpu, args):
    cfg = Config.fromfile(args.config)
    solver = cfg.workflow

    time_stamp = time.strftime(
        "%Y%m%d%H%M%S", time.localtime(int(time.time()))
    )
    file_name = cfg.file_name + time_stamp
    log_file = os.path.join(".hdlt_logs/", file_name)
    init_logger(log_file=log_file, rank=gpu)
    logger = logging.getLogger()
    logger.info(cfg)
    step = "float"
    logger.info("=" * 50 + "BEGIN %s STAGE" % step.upper() + "=" * 50)

    cudnn.benchmark = cfg.cudnn_benchmark
    if cfg.seed is not None:
        random.seed(cfg.seed)
        np.random.seed(cfg.seed)
        torch.manual_seed(cfg.seed)
        cudnn.deterministic = True
        cudnn.benchmark = False
        warnings.warn(
            "You have chosen to seed training. "
            "This will turn on the CUDNN deterministic setting, "
            "which can slow down your training considerably! "
            "You may see unexpected behavior when restarting "
            "from checkpoints."
        )

    # assign GPUs
    model = build_model(cfg.model)

    trainer = build_trainer(
        dict(
            gpus=gpu,
            model=model,
            train_dataloader=cfg.train_dataloader,
            **solver.trainer,
        )
    )
    if "callbacks" in solver:
        callbacks = build_callbacks(solver.callbacks, dict(trainer=trainer))
        trainer.set_callbacks(callbacks)
    trainer.fit()

    logger.info("=" * 50 + "END %s STAGE" % step.upper() + "=" * 50)


if __name__ == "__main__":
    args = parse_args()
    config = Config.fromfile(args.config)

    os.environ["CUDA_VISIBLE_DEVICES"] = config.context

    trainer = config.workflow["trainer"]
    launch = build_launcher(trainer)
    launch(main, config.num_gpus, dist_url=args.dist_url, args=(args,))
