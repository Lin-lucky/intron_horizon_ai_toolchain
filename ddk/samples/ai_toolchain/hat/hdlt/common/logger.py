# Copyright (c) Horizon Robotics. All rights reserved.

import logging
import os
import sys


def init_logger(
    log_file,
    logger_name=None,
    rank=0,
    level=logging.INFO,
    overwrite=False,
    stream=sys.stderr,
):
    head = "%(asctime)-15s %(levelname)s Node[" + str(rank) + "] %(message)s"
    if rank != 0:
        log_file += "-rank%d" % rank
    if os.path.exists(log_file) and overwrite:
        os.remove(log_file)
    try:
        os.makedirs(os.path.dirname(log_file))
    except Exception as e:
        pass
    logger = logging.getLogger(logger_name)

    formatter = logging.Formatter(head)
    file_handler = logging.FileHandler(log_file)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    stream_handler = logging.StreamHandler(stream)
    stream_handler.setFormatter(formatter)
    logger.addHandler(stream_handler)

    logger.setLevel(level)
