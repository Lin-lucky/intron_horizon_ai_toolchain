# Copyright (c) Horizon Robotics. All rights reserved.

import os
import torch
import torch.nn as nn


def _load_checkpoint(filename, map_location="cpu"):
    if not os.path.isfile(filename):
        raise IOError('{} is not a checkpoint file'.format(filename))
    checkpoint = torch.load(filename, map_location=map_location)
    return checkpoint


def load_checkpoint(
    model,
    filename,
    map_location="cpu",
    strict=False,
    logger=None,
):
    checkpoint = _load_checkpoint(filename, map_location)

    if not isinstance(checkpoint, dict):
        raise RuntimeError(
            'No state_dict found in checkpoint file {}'.format(filename))

    if 'state_dict' in checkpoint:
        state_dict = checkpoint['state_dict']
    else:
        state_dict = checkpoint

    if list(state_dict.keys())[0].startswith('module.'):
        state_dict = {k[7:]: v for k, v in checkpoint['state_dict'].items()}

    try:
        model.load_state_dict(state_dict, strict=strict)
        logger.info("Load the pretrained model successfully")
    except Exception as e:
        logger.info("Failed to load  the pretrained model")
        logger.info(e)
    return checkpoint
