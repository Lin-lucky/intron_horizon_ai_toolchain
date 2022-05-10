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
        "--export-name",
        type=str,
        default="fcos-eff-bifpn",
        help="export onnx name",
    )

    return parser.parse_args()


def set_export(model):
    if hasattr(model, "export"):
        model.export = True
    for child in model.children():
        set_export(child)


def main(gpu, args):
    cfg = Config.fromfile(args.config)
    model = build_model(cfg.model)
    model.eval()

    data_shape = (1,) + cfg.data_shape
    export_name = args.export_name + ".onnx"
    set_export(model)
    solver = cfg["validation"]
    if solver["pretrained_model"] is not None:
        checkpoint = torch.load(solver["pretrained_model"], map_location="cpu")
        param_name = list(checkpoint["state_dict"].keys())[0]
        if param_name.startswith("module"):
            model = torch.nn.DataParallel(model)

        model.load_state_dict(checkpoint["state_dict"])
        if param_name.startswith("module"):
            model = model.module
    dummy_in = torch.randn(data_shape)
    torch.onnx.export(
        model, dummy_in, export_name, input_names=["data"], opset_version=11
    )


if __name__ == "__main__":
    args = parse_args()
    main([0], args)
