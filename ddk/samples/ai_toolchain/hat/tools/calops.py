"""Calculate ops of torch model"""
import argparse
import copy
import os

import torch

from hdlt.common import Config
from hdlt.common.profiler import profile
from hdlt.models.builder import build_model


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", "-c", type=str, required=True,
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    config = Config.fromfile(args.config)
    model_cfg = copy.deepcopy(config.model)
    model = build_model(model_cfg)

    fake_data = torch.randn([1, ] + list(config.data_shape))
    model.eval()
    total_ops, total_params = profile(model, fake_data)

    print("Params: %.6f M" % (total_params / (1000 ** 2)))
    print("FLOPs: %.6f G" % (total_ops / (1000 ** 3)))
