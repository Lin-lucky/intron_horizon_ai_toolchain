# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import os
import sys
import argparse
import numpy as np
from horizon_nn.data.data_loader import ImageNetDataLoader
sys.path.append("../../../01_common/python/data/")
from transformer import *


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset",
                        type=str,
                        help='calibration data directory.',
                        required=True)
    parser.add_argument("--data_process",
                        type=str,
                        required=True,
                        help='Data process functions.')
    args = parser.parse_args()
    return args


def generate_output_path():
    os.system("rm -rf ./calibration_data_b")
    os.system("rm -rf ./calibration_data_g")
    os.system("rm -rf ./calibration_data_r")
    os.system("mkdir ./calibration_data_b")
    os.system("mkdir ./calibration_data_g")
    os.system("mkdir ./calibration_data_r")


def setup_dp(path):
    """ setup config """
    dp_file = path
    with open(dp_file) as f:
        code = compile(f.read(), dp_file, "exec")
        local = {}
        exec(code, globals(), local)
        return local["dp"]


def imagenet_loader(dataset_dir, loader):
    transformers = loader.data_transformer()
    data_loader = ImageNetDataLoader(transformers,
                                     dataset_dir,
                                     imread_mode='opencv',
                                     batch_size=100)
    return data_loader


def get_data(dataset, data_process):
    data_generator = imagenet_loader(dataset, data_process)
    data = next(data_generator)
    return data


def main():
    args = get_args()
    print("===")
    print(args.data_process)
    print(args.dataset)
    print("===")
    generate_output_path()
    data_process = setup_dp(args.data_process)
    calibration_data = get_data(args.dataset, data_process)
    image_b = calibration_data[:, 0, ...][:, np.newaxis, ...]
    for i in range(len(image_b)):
        image_b[i].astype(np.uint8).tofile(f"./calibration_data_b/{i}.b")

    image_g = calibration_data[:, 1, ...][:, np.newaxis, ...]
    for i in range(len(image_g)):
        image_g[i].astype(np.uint8).tofile(f"./calibration_data_g/{i}.g")

    image_r = calibration_data[:, 2, ...][:, np.newaxis, ...]
    for i in range(len(image_r)):
        image_r[i].astype(np.uint8).tofile(f"./calibration_data_r/{i}.r")

    print("generation done")


if __name__ == '__main__':
    main()
