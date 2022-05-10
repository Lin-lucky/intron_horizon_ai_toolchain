import argparse
import os

import cv2
import matplotlib
import torchvision

from hdlt.common import Config
from hdlt.data import DATALOADERS
from hdlt.data.builder import build_dataloader
from hdlt.data.datasets import public_datasets
from hdlt.visualizations.builder import build_visualization

matplotlib.use("TkAgg")


def parse_args():
    parser = argparse.ArgumentParser(description="Dataset visualization.")
    parser.add_argument(
        "--config", "-c", type=str, required=True, help="Config file path"
    )
    parser.add_argument(
        "--visual-type",
        type=str,
        required=True,
        choices=[
            "SegVisualization",
            "CocoVisualization",
            "VocVisualization",
            "ClassVisualization",
        ],
        help="The interface of torchvision.dataset",
    )
    parser.add_argument(
        "--data-dir",
        required=False,
        default=None,
        help="The directory that contains lmdb dataset.",
    )
    parser.add_argument(
        "--num-workers",
        required=False,
        default=None,
        help="The number of workers to load image.",
    )
    args = parser.parse_args()
    return args


def pop_transform(transform_list: list, type):
    for i, transform in enumerate(transform_list[:]):
        if transform["type"] == type:
            transform_list.remove(transform)

    return transform_list


IGNORED_TRANSFORMS = [
    "BgrToYuv444",
    "Normalizer",
    "ResizerMinMax",
    "SegBgrToYuv444",
    "SegNormalize",
    torchvision.transforms.Normalize,
]


if __name__ == "__main__":
    args = parse_args()
    config = Config.fromfile(args.config)
    if args.data_dir:
        config.val_dataloaders["dataset"]["data_path"] = args.data_dir
    if args.num_workers or args.num_workers == 0:
        config.val_dataloaders["num_workers"] = args.num_workers
    config.val_dataloaders["batch_size"] = 1

    for t in IGNORED_TRANSFORMS:
        if "transforms" in config.val_dataloaders["dataset"]:
            pop_transform(config.val_dataloaders["dataset"]["transforms"], t)
        if "transforms" in config.val_dataloaders:
            pop_transform(config.val_dataloaders["transforms"], t)

    dataloader = build_dataloader(config.val_dataloaders, DATALOADERS)
    visualization = build_visualization(dict(type=args.visual_type))

    for image, target in dataloader:
        if visualization:
            visualization = visualization(image, target)
        else:
            break
