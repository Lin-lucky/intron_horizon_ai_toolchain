"""Build LMDB from torchvision dataset"""

import argparse
import os

import torchvision

import hdlt.data.datasets.public_datasets as public_datasets


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pack LMDB from torchvision dataset."
    )
    parser.add_argument(
        "--data-type",
        required=True,
        help="The interface of torchvision.dataset",
    )
    parser.add_argument(
        "--data-dir",
        required=True,
        help="The directory that contains unpacked image files.",
    )
    parser.add_argument(
        "--split-name", default="train", help="The split to pack."
    )
    parser.add_argument(
        "--num-workers",
        default=10,
        help="The number of workers to load image.",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = parse_args()
    directory = os.path.expanduser(args.data_dir)
    print("Loading dataset from %s" % directory)
    lmdb_path = os.path.join(args.data_dir, "%s_lmdb" % args.split_name)
    assert callable(
        getattr(torchvision.datasets, args.data_type)
    ), "Cannot find %s in torchvision.dataset." % (args.data_type)
    x2lmdb = getattr(public_datasets, "%s2lmdb" % (args.data_type))
    x2lmdb(lmdb_path, directory, args.split_name, int(args.num_workers))
    print("Finish dataset building.")
