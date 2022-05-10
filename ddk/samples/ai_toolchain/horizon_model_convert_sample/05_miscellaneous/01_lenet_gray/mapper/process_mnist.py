# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import os
import struct
import shutil

import click
import numpy as np


@click.command()
@click.argument('mnist_file')
def main(mnist_file):
    if not os.path.exists(mnist_file):
        print('file not exists')
        exit(-1)
    with open(mnist_file, 'rb') as f:
        data = f.read()
    dataset = struct.unpack('%dB' % len(data[16:]), data[16:])
    dataset = np.array(dataset).reshape(10000, 28, 28)
    cal_data_dir = 'mnist_cal_data'
    if os.path.exists(cal_data_dir):
        shutil.rmtree(cal_data_dir)
    os.mkdir(cal_data_dir)
    dataset = dataset.astype(np.uint8)
    for i, data in enumerate(dataset[:100]):
        dst = os.path.join(cal_data_dir, '%d.bin' % i)
        data.tofile(dst)
        print('write file: %s' % dst)


if __name__ == '__main__':
    main()
