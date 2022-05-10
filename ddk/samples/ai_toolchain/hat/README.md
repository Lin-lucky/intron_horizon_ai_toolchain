# Horizon Deep Learning Toolkit

## Introduction
This project includes implementations of the state-of-the-art (SOTA) deep learning models including Image Classification, Objdect Detection, Semantic Segmentation tasks for the horizon BPU.


## Features
1. This project is based on the public pytorch 1.7.0.
2. All examples are compatible with Horizon BPU(X5/J5).


## How to build
1. hdlt only support Python 3.5 or later.
2. cuda cuda-9.2 is installed, and make sure `nvcc --version` print the correct version.
3. pytorch-1.7.0 and torchvision-0.8.1 is installed, and make sure
`python3 -c "import pytorch; print(pytorch.__version__)"` print the correct version.
4. copy hdlt python files into `PYTHONPATH`.


## How to run
Horizon Deep Learning Toolkit provides unified function module in tools, all workflows in configs. Therefore, tools + configs is general method of execution. for example, after preparing data and some pretrained models, scripts like this:
## For Fcos:
1. Prepare data
`python3 tools/vision2lmdb.py --data-dir mscoco/ --data-type CocoDetection --split-name train2017`
`python3 tools/vision2lmdb.py --data-dir mscoco/ --data-type CocoDetection --split-name val2017`

2 . Copy packed data to current directory

2. Train
`python3  tools/train.py  -c configs/detection/fcos/fcos_efficientnetb0_bifpn.py`

3. Validation
`python3  tools/validation.py -c configs/detection/fcos/fcos_efficientnetb0_bifpn.py`

## For Unet:
1. Prepare data
`python3 tools/vision2lmdb.py --data-dir cityscapes/ --data-type Cityscapes --split-name train`
`python3 tools/vision2lmdb.py --data-dir cityscapes/ --data-type Cityscapes --split-name val`

2 . Copy packed data to current directory

3. Train
`python3 tools/train.py -c configs/segmentation/unet.py`

4. Validation
`python3  tools/validation.py -c configs/segmentation/unet.py`

## For Yolov4
1. Prepare data
`python3 tools/vision2lmdb.py --data-dir mscoco/ --data-type CocoDetection --split-name train2017`
`python3 tools/vision2lmdb.py --data-dir mscoco/ --data-type CocoDetection --split-name val2017`

2 . Copy packed data to current directory

2. Train
`python3  tools/train.py  -c configs/detection/yolov4/yolov4_efficientnetb0_bifpn.py`
