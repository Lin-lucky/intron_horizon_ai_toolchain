import os
import time

import numpy as np
import torch
from PIL import Image
from torch import nn

file_name = "float_cityscapes_unet"
data_shape = (3, 1024, 2048)
num_classes = 19
start_lr = 0.01
train_epochs = 300
train_scales = (4, 8, 16, 32, 64)
alpha = 0.25
bn_kwargs = dict(eps=2e-5, momentum=0.1)
weight_decay = 5e-3

batch_size_per_gpu = 2
context = "0, 1, 2, 3, 4, 5, 6, 7"
dataloader_workers = batch_size_per_gpu  # per gpu
workdir = "."
save_prefix = "models"
cudnn_benchmark = True
seed = None

num_gpus = len(context.split(","))
save_prefix = os.path.join(workdir, save_prefix, "segmentation")
model_save_path = os.path.join(save_prefix, file_name)

# model
model = dict(
    type="Segmentor",
    backbone=dict(
        type="MobileNetV1",
        alpha=alpha,
        dw_with_relu=True,
        bn_kwargs=bn_kwargs,
        act_type=nn.ReLU6,
        include_top=False,
    ),
    neck=dict(
        type="Unet",
        base_channels=int(32 * alpha),
        bn_kwargs=bn_kwargs,
        act_type=nn.ReLU6,
        use_deconv=False,
        dw_with_relu=True,
        output_scales=train_scales,
    ),
    head=dict(
        type="SegHead",
        input_channels=tuple(np.array(train_scales) * int(32 * alpha)),
        output_channels=(num_classes,) * len(train_scales),
        bn_kwargs=bn_kwargs,
    ),
    losses=dict(
        type="SequenceFocalLoss",
        num_classes=num_classes,
        reduction="mean",
        weight=tuple(np.array((256, 128, 64, 32, 16)) / 19),
    ),
)

# data
train_dataloader = dict(
    type="BatchedTransDataloader",
    dataset=dict(
        type="CityscapesFromLMDB",
        data_path="/home/users/rui.xu/workspace/opt_det/rel/dataset/train_lmdb",
        transforms=[
            dict(type="SegPILToTensor", key="label", to_cuda=False),
            dict(type="SegPILToTensor", key="image", to_cuda=False),
        ],
        num_samples=2975,
    ),
    transforms=[
        dict(type="LabelRemap"),
        dict(type="SegOneHot", num_classes=num_classes),
        dict(type="SegResize", size=data_shape[1:]),
        dict(
            type="SegRandomAffine",
            degrees=0,
            scale=(0.5, 2.0),
        ),
        dict(type="SegBgrToYuv444", rgb_input=True),
        dict(type="SegNormalize", mean=128.0, std=128.0),
        dict(type="Scale", scales=tuple(1 / np.array(train_scales))),
    ],
    sampler=dict(type=torch.utils.data.DistributedSampler),
    batch_size=batch_size_per_gpu,
    shuffle=True,
    num_workers=dataloader_workers,
    pin_memory=True,
)

val_dataloaders = dict(
    type="BatchedTransDataloader",
    dataset=dict(
        type="CityscapesFromLMDB",
        data_path="/home/users/rui.xu/workspace/opt_det/rel/dataset/val_lmdb",
        transforms=[
            dict(type="SegPILToTensor", key="label", to_cuda=False),
            dict(type="SegPILToTensor", key="image", to_cuda=False),
        ],
        num_samples=500,
    ),
    transforms=[
        dict(type="LabelRemap"),
        dict(type="SegOneHot", num_classes=num_classes),
        dict(type="SegResize", size=data_shape[1:]),
        dict(type="SegBgrToYuv444", rgb_input=True),
        dict(type="SegNormalize", mean=128.0, std=128.0),
        dict(type="Scale", scales=tuple(1 / np.array(train_scales))),
    ],
    batch_size=batch_size_per_gpu,
    shuffle=False,
    num_workers=dataloader_workers,
    pin_memory=True,
)

workflow = dict(
    trainer=dict(
        type="DistributedDataParallelTrainer",
        epoch=train_epochs,
        optimizer=dict(
            type=torch.optim.SGD,
            lr=start_lr,
            momentum=0.9,
            weight_decay=weight_decay,
        ),
        metrics=[
            dict(type="MeanIoU", logit_input=True, num_classes=num_classes)
        ],
        sync_bn=True,
    ),
    callbacks=[
        dict(
            type="WarmupStepLrUpdater",
            base_lr=start_lr,
            warmup_epoch=0,
            lr_decay_step=[200, 240, 280],
            lr_decay_factor=0.1,
        ),
        dict(
            type="Validation",
            dataloaders=val_dataloaders,
            metrics=[
                dict(
                    type="MeanIoU", logit_input=False, num_classes=num_classes
                )
            ],
        ),
        dict(
            type="Checkpoint",
            monitor="iou",
            save_path=model_save_path,
            resume=None,
        ),
        dict(type="StatsMonitor", log_freq=40),
    ],
)

validation = dict(
    metrics=[dict(type="MeanIoU", logit_input=False, num_classes=num_classes)],
    pretrained_model=os.path.join(model_save_path, "model_best.pth.tar"),
)
