import torch
import torchvision

file_name = "float_fcos"
data_shape = (3, 512, 512)
batch_size_per_gpu = 28
context = "2, 3" #"0, 1, 2,3"
save_prefix = "./models"
num_gpus = len([int(i) for i in context.split(",")])
batch_size = batch_size_per_gpu * num_gpus
cudnn_benchmark = True
seed = None
check_quantize_model = False
quantize = False

# model
model = dict(
    type="FCOS",
    backbone=dict(type="EfficientNetB0"),
    neck=dict(
        type="BiFPN",
        out_channels=[64, 64, 64, 64, 64],
        num_outs=5,
        repeat=3,
        in_channels=[40, 112, 320],
        use_p8=False,
        attention=False,
    ),
    head=dict(
        type="FCOSHead",
        num_classes=80,
        in_channel=64,
        stacked_convs=4,
        feat_channel=64,
        strides=[8, 16, 32, 64, 128],
        shared_norm=False,
        need_centerness=True,
        centerness_on_reg=True,
    ),
    target=dict(
        type="FCOSTarget",
        match_cfg=dict(
            type="PointsMatcher",
            center_sample_radius=1.5,
        ),
    ),
    loss=dict(
        type="FCOSLoss",
        centerness_iou=True,
        cls_loss=dict(
            type="FocalLoss",
            loss_fcn=dict(
                type="CEWithWithLogitsLoss",
                weights=1.0,
                eps=0.0,
            ),
            gamma=2.0,
            alpha=0.25,
        ),
        bbox_loss=dict(
            type="IOULoss",
            iou_cfg=dict(type="GIOU"),
        ),
        centerness_loss=dict(
            type="CEWithWithLogitsLoss",
            weights=1.0,
            eps=0.0,
        ),
    ),
    post_process=dict(
        type="FCOSPostProcess",
        score_threshold=0.05,
        topk_candidates=1000,
        nms_threshold=0.6,
        detections_per_img=100,
    ),
)

# data
train_dataloader = dict(
    type=torch.utils.data.DataLoader,
    dataset=dict(
        type="CocoFromLMDB",
        data_path="../coco/train2017_lmdb",
        num_samples=118287,
        transforms=[
            dict(
                type="RandomCrop",
                ratio_range=(0.5, 2.0),
                crop_size=512,
            ),
            dict(type="PadwithDivisor", size_divisor=128),
            dict(
                type="FlipLeftRight",
                flip_ratio=0.5,
            ),
            dict(type="AugmentHSV", hgain=0.015, sgain=0.7, vgain=0.4),
            dict(type="BgrToYuv444", rgb_input=True),
            dict(type="ToTensor"),
            dict(
                type="Normalizer",
                mean=128.0,
                std=128.0,
            ),
        ],
    ),
    sampler=dict(type=torch.utils.data.DistributedSampler),
    batch_size=batch_size_per_gpu,
    shuffle=True,
    num_workers=8,
    pin_memory=True,
    collate_fn=dict(type="CoCoCollater"),
)

val_dataloaders = dict(
    type=torch.utils.data.DataLoader,
    dataset=dict(
        type="CocoFromLMDB",
        data_path="../coco/val2017_lmdb",
        transforms=[
            dict(
                type="ResizerMinMax",
                min_side=data_shape[1],
                max_side=data_shape[2],
            ),
            dict(type="PadwithDivisor", size_divisor=128),
            dict(type="BgrToYuv444", rgb_input=True),
            dict(type="ToTensor"),
            dict(
                type="Normalizer",
                mean=128.0,
                std=128.0,
            ),
        ],
        num_samples=5000,
    ),
    batch_size=batch_size_per_gpu,
    shuffle=False,
    num_workers=8,
    pin_memory=True,
    collate_fn=dict(type="CoCoCollater"),
)

workflow = dict(
    trainer=dict(
        type="DistributedDataParallelTrainer",
        epoch=300,
        optimizer=dict(
            type=torch.optim.SGD,
            params={"weight": dict(weight_decay=4e-5)},
            lr=0.14,
            momentum=0.937,
            nesterov=True,
        ),
    ),
    callbacks=[
        dict(
            type="WarmupCosLrUpdater",
            base_lr=0.14,
            warmup_epoch=0.3,
            epoch=300,
        ),
        dict(type="ExponentialMovingAverage"),
        dict(
            type="Validation",
            dataloaders=[val_dataloaders],
            metrics=[
                dict(
                    type="CocoMetrics",
                    anno_file="../coco/annotations/instances_val2017.json",
                )
            ],
            freq=1,
        ),
        dict(
            type="Checkpoint",
            monitor="~~~~ MeanAP @ IoU=[0.50,0.95 for bbox ~~~~",
            mode="max",
            save_path=save_prefix + "/" + file_name,
            resume=None,
            freq=1,
        ),
        dict(
            type="DetectionMonitor",
            log_freq=5,
            losses=["cls", "bbox", "centerness"],
        ),
    ],
)
validation = dict(
    type="validation",
    metrics=[
        dict(
            type="CocoMetrics",
            anno_file="../coco/annotations/instances_val2017.json",
        )
    ],
    pretrained_model=save_prefix + "/" + file_name + "/model_best.pth.tar",
)
