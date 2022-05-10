# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.
"""
This tutorial shows how to do Quantization-aware training(QAT).
Warning: we use a lot of boilerplate code from other PyTorch repos to,
for example, define the ``MobileNetV2`` model archtecture, define data loaders,
and so on. We of course encourage you to read it.

Quantization-aware training (QAT) is the quantization method that typically
results in the highest accuracy. With QAT, all weights and activations are
“fake quantized” during both the forward and backward passes of training:
that is, float values are rounded to mimic int8 values, but all computations
are still done with floating point numbers. Thus, all the weight adjustments
during training are made while “aware” of the fact that the model will
ultimately be quantized.

We'll start by doing the necessary imports:
"""
import torch
import torch.nn as nn
import torchvision
from torch.utils.data import DataLoader
import torchvision.transforms as transforms
import torchvision.models as models
import os
import time

# # Setup warnings
import warnings

warnings.filterwarnings(action="ignore",
                        category=DeprecationWarning,
                        module=r".*")
warnings.filterwarnings(action="ignore", module=r"horizon_nn.torch")

# Specify random seed for repeatable results
torch.manual_seed(191009)

######################################################################
# 1. Model architecture
# ---------------------
# load mobilenet_v2 from torchvision, and by default we will using pre-trained model.


def load_model():
    model = models.__dict__["mobilenet_v2"](pretrained=True)
    model.to("cpu")
    return model


######################################################################
# 2. Helper functions
# -------------------
#
# We next define several helper functions to help with model evaluation.
# These mostly come from
# `here <https://github.com/pytorch/examples/blob/master/imagenet/main.py>`_.


class AverageMeter(object):
    """Computes and stores the average and current value"""
    def __init__(self, name, fmt=":f"):
        self.name = name
        self.fmt = fmt
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count

    def __str__(self):
        fmtstr = "{name} {val" + self.fmt + "} ({avg" + self.fmt + "})"
        return fmtstr.format(**self.__dict__)


def accuracy(output, target, topk=(1, )):
    """Computes the accuracy over the k top predictions for the specified
    values of k
    """
    with torch.no_grad():
        maxk = max(topk)
        batch_size = target.size(0)

        _, pred = output.topk(maxk, 1, True, True)
        pred = pred.t()
        correct = pred.eq(target.view(1, -1).expand_as(pred))

        res = []
        for k in topk:
            correct_k = correct[:k].reshape(-1).float().sum(0, keepdim=True)
            res.append(correct_k.mul_(100.0 / batch_size))
        return res


def evaluate(model,
             criterion,
             data_loader,
             neval_batches,
             device=torch.device("cpu")):
    model.eval()
    top1 = AverageMeter("Acc@1", ":6.2f")
    top5 = AverageMeter("Acc@5", ":6.2f")
    cnt = 0

    with torch.no_grad():
        for image, target in data_loader:
            image, target = image.to(device), target.to(device)
            output = model(image)
            # add view
            output = output.view(-1, 1000)
            loss = criterion(output, target)
            cnt += 1
            acc1, acc5 = accuracy(output, target, topk=(1, 5))
            print(".", end="")
            top1.update(acc1[0], image.size(0))
            top5.update(acc5[0], image.size(0))
            if cnt >= neval_batches:
                print('')
                return top1, top5

    return top1, top5


######################################################################
# 3. Define dataset and data loaders
# ----------------------------------
#
# As our last major setup step, we define our dataloaders for our training
# and testing set.
#
# ImageNet Data
# ^^^^^^^^^^^^^
#
# The specific dataset we've created for this tutorial contains just 1000
# images from the ImageNet data, one from each class (this dataset, at just
# over 250 MB, is small enough that it can be downloaded relatively easily).
# The URL for this custom dataset is:
#
# .. code::
#
#     https://s3.amazonaws.com/pytorch-tutorial-assets/imagenet_1k.zip
#     or
#     hdfs://hobot-bigdata/user/shuqian.qu/data/imagenet_1k.zip
#
# To run the code in this tutorial using the entire ImageNet dataset, on the
# other hand, you could download the data using ``torchvision`` following
# `here <https://pytorch.org/docs/stable/torchvision/datasets.html#imagenet>`_.
# For example, to download the training set and apply some standard
# transformations to it, you could use:
#
# .. code:: python
#
#     import torchvision
#     import torchvision.transforms as transforms
#
#     imagenet_dataset = torchvision.datasets.ImageNet(
#         '~/.data/imagenet',
#         split='train',
#         download=True,
#         transforms.Compose([
#             transforms.RandomResizedCrop(224),
#             transforms.RandomHorizontalFlip(),
#             transforms.ToTensor(),
#             transforms.Normalize(mean=[0.485, 0.456, 0.406],
#                                  std=[0.229, 0.224, 0.225]),
#         ])
#
# With the data downloaded, we show functions below that define dataloaders
# we'll use to read in this data. These functions mostly come from
# `here <https://github.com/pytorch/vision/blob/master/references/detection/train.py>`_.  # noqa


def prepare_data_loaders(data_path):
    traindir = os.path.join(data_path, "train")
    valdir = os.path.join(data_path, "val")
    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                     std=[0.229, 0.224, 0.225])

    dataset = torchvision.datasets.ImageFolder(
        traindir,
        transforms.Compose([
            transforms.RandomResizedCrop(224),
            transforms.RandomHorizontalFlip(),
            transforms.ToTensor(),
            normalize,
        ]),
    )

    dataset_test = torchvision.datasets.ImageFolder(
        valdir,
        transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            normalize,
        ]),
    )

    train_sampler = torch.utils.data.RandomSampler(dataset)
    test_sampler = torch.utils.data.SequentialSampler(dataset_test)

    data_loader = torch.utils.data.DataLoader(dataset,
                                              batch_size=train_batch_size,
                                              sampler=train_sampler)

    data_loader_test = torch.utils.data.DataLoader(dataset_test,
                                                   batch_size=eval_batch_size,
                                                   sampler=test_sampler)

    return data_loader, data_loader_test


def train_one_epoch(model, criterion, optimizer, data_loader, device,
                    ntrain_batches):
    model.train()
    top1 = AverageMeter("Acc@1", ":6.2f")
    top5 = AverageMeter("Acc@5", ":6.2f")
    avgloss = AverageMeter("Loss", "1.5f")

    cnt = 0
    for image, target in data_loader:
        cnt += 1
        image, target = image.to(device), target.to(device)
        output = model(image)
        # add view
        output = output.view(-1, 1000)
        loss = criterion(output, target)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        acc1, acc5 = accuracy(output, target, topk=(1, 5))
        top1.update(acc1[0], image.size(0))
        top5.update(acc5[0], image.size(0))
        avgloss.update(loss, image.size(0))
        if cnt >= ntrain_batches:
            print("Loss", avgloss.avg)

            print(
                "Training: * Acc@1 {top1.avg:.3f} Acc@5 {top5.avg:.3f}".format(
                    top1=top1, top5=top5))
            return

    print("Full imagenet train set:  * Acc@1 {top1.global_avg:.3f} Acc@5"
          " {top5.global_avg:.3f}".format(top1=top1, top5=top5))
    return


data_path = "data/imagenet_1k"
train_batch_size = 30
eval_batch_size = 30

data_loader, data_loader_test = prepare_data_loaders(data_path)
criterion = nn.CrossEntropyLoss()

######################################################################
# create float module
float_model = load_model()
device = torch.device("cuda:0")
float_model = float_model.to(device)
num_eval_batches = 10
top1, top5 = evaluate(
    float_model,
    criterion,
    data_loader_test,
    neval_batches=num_eval_batches,
    device=device,
)
print("Float evaluation accuracy on %d images, %2.2f" %
      (num_eval_batches * eval_batch_size, top1.avg))

optimizer = torch.optim.SGD(float_model.parameters(), lr=0.0001)
# We need to use a ``HorizonQConfig`` specifying what kind of fake-quantization is
# to be inserted after weights and activations, instead of specifying observers
# use HorizonQConfig
from horizon_nn.torch import HorizonQConfig, adjust_qat_bits
######################################################################
# second, ``prepare_qat_fx`` performs the "fake quantization", preparing
# the model for quantization-aware training. model must be in training.
from torch.quantization.quantize_fx import prepare_qat_fx

qat_model = prepare_qat_fx(float_model.train(), HorizonQConfig)

# finally, we try to adjust qat graph to make it friendly to BPU, this step is not necessary.
qat_model = adjust_qat_bits(qat_model)

######################################################################
# Training a quantized model with high accuracy requires accurate modeling
# of numerics at inference.

num_train_batches = 20
qat_model = qat_model.to(device)

# Train and check accuracy after each epoch
for nepoch in range(1):
    from horizon_nn.torch import set_qat_train, set_qat_eval

    qat_model = set_qat_train(qat_model)
    train_one_epoch(qat_model, criterion, optimizer, data_loader, device,
                    num_train_batches)
    # set model run on eval
    qat_model = set_qat_eval(qat_model)
    top1, top5 = evaluate(
        qat_model,
        criterion,
        data_loader_test,
        neval_batches=num_eval_batches,
        device=device,
    )
    print("QAT Epoch %d :float evaluation accuracy on %d images, %2.2f" %
          (nepoch, num_eval_batches * eval_batch_size, top1.avg))

# using horizon_nn.torch.convert to convert qat model into int model, and eval accuracy.
# the accuracy of int model is same with accuracy running on chip.
from horizon_nn.torch import convert

# to quantize qat model, dummy data is necessary.
dummy_data = torch.randn((1, 3, 224, 224))
quantized_model = convert(
    qat_model,  # qat model
    dummy_data,  # dummy data, or real data, which is the input data to feed the qat model
    march='bernoulli2'  # quantization march, default is bayes
)

# export your model to visualize by netron(https://netron.app/)
quantized_model.export("quantized_model.onnx")

quantized_model.eval().to(device)
top1, top5 = evaluate(
    quantized_model,
    criterion,
    data_loader_test,
    neval_batches=num_eval_batches,
    device=device,
)
print("Quantized Epoch %d :int evaluation accuracy on %d images, %2.2f" %
      (nepoch, num_eval_batches * eval_batch_size, top1.avg))
# using compile to compile your quantized_model,
# it will save bin file and perf info files.
# from horizon_nn.torch.quantize import compile
from horizon_tc_ui.torch.qat_compile import compile
from horizon_nn.torch.functional import eliminate_transpose_and_quanti

quantized_model = eliminate_transpose_and_quanti(quantized_model, "bernoulli2")

compile(
    quantized_model,
    "test.onnx",
    march="bernoulli2",
    rt_input_type="nv12",
    rt_input_layout="NCHW",
    opt="O0",
)
