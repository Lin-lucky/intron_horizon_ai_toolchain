# Copyright (c) Horizon Robotics. All rights reserved.

import torch
import torch.nn as nn


def count_parameters(m, x, y):
    total_params = 0
    for p in m.buffers():
        total_params += torch.DoubleTensor([p.numel()])
    for p in m.parameters():
        total_params += torch.DoubleTensor([p.numel()])
    m.total_params[0] = total_params


def count_linear(m, x, y):
    total_mul = m.in_features
    num_elements = (
        y.numel() if isinstance(y, torch.Tensor)
        else y.float.numel()
    )
    total_ops = total_mul * num_elements
    m.total_ops += torch.DoubleTensor([int(total_ops)])


def count_convNd(m, x, y):
    kernel_ops = torch.zeros(m.weight.size()[2:]).numel()  # Kw x Kh

    # N x Cout x H x W x  (Cin x Kw x Kh + bias)
    nelement = (
        y.nelement() if isinstance(y, torch.Tensor)
        else y.float.nelement()
    )
    # index 1 represent channel
    in_channels = (
        x[0].shape[1] if isinstance(x[0], torch.Tensor)
        else x[0].float.shape[1]
    )
    try:
        groups = m._conv_kwargs["groups"]
    except Exception as e:
        # for nn.Conv2d, qat.Conv2d
        groups = m.groups

    total_ops = nelement * (
        in_channels // groups * kernel_ops
    )
    m.total_ops += torch.DoubleTensor([int(total_ops)])


register_op_hooks = {
    nn.Conv2d: count_convNd,
    nn.Linear: count_linear
}
op_collect = {}


def add_op_count_hooks(m: nn.Module):
    m.register_buffer('total_ops', torch.zeros(1, dtype=torch.float64))
    m.register_buffer('total_params', torch.zeros(1, dtype=torch.float64))

    m_type = type(m)
    if m_type in register_op_hooks:
        fn = register_op_hooks[m_type]
        op_collect[m] = (
            m.register_forward_hook(fn),
            m.register_forward_hook(count_parameters)
        )


def dfs_count(model: nn.Module):
    total_ops, total_params = 0.0, 0.0
    for m in model.children():
        if m in op_collect and \
                not isinstance(m, (nn.Sequential, nn.ModuleList)):
            m_ops, m_params = m.total_ops.item(), m.total_params.item()
        else:
            m_ops, m_params = dfs_count(m)
        total_ops += m_ops
        total_params += m_params
    return total_ops, total_params


def profile(model: nn.Module, inputs):
    model.eval()
    model.apply(add_op_count_hooks)
    with torch.no_grad():
        model(inputs)
    return dfs_count(model)
