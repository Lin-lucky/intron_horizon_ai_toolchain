# Copyright (c) Horizon Robotics. All rights reserved.

from typing import Sequence

import numpy as np
import torch
from torch import Tensor
from torch._six import container_abcs, int_classes, string_classes
from torch.utils.data._utils.collate import np_str_obj_array_pattern


def _as_list(obj):
    """
    A utility function that converts the argument
    to a list if it is not already.
    """

    if isinstance(obj, (list, tuple)):
        return obj
    else:
        return [obj]


def _as_numpy(a):
    """Convert a (list of) numpy into numpy.ndarray"""
    if isinstance(a, (list, tuple)):
        out = [x for x in a]
        try:
            out = np.concatenate(out, axis=0)
        except ValueError:
            out = np.array(out)
        return out
    return a


def _convert_numpy(data, to_list=False):
    r"""Converts each Tensor array data field into a numpy"""
    elem_type = type(data)
    if (
        elem_type.__module__ == "numpy"
        and elem_type.__name__ != "str_"
        and elem_type.__name__ != "string_"
    ):
        data = data.tolist() if to_list else data
        return data
    elif isinstance(data, torch.Tensor):
        data = data.numpy().tolist() if to_list else data.numpy()
        return data
    elif isinstance(data, container_abcs.Mapping):
        return {key: convert_numpy(data[key]) for key in data}
    elif isinstance(data, tuple) and hasattr(data, "_fields"):  # namedtuple
        return elem_type(*(_convert_numpy(d) for d in data))
    elif isinstance(data, container_abcs.Sequence) and not isinstance(
        data, string_classes
    ):
        return [convert_numpy(d) for d in data]
    else:
        return data


def _get_keys_from_dict(target_dict, field):
    field_found = []
    for k, v in target_dict.iteritems():
        if key == field:
            field_found.append(v)
        elif isinstance(value, dict):
            results = _get_keys_from_dict(value, field)
            for result in results:
                field_found.append(result)
        elif isinstance(value, list):
            for item in value:
                if isinstance(item, dict):
                    more_results = _get_keys_from_dict(item, field)
                    for another_result in more_results:
                        fields_found.append(another_result)
    return fields_found


def _to_cuda(data, device=None, non_blocking=False):
    if isinstance(data, dict):
        for k in data:
            data[k] = _to_cuda(data[k], device, non_blocking)
    elif isinstance(data, Sequence):
        t = type(data)
        data = t(_to_cuda(d, device, non_blocking) for d in data)
    elif isinstance(data, torch.Tensor):
        data = data.cuda(device, non_blocking=non_blocking)
    else:
        raise TypeError
    return data
