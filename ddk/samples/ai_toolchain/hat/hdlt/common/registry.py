# Copyright (c) Horizon Robotics. All rights reserved.

import inspect
import six


class Registry(object):
    def __init__(self, name):
        self._name = name
        self._module_dict = dict()

    @property
    def name(self):
        return self._name

    @property
    def module_dict(self):
        return self._module_dict

    def get(self, name):
        if name in self._module_dict:
            res = self._module_dict[name]
        else:
            raise KeyError("No object named '{}' found in '{}' registry! \
                Please check '{}' was registered before builded in __init__. \
                Sometimes due to third party lib missing.".format(
                    name, self._name, name
                )
            )
        return res

    def register_module(self, cls, cls_name=None, force_replace=False):
        if cls_name is None:
            cls_name = cls.__name__

        if not force_replace and cls_name in self._module_dict:
            raise KeyError('{} is alreay registered in {}'.format(
                cls_name, self._name
            ))
        self._module_dict[cls_name] = cls
        return cls


def build_from_cfg(cfg, registry, default_args=None):
    r"""Build a module from config dict.

    cfg : Config dict
        It should at least contain the key "type".
    registry : Registry
        The registry to search the type from.
    default_args : dict
        Default initialization arguments.

    Returns:
        obj: The constructed object.
    """

    assert isinstance(cfg, dict) and 'type' in cfg
    assert isinstance(default_args, dict) or default_args is None
    args = cfg.copy()
    obj_type = args.pop('type')
    if isinstance(obj_type, six.string_types):
        obj_cls = registry.get(obj_type)
        if obj_cls is None:
            raise KeyError('{} is not in the {} registry'.format(
                obj_type, registry.name))
    elif inspect.isclass(obj_type):
        obj_cls = obj_type
    else:
        raise TypeError('type must be a str or valid type, but got {}'.format(
            type(obj_type)))
    if default_args is not None:
        for name, value in default_args.items():
            args[name] = value
    return obj_cls(**args)
