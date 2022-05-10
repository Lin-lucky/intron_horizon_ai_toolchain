# Copyright (c) Horizon Robotics. All rights reserved.

from hdlt.common.registry import build_from_cfg

from .registry import OPTIMIZERS


def build_optimizer(cfg, default_args=None):
    if cfg is None:
        return None

    assert "model" in default_args
    if "params" in cfg:
        loc_name = dict()
        for k, v in cfg["params"].items():
            loc_name[k] = {"params": []}
            loc_name[k].update(v)

        loc_name["others"] = {
            "params": [],
            "weight_decay": (
                 cfg["weight_decay"] if "weight_decay" in cfg else 0
            ),
        }
        for name, p in default_args["model"].named_parameters():
            if not p.requires_grad:
                pass
            flag = False
            for k, v in cfg["params"].items():
                if k in name:
                    loc_name[k]["params"].append(p)
                    flag = True
                    break
            if not flag:
                loc_name["others"]["params"].append(p)

        res = []
        for k, v in loc_name.items():
            res.append(v)
        cfg["params"] = res
    else:
        cfg["params"] = filter(
            lambda p: p.requires_grad,
            default_args["model"].parameters()
        )

    return build_from_cfg(cfg, OPTIMIZERS)
