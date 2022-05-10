# Copyright (c) Horizon Robotics. All rights reserved.

import os
import subprocess
from datetime import datetime
from distutils.version import LooseVersion


def get_version(version):
    # for setup in pre release version
    if not os.getenv("TAG_NAME"):
        version += ".dev{}".format(datetime.now().strftime("%Y%m%d%H%M"))
        commit_id = "Unknow"
        cwd = os.path.dirname(os.path.abspath('__file__'))
        try:
            commit_id = (
                subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=cwd)
                .decode("ascii")
                .strip()
            )
        except Exception:
            pass
        if commit_id != "Unknow":
            version += "+" + commit_id[:7]
    package_name = "hdlt"
    print("Building wheel {}-{}".format(package_name, version))
    return version


# torch and torchvision version check
torch_version = ["1.8.1+cu101", "1.8.1+cu111"]
torchvision_version = ["0.9.1+cu101", "0.9.1+cu111"]


try:
    import torch
    t_version = [LooseVersion(version) for version in torch_version]
    if not LooseVersion(torch.__version__) in t_version:
        msg = (
            "Torch=={} is required. Current torch=={}."
            "Different version of torch may not work properly."
        ).format(torch_version, torch.__version__)
        raise ImportError(msg)
except ImportError as e:
    raise ImportError(
        "Unable to import dependency torch. {}".format(e)
    )

try:
    import torchvision as tv
    tv_version = [LooseVersion(version) for version in torchvision_version]
    if not LooseVersion(tv.__version__) in tv_version:
        msg = (
            "Torchvision=={} is required. Current torchvision=={}."
            "Different version of torchvision may not work properly."
        ).format(torchvision_version, tv.__version__)
        raise ImportError(msg)
except ImportError as e:
    raise ImportError(
        "Unable to import dependency torchvision. {}".format(e)
    )


# current version
__version__ = "0.3.1"
