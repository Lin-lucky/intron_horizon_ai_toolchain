# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import numpy as np
from horizon_nn.custom import op_implement_register


@op_implement_register("CustomIdentity")
class CustomIdentity(object):
    def __init__(self, kernel_size, threshold):
        self._kernel_size = kernel_size
        self._default_threshold = threshold

    def compute(self, X):
        return X


@op_implement_register("GlobalAveragePool")
class GlobalAveragePool(object):
    def __init__(self):
        pass

    def compute(self, X):
        return np.nanmean(X, axis=(2, 3)).reshape(-1, 1024, 1, 1)
