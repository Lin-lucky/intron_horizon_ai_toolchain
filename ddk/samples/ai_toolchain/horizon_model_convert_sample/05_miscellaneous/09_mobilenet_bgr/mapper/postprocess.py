# Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

import numpy as np
import logging
from multiprocessing.pool import ApplyResult
from horizon_tc_ui.data.imagenet_val import imagenet_val
from horizon_tc_ui.utils import tool_utils


def postprocess(model_output):
    prob = np.squeeze(model_output[0])
    idx = np.argsort(-prob)
    top_five_label_probs = [(idx[i], prob[idx[i]], imagenet_val[idx[i]])
                            for i in range(5)]
    return top_five_label_probs
