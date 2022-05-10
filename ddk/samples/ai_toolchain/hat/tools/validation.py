"""validation tools"""
import argparse
import logging
import os
import time

import torch

from hdlt.common import Config, init_logger
from hdlt.common.utils import _as_list, _to_cuda
from hdlt.data.builder import build_dataloader
from hdlt.metrics.builder import build_metric
from hdlt.models.builder import build_model


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        "-c",
        type=str,
        required=True,
        help="train config file path",
    )
    parser.add_argument("--gpu", type=int, default=0)
    return parser.parse_args()


def load_model(cfg, model, logger):
    solver = cfg["validation"]
    if solver["pretrained_model"] is not None:
        checkpoint = torch.load(solver["pretrained_model"], map_location="cpu")
        param_name = list(checkpoint["state_dict"].keys())[0]
        if param_name.startswith("module"):
            model = torch.nn.DataParallel(model)
        model.load_state_dict(checkpoint["state_dict"], strict=False)
        if param_name.startswith("module"):
            model = model.module
        logger.info(
            "load pretrained model from %s" % (solver["pretrained_model"])
        )

    return model


def eval(dataloader, model, metrics, logger, prefix=""):
    model.cuda().eval()
    for met in metrics:
        met.reset()
    with torch.no_grad():
        for inputs, targets in dataloader:
            inputs = _to_cuda(inputs)
            targets = _to_cuda(targets)
            outputs, targets = model(inputs, targets)

            for met in metrics:
                met.update(targets, outputs)

    log_info = "{}: ".format(prefix)
    for metric in metrics:
        name, value = metric.get()
        for k, v in zip(_as_list(name), _as_list(value)):
            log_info += "{}={}".format(k, v)
    logger.info(log_info)


def main(cfg, gpu):
    os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu)
    time_stamp = time.strftime(
        "%Y%m%d%H%M%S", time.localtime(int(time.time()))
    )
    file_name = cfg.file_name + "_validation_" + time_stamp
    log_file = os.path.join(".hdlt_logs/", file_name)
    init_logger(log_file=log_file)
    logger = logging.getLogger()
    logger.info(cfg)

    solver = cfg["validation"]
    model = build_model(cfg.model)
    dataloader = build_dataloader(cfg.val_dataloaders)
    metrics = build_metric(solver["metrics"])

    # float result
    model = load_model(cfg, model, logger)
    eval(dataloader, model, metrics, logger, "Float")


if __name__ == "__main__":
    args = parse_args()
    config = Config.fromfile(args.config)
    main(config, args.gpu)
