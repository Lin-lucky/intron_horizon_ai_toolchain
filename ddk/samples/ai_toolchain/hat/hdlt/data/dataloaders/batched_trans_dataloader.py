import warnings

import torch

from ..registry import DATALOADERS


class TransformWrappedIter:
    def __init__(self, iter, transforms):
        self.iter = iter
        self.transforms = transforms

    def __iter__(self):
        return self

    def __next__(self):
        image, target = self.iter.__next__()
        image = image.cuda()
        target = target.cuda()
        data = {"image": image, "label": target}
        if len(self.transforms) > 0:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                for transform in self.transforms:
                    data = transform(data)
        return data["image"], data["label"]

    def _reset(self):
        self.iter._reset()


@DATALOADERS.register_module
class BatchedTransDataloader(torch.utils.data.DataLoader):
    r"""
    A dataloader that apply batched transform.

    Args:
        transforms (list, optional): Transforms that accept batched data
            in Tensor. Defaults to [].
        Other args: Please refer to ``torch.utils.data.DataLoader``

    Returns:
        Please refer to ``torch.utils.data.DataLoader``
    """

    def __init__(
        self,
        dataset,
        transforms=[],
        batch_size=1,
        shuffle: bool = False,
        sampler=None,
        batch_sampler=None,
        num_workers: int = 0,
        collate_fn=None,
        pin_memory: bool = False,
        drop_last: bool = False,
        timeout: float = 0,
        worker_init_fn=None,
        multiprocessing_context=None,
        generator=None,
        *,
        prefetch_factor: int = 2,
        persistent_workers: bool = False
    ):
        super(BatchedTransDataloader, self).__init__(
            dataset,
            batch_size,
            shuffle,
            sampler,
            batch_sampler,
            num_workers,
            collate_fn,
            pin_memory,
            drop_last,
            timeout,
            worker_init_fn,
            multiprocessing_context,
            generator,
            prefetch_factor=prefetch_factor,
            persistent_workers=persistent_workers,
        )

        self.transforms = []
        for transform in transforms:
            if "torchvision" in transform.__class__.__module__:

                def wrapper(data: dict):
                    data["image"] = transform(data["image"])
                    return data

                self.transforms.append(wrapper)
            else:
                self.transforms.append(transform)

    def _get_iterator(self):
        it = super(BatchedTransDataloader, self)._get_iterator()
        return TransformWrappedIter(it, self.transforms)
