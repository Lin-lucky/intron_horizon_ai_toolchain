import random
from torch.utils.data import DataLoader, Dataset
from torchvision.datasets.vision import VisionDataset

from hdlt.data.registry import DATASETS, TRANSFORMS
from hdlt.common.registry import build_from_cfg

@DATASETS.register_module
class ComplexDataset(Dataset):
    def __init__(self, dataset_cfg, transforms_cfg):
        super(ComplexDataset, self).__init__()
        self.dataset = build_from_cfg(dataset_cfg, DATASETS)
        self.image_needed = 1
        self.transforms = [build_from_cfg(transform, TRANSFORMS) for transform in transforms_cfg] 
        for trans in self.transforms:
            if hasattr(trans, 'images_needed'):
                needs = trans.images_needed
            else:
                needs = 1
            self.image_needed *= needs
        print(self.__dict__)

    def __getstate__(self):
        state = self.__dict__
        dataset_state = self.dataset.__getstate__()
        state['dataset_state'] = dataset_state
        return state

    def __setstate__(self, state):
        self.__dict__ = state
        dataset_state = state['dataset_state']
        self.dataset.__setstate__(dataset_state)

    def __len__(self):
        return self.dataset.__len__() 

    def class_names(self):
        return self.dataset.class_names()

    def __getitem__(self, item):
        data = self.dataset.__getitem__(item)
        datalist = [data]
        
        indexes = [random.randint(0, self.__len__() -1) for _ in range(self.image_needed - 1)]
        for i in indexes:
            data_addition = self.dataset[i]
            datalist.append(data_addition)
        for trans in self.transforms:
            if hasattr(trans, 'images_needed'):
                needs = trans.images_needed
                data = trans(datalist[:needs])
            else:
                needs = 1
                data = trans(datalist[0])
            datalist = datalist[needs:]
            datalist.append(data)
             
        return data   
