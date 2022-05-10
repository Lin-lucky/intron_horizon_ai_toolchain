# centernet

## 准备模型和数据
1. Onnx 模型
    1.1 可以从 URL: https://github.com/HorizonRobotics-Platform/ModelZoo/tree/master/Centernet 获得  
    1.2 md5sum: centernet_resnet50.onnx  fa1e884882a54fa3520d1e51477b4c1a
2. COCO验证集，用于计算模型精度，可以从COCO官网下载：http://cocodataset.org/
3. 校准数据集：可以从COCO验证集中抽取50张图片作为校准数据
4. 原始浮点模型精度：`[IoU=0.50:0.95] 0.318 [IoU=0.50] 0.505`

# centernet
## Prepare Model and Data
1. Onnx Model
    1.1  model can be obtained from URL: https://github.com/HorizonRobotics-Platform/ModelZoo/tree/master/Centernet
    1.2 md5sum: centernet_resnet50.onnx  fa1e884882a54fa3520d1e51477b4c1a

2. COCO verification dataset is used for computing model accuracy. It can be downloaded from COCO official website: http://cocodataset.org/
3. Calibration dataset: extract 50 images from your COCO verification dataset to serve as calibration dataset
4. origin float model accuracy : `[IoU=0.50:0.95] 0.318 [IoU=0.50] 0.505`
