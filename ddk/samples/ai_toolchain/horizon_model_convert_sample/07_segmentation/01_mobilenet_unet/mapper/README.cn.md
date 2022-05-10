# mobilenet_unet

## 准备模型和数据
1. Onnx 模型
    1.1 可以从 URL: https://github.com/HorizonRobotics-Platform/ModelZoo/tree/master/MobilenetUnet 获得  
    1.2 md5sum: tf_unet_trained.onnx  21c6c645ebca92befbebc8c39d385c1e
2. Cityscapes 验证集，用于计算模型精度，可以从Cityscapes官网下载：https://www.cityscapes-dataset.com/
3. 校准数据集：可以从Cityscapes验证集中抽取50张图片作为校准数据
4. 原始浮点模型精度：`0.6411`

# mobilenet_unet

## Prepare Model and Data
1. Onnx Model
    1.1 Model can be obtained from URL: https://github.com/HorizonRobotics-Platform/ModelZoo/tree/master/MobilenetUnet
    1.2 md5sum: tf_unet_trained.onnx  21c6c645ebca92befbebc8c39d385c1e
2. Cityscapes verification dataset is used for calculating model accuracy. Please download it from Cityscapes official website: https://www.cityscapes-dataset.com/.
3. Calibration dataset: user can extract 50 images from Cityscapes verification dataset to serve as calibration dataset.
4. origin float model accuracy : `0.6411`
