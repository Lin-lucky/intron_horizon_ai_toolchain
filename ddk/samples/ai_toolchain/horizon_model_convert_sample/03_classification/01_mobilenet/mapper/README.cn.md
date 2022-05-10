# MobileNetV1, MobileNetV2

## 准备模型和数据 

1. caffe 模型
    1.1 可以从 URL: https://github.com/shicai/MobileNet-Caffe  获得  
    1.2 md5sum: 3fd6889ec48bda46451d67274144e2a8  mobilenet.caffemodel
                8922f90f629d428fecf866e798ac7c08  mobilenet_deploy.prototxt
                54aab8425ea068d472e8e4015f22360c  mobilenet_v2.caffemodel
                13101ee86ab6d217d5fd6ed46f7a4faa  mobilenet_v2_deploy.prototxt
2. 可以从[ImageNet data](http://www.image-net.org/) 获得5万张验证图片集
3. 校验数据集：可以从ImageNet中抽取少部分图片(100张)作为模型校验数据集
4. 原始浮点模型精度：`0.7061`

# MobileNetV1, MobileNetV2

## Prepare Model and Data

1. Caffe model
    1.1 can be obtained from URL: https://github.com/shicai/MobileNet-Caffe
    1.2 md5sum: 3fd6889ec48bda46451d67274144e2a8  mobilenet.caffemodel
                8922f90f629d428fecf866e798ac7c08  mobilenet_deploy.prototxt
                54aab8425ea068d472e8e4015f22360c  mobilenet_v2.caffemodel
                13101ee86ab6d217d5fd6ed46f7a4faa  mobilenet_v2_deploy.prototxt
2. from [ImageNet data](http://www.image-net.org/) you can download 50,000 verification images
3. calibration dataset: extract small volume of images(100 pieces) from ImageNet to serve as model calibration dataset
4. origin float model accuracy : `0.7061`
