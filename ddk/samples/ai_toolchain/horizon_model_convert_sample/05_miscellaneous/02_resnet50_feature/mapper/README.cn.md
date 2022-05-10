# Resnet_50_feature

## 准备模型和数据
1. caffe 模型
    1.1 可以从 URL: https://github.com/KaimingHe/deep-residual-networks 获得
    1.2 验证 md5sum: ResNet-50-deploy.prototxt 4ba3f945e7b86b07648e4f4351de0699
    1.3 在prototxt中去掉第一个res-block
2. 可以从[ImageNet data](http://www.image-net.org/) 获得5万张验证图片集
3. 校验数据集：可以从ImageNet中抽取少部分图片(100张)作为模型校验数据集, 但需要将图片进行处理, 处理成为适当的feature输入的格式

# Resnet_50_feature

## Prepare Model and Data

1. Caffe model
    1.1 can be obtained from URL: https://github.com/KaimingHe/deep-residual-networks
    1.2 verification md5sum: ResNet-50-deploy.prototxt 4ba3f945e7b86b07648e4f4351de0699
    1.3 remove the first res-block in prototxt
2. from [ImageNet data](http://www.image-net.org/) you can download 50,000 verification images
3. calibration dataset: extract small volume of images(100 pieces) from ImageNet to serve as model calibration dataset
