# MobilenetSSD

## 准备模型和数据
1. MobilenetSSD模型
  1.1 可以从 URL: https://github.com/chuanqi305/MobileNet-SSD 获得 caffe 模型, prototxt中将模型后端的 priorBox 和 detection OP拿掉, 放在了后处理代码中. 
  1.2 验证 md5sum: mobilenet_iter_73000.caffemodel bbcb3b6a0afe1ec89e1288096b5b8c66
2. VOC验证集，用于计算模型精度，可以从VOC官网下载： http://host.robots.ox.ac.uk/pascal/VOC/voc2012/
3. 校准数据集：可以从VOC验证集中抽取50张图片作为校准数据
4. 原始浮点模型精度：`0.7342`

# MobilenetSSD

## Prepare Model and Data
1. MobilenetSSD model
  1.1 Caffe model can be downloaded from URL: https://github.com/chuanqi305/MobileNet-SSD. priorBox and detection OP are taken away from the prototxt, and added to the post processing code.
  1.2 verification md5sum: mobilenet_iter_73000.caffemodel bbcb3b6a0afe1ec89e1288096b5b8c66
2. VOC verification dataset is used for computing model accuracy and can be downloaded from VOC official website: http://host.robots.ox.ac.uk/pascal/VOC/voc2012/
3. Calibration dataset: extract 50 images from VOC verification dataset to serve as calibration dataset
4. origin float model accuracy : `0.7342`
