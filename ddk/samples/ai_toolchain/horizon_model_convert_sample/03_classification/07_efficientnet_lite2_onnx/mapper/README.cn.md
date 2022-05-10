# Efficientdet_lite2

## 准备模型和数据
1. onnx 模型
    1.1 可以从 URL: https://github.com/tensorflow/tpu/tree/master/models/official/efficientnet/lite 获得efficientnet-lite模型的tar包
    1.2 验证 md5sum: efficientnet-lite2.tar.gz da60c250593ee1c5eff9d3efb3c5eb92
    1.3 下载后可从tar包中得到.tflite文件, 然后可通过tflite2onnx工具 (https://pypi.org/project/tflite2onnx/) 将tflite转换为onnx模型 不同版本的tflite2onnx转换出来的layout会不一样, 若转换出来的onnx模型的输入layout是NCHW排布，则build时input_layout_train应该选择 NCHW
2. 可以从[ImageNet data](http://www.image-net.org/) 获得5万张验证图片集
3. 校验数据集：可以从ImageNet中抽取少部分图片(100张)作为模型校验数据集
4. 原始浮点模型精度：`0.7738`

# Efficientdet_lite2

## Prepare Model and Data
1. ONNX model
    1.1 the tar package of efficientnet-lite can be obtained from URL: https://github.com/tensorflow/tpu/tree/master/models/official/efficientnet/lite
    1.2 verification md5sum: efficientnet-lite2.tar.gz da60c250593ee1c5eff9d3efb3c5eb92
    1.3 after download, obtain the .tflite file from the tar package, than convert tflite into ONNX model using the tflite2onnx tool (https://pypi.org/project/tflite2onnx/). Different versions of tflite2onnx will generate model with different output layout. If the outcome onnx model output layout is NCHW, then the input_layout_train should choose NCHW during build phase.
2. from [ImageNet data](http://www.image-net.org/) you can download 50,000 verification images
3. calibration dataset: extract small volume of images(100 pieces) from ImageNet to serve as model calibration dataset
4. origin float model accuracy : `0.7738`
