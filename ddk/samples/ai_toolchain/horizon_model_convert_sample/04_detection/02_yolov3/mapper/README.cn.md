# Yolov3

## 准备模型和数据
1. YOLOv3模型
  1.1 URL: https://github.com/ChenYingpeng/caffe-yolov3/  caffemodel 可以在该github的README.md提供的百度云下载路径中下载, 并在输出节点增加一个NCHW2NHWC的Permute操作。
  1.2 地平线model_zoo修改好的prototxt文件md5sum: yolov3_transposed.prototxt 935af6e1530af5c0017b3674adce95e9
  1.3 原始模型md5sum: yolov3.caffemodel 9a0f09c850656913ec27a6da06d9f9cc
2. COCO验证集，用于计算模型精度，可以从COCO官网下载：http://cocodataset.org/
3. 校准数据集：可以从COCO验证集中抽取50张图片作为校准数据
4. 原始浮点模型精度：`[IoU=0.50:0.95] 0.333 [IoU=0.50] 0.560`

# Yolov3

## Prepare Model and Data
1. YOLOv3 model
  1.1 URL: https://github.com/ChenYingpeng/caffe-yolov3/  caffemodel can be downloaded from the Baidu Cloud downloading path in the README.md from github, and add a Permute Operator to change output layout from NCHW to NHWC
  1.2 md5sum of horizon modified prototxt file in model_zoo: yolov3_transposed.prototxt 935af6e1530af5c0017b3674adce95e9
  1.3 md5sum of the original model: yolov3.caffemoel 9a0f09c850656913ec27a6da06d9f9cc
2. COCO verification dataset is used for computing model accuracy. It can be downloaded from COCO official website: http://cocodataset.org/
3. Calibration dataset: extract 50 images from your COCO verification dataset to serve as calibration dataset
4. origin float model accuracy : `[IoU=0.50:0.95] 0.333 [IoU=0.50] 0.560`
