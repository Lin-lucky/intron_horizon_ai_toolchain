# Yolov2

## 准备模型和数据
1. YOLOv2模型需要首先从yolo官网(https://pjreddie.com/darknet/yolo/)下载yolov2 608x608的 .cfg和.weight文件并使用darknet2caffe (https://github.com/xingyanan/darknet2caffe) 转换工具将其转换为caffe model。(该转换工具是一个简化版本，使用时，需要修改该工具生成的.prototxt文件，将其中的'Reshape' 层修改成'Passthrough'层，Passthrough层具体修改后的参数请见提供的yolov2.prototxt例子, 并在输出节点增加一个NCHW2NHWC的Permute操作。)
2. 地平线模型发布物model_zoo中提供的模型文件，prototxt文件及md5sum：yolov2.caffemodel 7aa7a6764401cebf58e73e72fcbd2a45 yolov2_transposed.prototxt 72e9a51c1e284e4b66e69f72ca9214c8 
3. COCO验证集，用于计算模型精度，可以从COCO官网下载：http://cocodataset.org/
4. 校准数据集：可以从COCO验证集中抽取50张图片作为校准数据
5. 原始浮点模型精度：`[IoU=0.50:0.95] 0.276 [IoU=0.50] 0.494`

# YOLOv2

## Prepare Model and Data
1. Firstly, download the YOLOv2 608x608 .cfg and .weight files from YOLO official website(https://pjreddie.com/darknet/yolo/), and convert into Caffe model using darknet2caffe (https://github.com/xingyanan/darknet2caffe) conversion tool(the conversion tool is a simplified version, you will need to modify the 'Reshape' layer in the .prototxt file into 'Passthrough' layer when using it. The modified parameters of the 'Passthrough' layer please refer to the yolov2.prototxt example, and add a Permute Operator to change output layout from NCHW to NHWC)
2. verify the md5sum of the input model and prototxt file in Horizon's model_zoo: yolov2.caffemodel 7aa7a6764401cebf58e73e72fcbd2a45 yolov2_transposed.prototxt 72e9a51c1e284e4b66e69f72ca9214c8 
3. COCO verification dataset is used for computing model accuracy. It can be downloaded from COCO official website: http://cocodataset.org/
4. Calibration dataset: extract 50 images from your COCO verification dataset to serve as calibration dataset
5. origin float model accuracy : `[IoU=0.50:0.95] 0.276 [IoU=0.50] 0.494`
