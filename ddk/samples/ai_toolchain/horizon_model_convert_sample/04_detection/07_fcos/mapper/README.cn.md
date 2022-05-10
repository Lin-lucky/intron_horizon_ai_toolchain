# fcos

## 准备模型和数据
1. Onnx 模型
    1.1 可以从地平线模型开源repo获取, repo 地址暂时待定
    1.2 md5sum: fcos.onnx  1321e3f5cbb7c4a521e41820174a82d5
2. COCO验证集，用于计算模型精度，可以从COCO官网下载：http://cocodataset.org/
3. 校准数据集：可以从COCO验证集中抽取50张图片作为校准数据
4. 原始浮点模型精度：`[IoU=0.50:0.95] 0.356 [IoU=0.50] 0.520`

# fcos
## Prepare Model and Data
1. Onnx Model
    1.1  model can be obtained from Horizon Open Source repo. Repo address TBD
    1.2 md5sum: fcos.onnx  1321e3f5cbb7c4a521e41820174a82d5
2. COCO verification dataset is used for computing model accuracy. It can be downloaded from COCO official website: http://cocodataset.org/
3. Calibration dataset: extract 50 images from your COCO verification dataset to serve as calibration dataset
4. origin float model accuracy : `[IoU=0.50:0.95] 0.356 [IoU=0.50] 0.520`
