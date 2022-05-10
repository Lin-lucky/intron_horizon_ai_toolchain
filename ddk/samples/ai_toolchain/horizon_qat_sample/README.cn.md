# Horizon pytorch 社区QAT示例

## 简介
该示例向用户介绍如何通过pytorch社区提供的fx graph量化训练工具, 将得到的qat模型编译为可以在地平线AI芯片上部署的模型文件

## 环境设置
  * CUDA 10.0
  * GPU < RTX 30 series
  * hbdk>=3.26.5
  * horizon_nn>=0.12.5
  * horizon_tc_ui>=1.4.0
  * torch>=1.8.1
  * torchvision>=0.9.1
  
## 示例运行
  该示例基于社区qat提供的事例, 以预先训好的 mobilenetv2 为基础, 展示了qat训练过程及qat模型转换编译得到上板模型的过程.
  在示例运行前需要获取对应数据集, 该数据集已放置在Horizon 提供的ftp服务器上方便下载: ftp://vrftp.horizon.ai//Open_Explorer/qat_dataset/imagenet/imagenet_1k.zip
  该示例提供了一键启动脚本 run.sh
  具体步骤请参考 mobilenet_example.py  
   
  
