# 参考解决方案概述
## 一. 概述
该repo提供了多任务结构化感知、视频盒子、usb camera等参考解决方案，同时提供了一个yolov3+mobilenetv2模型串联的示例程序。目录结果说明如下：

- multitask_perception_solution: 多任务结构化感知示例，内部提供编译部署以及运行的脚本。
- usb_camera_solution: usb camera示例，内部提供编译部署以及运行的脚本。
- video_box: 视频盒子的示例，内部提供编译部署以及运行的脚本。
- solution_example: 用于存放其他示例，当前提供了yolov3+mobilenetv2模型串联的示例。
- common: 存放一些通用的代码实现，如线程池、队列，以及第三方二进制库。此外xstream_methods中包含了大部分的Method实现，xproto_plugins中定义了xproto的消息以及plugin的实现。多任务结构化感知、视频盒子、usb camera等参考解决方案编译的时候会源码依赖部分xstream_methods以及xproto_plugins。
- tools: 存放了host package包[功能描述如后文]，以及web展示服务。

## 二. 编译
### 2.1 host package包安装
**需要在开发机上安装host package包**: host package包中包含系统软件库、模型预测库bpu_predict、xstream有向图计算框架、xproto消息总线框架、image_utils图像辅助处理库等。

host package包的安装方式
```shell
  cd tools/host_package
  bash install_host_package.sh
```
按照上面步骤安装后，会在开发机~/.horizon目录下存放这些依赖库的头文件与动态库，用于源码的编译。

**注意**: 若基于Open Explorer发布包，则不需要安装solution代码内部的host package，安装Open Explorer发布包中的host package即可。

### 2.2 参考解决方案编译
进入各个示例，参考内部的README.md进行编译。大部分都是运行如下一个脚本即可编译和打包
```shell
  bash build_and_deploy.sh
```

