# USB CAMERA参考解决方案

## 一. 介绍
usb camera参考方案基于uvc+hid/rndis协议，向用户提供了通过usb拉取视频和智能数据的方法，用户可在pc端通过potplayer,amcap等第三方播放器观看视频效果，也可在android设备端通过地平线开发的app观看视频和智能结果展示。应用场景包括智慧电视等基于usb传输的多媒体设备。

目前usb camera方案支持nv12,mjepg,h264等编码格式，用户可通过播放器动态选择。

## 二. 编译
 ```
bash build_and_deploy.sh
 ```
该脚本会在当前目录下创建deploy文件夹，里面包含通用库、VIO配置文件、模型及usb_camera目录

## 三. 运行
将部署包deploy拷贝到板子上，即可运行。

```
sh run_tv_uvc.sh
```
```
sh run_body.sh
```


