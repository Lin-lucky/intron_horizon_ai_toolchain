# roi zoom plugin
该plugin从xproto消息总线获取感知结果，解析图像数据以及检测框结果。然后基于XJ3 VPS模块对检测框进行抠图缩放，最终将扣好的图像使用VO模块，在HDMI显示器上展示。

该plugin不会向总线推送消息。

# 编译
```shell
  mkdir build
  cd build
  make
  make install
```