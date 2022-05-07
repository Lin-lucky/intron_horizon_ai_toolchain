# audio output plugin
该plugin的主要功能是通过alsa接口发送音频数据到扬声器播放，音频数据来自xproto数据总线，封装成AudioMessage对象。

# 编译
```shell
  mkdir build
  cd build
  cmake ..
  make
  make install
```