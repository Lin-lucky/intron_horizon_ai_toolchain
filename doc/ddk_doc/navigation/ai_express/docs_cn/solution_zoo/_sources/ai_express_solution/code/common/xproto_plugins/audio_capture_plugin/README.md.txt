# audio capture plugin
该plugin的主要功能是通过alsa接口获取x3外界的拾音器，将获得的音频信号封装成AudioMessage对象，发送到xproto消息总线。

# 编译
```shell
  mkdir build
  cd build
  make
  make install
```