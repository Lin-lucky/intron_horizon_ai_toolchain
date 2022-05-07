# rtsp server plugin
RTSPServerPlugin启动rtsp server, 基于xproto框架，监听图像和智能化结果消息，完成数据编码和打包后，通过rtsp server发送给客户端展示。
客户端（如vlc）通过以下url建立连接，拉取数据:
rtsp://ip:555/horzionStream

# 配置

## rtsp_plugin.json
```
{
  "enable_smart": 0,  // 1：视频帧携带智能数据 0：不携带
  "video_type": 0,    // 0： H264, 1: H265
  "image_width": 1920,
  "image_height": 1080,
  "layer": 0,
  "data_buf_size": 3110400,
  "packet_size": 102400,
  "frame_buf_depth": 3,
  "use_vb":0,
  "rotation": 0,    // 0, 90, 180, 270
  "mirror": 0,      // 1: 水平镜像，2：垂直镜像 3：同时水平和垂直镜像
  "is_cbr": 1,
  "bitrate": 6000,
  "debug_dump_stream": 0,   // 是否保存编码视频到本地文件
  "debug_encode_time": 0,   // 计算编码耗时
  "rtsp_server_config": "configs/rtsp_server.json"
}
```
enable_smart为0时，发送编码后的图像数据。

enable_smart为1时，将智能数据封装为SEI数据，和图像数据同步后发出。

## rtsp_server.json
```
{
    "auth_mode": 0,
    "user": "admin",
    "password": "123456",
    "video_type": 0,  // 0:264, 1:H265
    "audio_type": 0,  // 0:g711, 1:g726
    "debug_media_source": "test.264"
  }
```
若auth_mode为1，则开启rtsp认证功能，建立rtsp的连接为:

rtsp://user:passward@ip:555/horizonStream

# 编译
mkdir build & cd build & cmake .. & make