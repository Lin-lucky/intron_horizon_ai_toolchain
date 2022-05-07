# 多路视频分析盒子参考解决方案
## 一. 介绍
多路视频分析盒子方案采用地平线X3处理器，支持多达8路视频流分析。根据模型能力配置，可以实现人头、人体、人脸检测，人体骨骼关键点提取，人脸抓拍，人脸识别等多种功能。对于开发者来说，通过替换深度神经网络模型可以快速实现自己定义的功能。

多路视频盒子整体方案如下图所示
![框架](./images/video_box.png "")
X3多路视频分析盒子与前端图像采集设备如IPC通过网口进行连接。前端采集设备采集实时视频并压缩编码，X3多路视频分析盒子通过拉取图像采集设备的RTSP码流获得视频输入。

X3多路视频分析盒子内部数据流向示意图如下两图所示：
![框架](./images/video_box_data_flow.png "")
![框架](./images/video_box_flow.png "")
首先通过RTSP Client获取前端视频采集设备发送的码流，然后对码流进行解码获得NV12格式的图像，接着对该NV12进行裁剪以满足模型输入需要，对裁剪后的NV12图像做金字塔下缩放，最后将金字塔图像送给smart plugin进行智能分析得到结构化数据。整个流程涉及X3的几个功能模块，其中Decode会用到VPS模块，Crop+Pym会用到IPU模块，而smart plugin会使用到BPU模块。

RTSP Client这里以开源live555项目的testRTSPClient为基础进行开发，实现根据配置文件进行多路拉流。这里有一个地方需要注意，如果将获取的裸码流直接保存成H264或者H265文件并不能直接被播放器播放。还缺少SPS、PPS数据头，每帧之间缺少start code（0x00 0x00 0x00 0x01）。获取SPS、PPS有两种方式，一种是调用live555的"MediaSubsession::fmtp_spropparametersets()"生成SPS、PPS信息；另一种是大多数的设备在发送IDR帧之前会先发送SPS、PPS信息。无论哪种方式，start code需要自己添加。

Decode可以根据需要配置H264解码或者H265解码，也可以根据需要配置只解码I帧和P帧，或者只解码I帧以减少带宽占用减轻压力。

Crop+Pym这里为什么需要对图像进行剪裁？对于1080P分辨路的码流，Decode解码出来的图像并非1920*1080大小，而是1920*1088，而在涉及模型的时候一般认为输入的大小，是1920*1080或者960*540。为了匹配模型的输入分辨率限制这里对解码出来的图片进行剪裁和金字塔缩放。金字塔缩放比较灵活，基础层宽、高分辨率均是上一层的一半，基础层之间还有三个可配置分辨率的层。Crop+Pym的功能实现在X3的VPS模块中。

Smart是负责对图片进行处理获得结构化数据，目前每一路的视频输入都是对应一个独立的samrt处理，这意味着可以根据实际需要每一路做不同的处理方式，比如有的用来做人头、人脸、人体检测，有的用来做分割，等等。

图像处理的具体流程如下图所示：
![框架](./images/video_box_process.png "")

为了直观显示智能数据处理结果，这里将处理后的图像直接通过VOT模块输出至HDMI，将1080P分辨的显示器连接到开发板上即可看见结果。

## 二. 编译与部署
 ```
bash build_and_deploy.sh
 ```
该脚本会在当前目录下创建deploy文件夹，里面包含通用库、VIO配置文件、模型等。

## 三. 运行
将部署包拷贝到板子上，即可运行。
 ```
运行sh run_video_box.sh即可。
 ```

## 四. 配置文件说明

1.video_box_config.json

```json
{
  "xstream_workflow_file": "./video_box/configs/body_detection.json",	// xstream配置文件 
  "enable_profile": 0,
  "profile_log_path": "",
  "enable_result_to_json": false,
  "box_face_thr": 0.95,													// 人脸框阈值
  "box_head_thr": 0.95,													// 人头框阈值
  "box_body_thr": 0.95,													// 人体框阈值
  "lmk_thr": 0.0,
  "kps_thr": 0.50,
  "box_veh_thr": 0.995,
  "plot_fps": false,
  "rtsp_config_file": "./video_box/configs/rtsp.json",					// rtsp配置文件
  "display_config_file": "./video_box/configs/display.json",				// 显示配置文件
  "drop_frame_config_file": "./video_box/configs/drop_frames.json"  // 配置是否丢帧以及丢帧策略
}
```

2.rtsp.json

~~~json
{
  "channel_num": 4,                            					// 连接路数
  "channel0": {
    "rtsp_link": "rtsp://admin:admin123@10.96.35.66:554/0",   	// rtsp URL地址
    "tcp": 1,													// 是否使用TCP连接
    "frame_max_size": 200, 										// 码流最大包大小，单位为KB
    "save_stream": 0											// 是否保存当前码流到本地
  },
  "channel1": {
    "rtsp_link": "rtsp://admin:admin123@10.96.35.199:554/0",
    "tcp": 0,
    "frame_max_size": 200,
    "save_stream": 0
  },
  "channel2": {
    "rtsp_link": "rtsp://admin:admin123@10.96.35.66:554/0",
    "tcp": 1,
    "frame_max_size": 200,
    "save_stream": 0
  },
  "channel3": {
    "rtsp_link": "rtsp://admin:admin123@10.96.35.66:554/0",
    "tcp": 0,
    "frame_max_size": 200,
    "save_stream": 0
  }
}
~~~

3.display.json

```json
{
  "vo": {
    "enable": true,											// 是否开启VO输出
    "display_mode": 0, 									// 显示模式，0: auto, 1: 9 pictures
    "transition_support": 0             // 是否支持动态融合
  },
  "rtsp": {
    "stream_1080p": false,									// 是否开启1080P RTSP码流推送
    "stream_720p": false,									// 是否开启720P RTSP码流推送
    "encode_smart_info": true             // 编码码流是否包括智能信息
  }
}
```

4.visualplugin_video_box.json

```json
{
  "auth_mode": 0,								// RTSP推流是否开启密码验证功能
  "display_mode": 1,							// 设置为0开启本地转发或推流功能
  "user": "admin",								// RTSP推流用户名
  "password": "123456",							// RTSP推流密码
  "data_buf_size": 3110400,						// live555缓存区大小
  "packet_size": 102400,						// live555分包大小
  "input_h264_filename": "",					// 需要转发的码流文件名，不需要转发则置空
  "local_forward": 1							// 设置为1为本地转发
}
```

5.drop_frames.json

```json
{
    "frame_drop": {
        "drop_frame": false,      // 是否丢帧
        "interval_frames_num": 2  // 间隔多少帧丢弃一帧
    }
}
```

## 五. 常见问题

1.拉取码流出错

1) rtsp.json文件里面的url地址问题，注意检查url的用户名密码以及ip。

2) 设备支持的连接数目达到最大限制，不再支持继续接入。
   比如，某些设备rtsp的tcp方式最大支持一路，当前已使用tcp接入了一路，此时可修改rtsp.json对应通道的连接方式。


2.解码出错

1）目前接收码流buffer默认为200KB，如果码流帧大小超过200KB，会导致接收到的帧不完整，送给解码器后解码失败。
   码流帧大小的限制可以在rtsp.json文件里面修改。

2）送给解码器要是一帧完整的帧，包含start code（0x00 0x00 0x00 0x01）、SPS、PPS数据头。


3.程序启动失败

运行多路时，比如8路，程序启动失败，可能原因是默认ion Buffer设置太小，需要手动修改下，修改方法为 uboot 命令行下修改环境变量的，命令如下。

以配置 1GB ion 空间为例： 

~~~shell
setenv ion_size '1024' 
saveenv 
~~~

4.HDMI没有显示

开发板硬件上支持 HDMI 和 LCD 两种 display 方式，默认显示方式为LCD，若要HDMI显示需要手动在`uboot`下修改环境变量切换，切换命令如下。 
HDMI 方式： 

~~~shell
setenv extra_bootargs video=hobot:x3sdb-hdmi 
saveenv 
~~~


LCD 方式：

~~~shell
setenv extra_bootargs video=hobot:x3sdb-mipi720p
saveenv 
~~~

5、显示模式

目前支持单画面，4画面，9画面显示的方式。默认是根据接入的路数动态切换显示画面的个数。若需要修改显示方式，可以在配置文件修改display_mode字段。

6、本地转发与IPC转发

本地转发是指在X3侧启动RTSP Server, 本地H264文件作为转发视频源, 由video_box启动RTSP Client, 向本地Server发送请求。使用本地转发，首先将visualplugin_video_box中的local_forward置为1, 再修改input_h264_filename, 指定h264文件名, video_box提供了默认文件test.264。
然后修改rtsp.json的rtsp_link, 以默认文件为例, 本地转发的rtsp_link为rtsp://ip:555/test.264。

IPC转发是指远程IPC作为RTSP Server，video_box向其发送请求，获取视频图像。使用IPC转发时，无需配置input_h264_filename和local_forward, 只需修改rtsp.json，将rtsp_link改为指定IPC的url即完成IPC转发配置。

