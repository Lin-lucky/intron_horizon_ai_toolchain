# 多任务感知参考解决方案

## 一. 介绍
多任务感知示例，提供了人脸结构化与人体结构化的示例。
- 人脸结构化：可以检测图像中的人脸框、人脸抓拍图以及人脸特征值
- 人体结构化：可以检测图像中的人体框、人头框、人脸框、人脸关键点、人脸姿态、年龄与性别

示例支持从MIPI Camera/外接USB CAMERA或者本地图像获取图像源。


## 二. 编译与部署
 ```
  cd multitask_perception_solution
  bash build_and_deploy.sh
 ```
会生成一个deploy目录，将该目录拷贝到XJ3上，即可运行。

## 三. 运行
将deploy目录拷贝到XJ3
如下命令可以运行人脸结构化示例：

 ```
  cd deploy
  sh run_face_recog.sh
 ```

 如下命令可以运行人体结构化示例：
  ```
  cd deploy
  sh run_body.sh
 ```
运行上面两个脚本的时候，根据交互式信息，选择合适的输入源。

默认是使用PC 浏览器查看感知效果：推荐使用Chrome浏览器，输入板子IP即可访问。

 ## 四. 展示方式变更
 多任务感知示例默认使用PC浏览器方式展示。同时可以通过修改配置，改为其他模式展示：
 
 修改deploy/configs/visualplugin_face.json的display_mode字段来选择。

|display_mode|模式|
| --------   | -----:  |
|0 |rtsp server|
|1 |web|
|2 |uvc|
|3 |rtsp server|

vlc方式指的是x3会开启一个RTSP Server，基于rtsp协议转发视频[只转换视频，不传输感知结果]，通过vlc播放器，可以查看视频。

vlc方式拉流url：rtsp://ip:555/horizonStream。 

## 五. 获取序列化前智能结果
Ai-express定义了新的数据message：智能数据、控制数据、统计数据等；
* 更方便访问智能数据
* 为进程间通信定义统一的数据结构，方便序列化/反序列化
新的message订阅、发布方式保持不变，以智能数据为例增加了获取序列化前数据的样例，说明如下：

### 5.1 编译
编辑build_and_deploy.sh脚本，打开TEST_NEW_SMART_MESSAGE开关后再执行编译
```
go_build_all -DPARENT_BUILD=ON -DRELEASE_LIB=ON -DTEST_NEW_SMART_MESSAGE=ON
```

### 5.2 智能结果获取
ExamplePlugin中订阅了TYPE_SMART_LEGIBLE_MESSAGE消息，在ExamplePlugin::FeedSmart中接收序列化前的智能结果.
智能结果相关数据结构定义在: common/xproto_plugins/message/include/smart_message/smart_legible_message.h

### 5.3 跨进程发布/订阅消息
xproto::XSession::Instance().AsMaster(6688); // 使能消息可以在进程/设备间流转，其中6688是端口号
AsMaster设置当前设备为主设备，可以对外发布msg，其他进程/设备可以使用xproto订阅消息，订阅方式和进程内订阅消息的编程模式一致
xproto框架中stage4_ipc_subscriber提供了订阅示例。
