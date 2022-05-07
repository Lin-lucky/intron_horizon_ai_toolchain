# Message使用说明
* 1、定义了基础message类，实际的消息可能是他们的派生类
* 2、plugin内部产生的消息，直接使用message中的消息定义或者派生
* 3、订阅消息的plugin，直接引用message中的消息定义，通过多态方式，运行时扩展

# Message介绍
- **audio_message**：用于发送、接收语音消息
- **image_message**：图像数据载体
- **strategy_message**：策略消息，比如roi message
- **transport_message**：数据传输载体，比如uvc、rtsp等
