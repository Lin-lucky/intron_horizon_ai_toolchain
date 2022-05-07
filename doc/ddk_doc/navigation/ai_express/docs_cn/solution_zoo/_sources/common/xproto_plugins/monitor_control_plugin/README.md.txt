# mc plugin

## Detail
McPlugin实现AP和CP（X3）之间的交互控制，以及X3状态管理，智能检测结果渲染到视频帧，HDMI输出控制。
McPlugin监听UvcPlugin，实现交互控制。监听VioPlugin、SmartPlugin，获取视频帧、智能帧。

## Usage
### 使用说明
**默认配置文件：**
mc_config.json、vot_config.json


**配置文件说明：**
```
1. mc_config.json
{
  "enable_auto_start": 0,                       # 是否使能自启动，不使能需要通过AP拉起X3。standalone模式需要使能。
  "enable_vot": 1,                              # 是否使能VOT（HDMI）输出，使能会将智能检测结果的渲染到视频帧并通过HDMI输出展示。
  "vot_config": "./configs/vot_config.json",    # VOT模块的配置文件路径
  "enable_dump_smart": 0,                       # 是否使能dump智能检测结果。使能会按帧输出每帧智能检测结果，格式：[frame id] [框坐标、score、框类型、id] .......
                                                # [hand lmk] [模型输出的分类结果、投票结果、经过策略结果、score] [模型输出的原始15类结果]
                                                # 如果有hand，才输出21点hand lmk。如果模型输出的手势识别分类结果不是-1（-1说明模型没有输出），才输出模型输出的原始15类结果
  "enable_append_smart": 0,                     # 是否使能渲染附加智能检测结果。使能会附加渲染手势识别原始结果等信息。
  "enable_dump_img": 0,                         # 是否使能dump原图（金字塔第0层）。使能会存储原图，并且影响智能帧率。
  "save_dump_img_path": "dump_pym0_img/",       # dump原图的保存路径
  "enable_feedback": 0,                         # 是否使能回灌。如果使能，mc plugin依次读取namelist中的图片，构建并发布图片buffer的xproto msg，vio plugin中通过硬件解码将jpeg格式图片转换成nv12格式
  "feedback_file_path": "configs/vio_hg/name_jpg.list", # 回灌namelist文件名
  "desc": "enable_dump_smart: dump smart data for badcase analysis; enable_append_smart: append more oupt for vot display, e.g. gesture raw oupt, ..."
}

2. vot_config.json
{
  "en_encoder": false,                      # 是否使能编码，使能会将视频帧做编码(h264编码)后输出存储
  "encoder_input": 0,                       # 解码器的输入源，0：输入智能结果渲染后的视频帧；1：输入原始视频帧（用于数据采集）
  "encoder_output_save_file": "draw.h264",  # 视频编码存储文件名
  "en_bgr_convert": false,                  # 是否使能渲染时图片格式转换。使能会将图片转成bgr后彩色渲染，并且会影响智能帧率
  "en_fps_draw": true,                      # 是否渲染智能fps
  "en_gesture_val_draw": false,             # 是否渲染手势识别结果数值，包括模型原始输出，投票输出和策略输出
  "en_handid_draw": false,                  # 是否渲染人手ID
  "desc": "en_encoder: input img to encoder and save output(h264); encoder_input: 0: input img with smart drawing to encoder, 1: input raw img to encoder; encoder_output_save_file: saved file name; en_bgr_convert: convert nv12 to bgr and plot on bgr img; en_fps_draw: draw fps on img"
}

```
