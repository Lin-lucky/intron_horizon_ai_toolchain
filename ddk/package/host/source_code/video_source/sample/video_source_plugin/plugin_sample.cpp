/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <chrono>
#include <cstdint>
#include <ostream>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <algorithm>

#include "hobotlog/hobotlog.hpp"
#include "video_source_plugin/video_source_plugin.h"
#include "xproto/message/flowmsg.h"
#include "xproto/message/msg_registry.h"
#include "xproto/plugin/xpluginasync.h"

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;

using xproto::message::VioMessage;
using xproto::message::MultiVioMessage;
using xproto::VideoSourcePlugin;

using std::chrono::milliseconds;

#define TYPE_IMAGE_MESSAGE "XPLUGIN_IMAGE_MESSAGE"
#define TYPE_DROP_MESSAGE "XPLUGIN_DROP_MESSAGE"
#define TYPE_MULTI_IMAGE_MESSAGE "XPLUGIN_MULTI_IMAGE_MESSAGE"
#define MAX_PIPE_NUM (8)
// 视频帧消费插件
class VideoConsumerPlugin : public XPluginAsync {
 public:
  VideoConsumerPlugin() = default;
  ~VideoConsumerPlugin() = default;

  // 初始化,订阅消息
  int Init() {
    // 订阅视频帧消息
    RegisterMsg(TYPE_IMAGE_MESSAGE, std::bind(&VideoConsumerPlugin::OnGetImage,
          this, std::placeholders::_1));
    RegisterMsg(TYPE_MULTI_IMAGE_MESSAGE,
        std::bind(&VideoConsumerPlugin::OnGetMultiImage,
          this, std::placeholders::_1));

    last_timestamp_list_.resize(MAX_PIPE_NUM);
    return XPluginAsync::Init();
  }

  struct comp {
    bool operator() (
        const std::shared_ptr<VioMessage> &a,
        const std::shared_ptr<VioMessage> &b) {
      return a->time_stamp_ > b->time_stamp_; }  // descend order
  } cmp_time_stamp;

  int OnGetMultiImage(XProtoMessagePtr msg) {
    // feed multi video frame to xstreamsdk.
    auto multi_frame = std::static_pointer_cast<MultiVioMessage>(msg);
    auto multi_vio_img = multi_frame->multi_vio_img_;

    // timestamp compare
    auto multi_vio_img_sort(multi_vio_img);
    std::sort(multi_vio_img_sort.begin(), multi_vio_img_sort.end(),
        cmp_time_stamp);
    auto max_index = multi_vio_img.size() - 1;
    auto max_time_diff = multi_vio_img_sort[0]->time_stamp_ -
      multi_vio_img_sort[max_index]->time_stamp_;

    // parse multi vio message
    for (size_t i = 0; i < multi_vio_img.size(); i++) {
      auto frame = multi_vio_img[i];
      // parse valid frame from msg
      auto pym_image_list = frame->image_;
      auto channel_id = frame->channel_;
      auto timestamp = frame->time_stamp_;
      auto frame_id = frame->sequence_id_;
      VioMessage *vio_msg = frame.get();
      // get pyramid size
      int pym_image_num = pym_image_list.size();
      for (int i = 0; i < pym_image_num; i++) {
        auto pym_image = vio_msg->image_[i];
        auto origin_image_width = pym_image->img_.down_scale[0].width;
        auto origin_image_height = pym_image->img_.down_scale[0].height;
        LOGI << "channel_id: " << channel_id
          << " frame_id: " << frame_id
          << " time_stamp: " << timestamp
          << " max_time_diff: " << max_time_diff << "ms"
          << " origin_image_width: " << origin_image_width
          << " origin_image_height: " << origin_image_height;

        // dump pyramid image
        if (0 == access("./pym_output.txt", F_OK)) dump_en_ = true;
        if (dump_en_ == true) {
          DumpPyramidImage(channel_id, pym_image);
        }
      }
    }
    return 0;
  }

  int OnGetImage(XProtoMessagePtr msg) {
    // feed video frame to xstreamsdk.
    // 1. parse valid frame from msg
    auto frame = std::static_pointer_cast<VioMessage>(msg);
    auto pym_image_list = frame->image_;
    auto channel_id = frame->channel_;
    auto timestamp = frame->time_stamp_;
    auto frame_id = frame->sequence_id_;
    VioMessage *vio_msg = frame.get();
    // get pyramid size
    for (size_t i = 0; i < pym_image_list.size(); i++) {
      auto pym_image = vio_msg->image_[i];
      auto origin_image_width = pym_image->img_.down_scale[0].width;
      auto origin_image_height = pym_image->img_.down_scale[0].height;
      auto last_timestamp = last_timestamp_list_[channel_id];
      LOGI << "channel_id: " << channel_id
        << " frame_id: " << frame_id
        << " time_stamp: " << timestamp
        << " time_stamp_diff: " << timestamp - last_timestamp << "ms"
        << " origin_image_width: " << origin_image_width
        << " origin_image_height: " << origin_image_height;
      last_timestamp_list_[channel_id] = timestamp;

      if (0 == access("./pym_output.txt", F_OK)) dump_en_ = true;
      if (dump_en_ == true) {
        DumpPyramidImage(channel_id, pym_image);
      }
    }

    return 0;
  }

  std::string desc() const { return "VideoConsumerPlugin"; }

  // 启动plugin
  int Start() {
    std::cout << "video consumer plugin start" << std::endl;
    return 0;
  }

  // 停止plugin
  int Stop() {
    std::cout << "video consumer plugin stop" << std::endl;
    return 0;
  }

 private:
  void DumpPyramidImage(const int &channel_id,
    const std::shared_ptr<xstream::PyramidImageFrame> &pym_frame);

 private:
  bool dump_en_ = false;
  uint32_t dump_index_ = 0;
  std::vector<uint64_t> last_timestamp_list_;
};

void VideoConsumerPlugin::DumpPyramidImage(const int &channel_id,
    const std::shared_ptr<xstream::PyramidImageFrame> &pym_frame) {
  if (channel_id < 0 || channel_id >=8) {
    LOGE << "channel id is error, channel_id: " << channel_id;
    return;
  }
  if (pym_frame == nullptr) {
    LOGE << "pym_frame is nullptr";
    return;
  }
  for (int i = 0; i < PYRAMID_DOWN_SCALE_MAX; i++) {
    if (i % 4 != 0) continue;
    int j = i / 4;
    xstream::PyramidAddrInfo &image_info = pym_frame->img_.down_scale[i];
    auto width = image_info.width;
    auto height = image_info.height;
    auto stride = image_info.step;
    if (height <= 0 || width <= 0) {
      LOGE << "pyramid width: " << width
        << "height:" << height << " is invalid";
      return;
    }
    auto y_img_len = height * width;
    auto uv_img_len = height * width / 2;
    auto img_size = y_img_len + uv_img_len;
    char* y_addr = reinterpret_cast<char*>(image_info.y_vaddr);
    char* c_addr = reinterpret_cast<char*>(image_info.c_vaddr);
    LOGI << "DumpPyramidImage: "
      << " channel_id: " << channel_id
      << " width: " << width
      << " height:" << height
      << " stride:" << stride
      << " img_size:" << img_size
      << " y_addr: " << reinterpret_cast<void*>(y_addr)
      << " c_addr: " << reinterpret_cast<void*>(c_addr);

    auto *img_addr = reinterpret_cast<uint8_t *>(
        std::calloc(1, img_size));
    if (width == stride) {
      memcpy(img_addr, y_addr, y_img_len);
      memcpy(img_addr + y_img_len, c_addr, uv_img_len);
    } else {
      // copy y data jump over stride
      for (int i = 0; i < height; i++) {
        auto src_y_addr = y_addr + i * stride;
        auto dst_y_addr = img_addr + i * width;
        memcpy(dst_y_addr, src_y_addr, width);
      }
      // copy uv data jump over stride
      auto dst_y_size = width * height;
      for (int i = 0; i < height / 2; i++) {
        auto src_c_addr = c_addr + i * stride;
        auto dst_c_addr = img_addr + dst_y_size + i * width;
        memcpy(dst_c_addr, src_c_addr, width);
      }
    }
    std::string filename = "chn" + std::to_string(channel_id)
      + "_" + std::to_string(dump_index_)
      + "_pym_out_roi_layer_DS" + std::to_string(j)
      + "_" + std::to_string(width)
      + "_" + std::to_string(height)
      + ".yuv";
    FILE* yuvFd = fopen(filename.c_str(), "w+");
    if (yuvFd == NULL) {
      LOGE << "open file: " << filename << "failed";
      return;
    }
    fwrite(img_addr, 1, img_size, yuvFd);
    fflush(yuvFd);
    if (yuvFd) fclose(yuvFd);
    if (img_addr) free(img_addr);
    LOGD << "filedump: " << filename << " dump success...";
  }
  dump_index_++;
}

struct SmartContext {
  volatile bool exit;
  SmartContext() : exit(false) {}
};

SmartContext g_ctx;

static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    g_ctx.exit = true;
  }
}

int main(int argc, char **argv) {
  std::string config_file;
  std::string log_level;
  std::string run_mode = "normal";
  if (argc < 3) {
    std::cout << "Usage: ./sample config_file run_mode"
              << " [-i/-d/-w/-f] " << std::endl;
  }
  if (argv[1] == nullptr) {
    std::cout << "set default video source config:"
              << " [./configs/x3_video_source.json.fb]" << std::endl;
    config_file = "./configs/x3_video_source.json.fb";
  } else {
    config_file = argv[1];
  }
  if (argv[2] != nullptr) {
    run_mode.assign(argv[2]);
    if (run_mode != "ut"
        && run_mode != "normal"
        && run_mode != "loop") {
      std::cout << "not support mode: " << run_mode << std::endl;
      return 0;
    }
  }
  std::cout << "run_mode: " << run_mode << std::endl;
  if (argv[3] == nullptr) {
    std::cout << "set default log level: [-i] ";
    log_level = "-i";
  } else {
    log_level = argv[3];
  }
  if (log_level == "-i") {
    SetLogLevel(HOBOT_LOG_INFO);
  } else if (log_level == "-d") {
    SetLogLevel(HOBOT_LOG_DEBUG);
  } else if (log_level == "-w") {
    SetLogLevel(HOBOT_LOG_WARN);
  } else if (log_level == "-e") {
    SetLogLevel(HOBOT_LOG_ERROR);
  } else if (log_level == "-f") {
    SetLogLevel(HOBOT_LOG_FATAL);
  } else {
    SetLogLevel(HOBOT_LOG_INFO);
    LOGW << "set default log level: [-i] ";
  }

  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);
  signal(SIGSEGV, signal_handle);

  std::shared_ptr<std::thread> sp_consumer_frame_task_ = nullptr;
  sp_consumer_frame_task_ = std::make_shared<std::thread>(
      [&] () {
          auto consumer_plg = std::make_shared<VideoConsumerPlugin>();
          consumer_plg->Init();
          consumer_plg->Start();
          while (!g_ctx.exit) {
             std::this_thread::sleep_for(milliseconds(40));
          }
          consumer_plg->Stop();
          consumer_plg->DeInit();
         });

  auto video_source_plg = std::make_shared<VideoSourcePlugin>(config_file);
  video_source_plg->SetLoggingLevel(log_level);
  video_source_plg->Init();
  if (run_mode == "normal") {
    LOGW << "start normal mode";
    video_source_plg->Start();
    while (!g_ctx.exit) {
      std::this_thread::sleep_for(milliseconds(40));
    }
  } else if (run_mode == "loop") {
    LOGW << "start loop mode";
    while (!g_ctx.exit) {
      LOGW << "video source start\n\n";
      video_source_plg->Start();
      std::this_thread::sleep_for(std::chrono::seconds(10));
      LOGW << "video source stop\n\n";
      video_source_plg->Stop();
    }
  } else if (run_mode == "ut") {
    video_source_plg->Start();
    std::this_thread::sleep_for(std::chrono::seconds(30));
    g_ctx.exit = true;
  } else {
    LOGE << "no support run mode: " << run_mode;
    g_ctx.exit = true;
  }

  LOGI << "video source sample quit\n\n";
  if (sp_consumer_frame_task_) {
    sp_consumer_frame_task_->join();
    sp_consumer_frame_task_ = nullptr;
  }
  if (run_mode == "normal" || run_mode == "ut") {
    video_source_plg->Stop();
  }
  video_source_plg->DeInit();

  return 0;
}
