#include "media_producer.h"
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <vector>

const char *video_path[] = {
  "./data/chn0.264",
  "./data/chn1.264"
};

namespace rtspcomponent {
MediaProducer::MediaProducer(std::string rtsp_config) {
    rtsp_server_ = std::make_shared<RtspServer>(rtsp_config);
    config_ = std::make_shared<RtspServerConfig>(rtsp_config);
}

MediaProducer::~MediaProducer() {}

int MediaProducer::Init() {
  auto ret = rtsp_server_->Init();
  if (ret != 0) {
    LOGE << "rtspserver init failed";
    return -1;
  }

  if (!config_->LoadConfig()) {
    LOGE << "MediaProducer config init failed";
    return -1;
  }

  return 0;
}

int MediaProducer::DeInit() {
  input_ctx_.clear();
  thread_exit_flag_.clear();
  media_source_list_.clear();
  get_frame_thread_.clear();
  return 0;
}

int MediaProducer::Start() {
  // 启动rtsp_server_
  auto ret = rtsp_server_->Start();
  if (ret != 0) {
    LOGE << "rtsp server start failed";
    return -1;
  }

  // 创建线程读取媒体文件并推送
  int video_path_size = sizeof(video_path) / sizeof(video_path[0]);
  int chn_num = config_->GetIntValue("chn_num");
  for (int i = 0; i < chn_num; i++) {
    bool exit = false;
    thread_exit_flag_.push_back(exit);
    AVFormatContext *input_ctx = nullptr;
    input_ctx_.push_back(input_ctx);
    media_source_list_.push_back(video_path[i % video_path_size]);
    std::shared_ptr<std::thread> get_frame_thread =
      std::make_shared<std::thread>(&MediaProducer::GetMediaThread, this, i);
    get_frame_thread_.push_back(get_frame_thread);
  }

  return 0;
}

int MediaProducer::Stop() {
  for (int i = 0; i < get_frame_thread_.size(); i++) {
    if (get_frame_thread_[i] && get_frame_thread_[i]->joinable()) {
      thread_exit_flag_[i] = true;
      get_frame_thread_[i]->join();
      get_frame_thread_[i] = nullptr;
    }
  }

  if (rtsp_server_) {
    rtsp_server_->Stop();
  }
  return 0;
}

int MediaProducer::GetMediaThread(int chn_id) {
  std::string media_source = media_source_list_[chn_id];
  Video_Type video_type = config_->GetChnVideoType(chn_id);
  std::string stream_name = config_->GetChnStreamName(chn_id);

  while (!thread_exit_flag_[chn_id]) {
    AVPacket packet;
    auto ret = avformat_open_input(&input_ctx_[chn_id],
              media_source.c_str(), NULL, NULL);
    if (ret != 0) {
      LOGE << "open media file failed";
      return -1;
    }
    ret = av_find_best_stream(input_ctx_[chn_id],
            AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    if (ret < 0) {
        LOGE << "av_find_best_stream failed";
        return -1;
    }

    while (!thread_exit_flag_[chn_id]) {
      ret = av_read_frame(input_ctx_[chn_id], &packet);
      if (ret < 0) {
        av_packet_unref(&packet);
        break;
      } else {
        // 向通道chn_id 转发数据
        rtsp_server_->SendData(packet.data, packet.size,
                        video_type, chn_id);

      #if 0
        std::string file_name = "dump_stream.264";
        std::fstream fout(file_name, std::ios::out |
                          std::ios::binary | std::ios::app);
        fout.write((const char *)packet.data, packet.size);
         fout.close();
      #endif
      }
        av_packet_unref(&packet);
        usleep(40000);
      }
      avformat_close_input(&input_ctx_[chn_id]);
  }
  return 0;
}
}  // namespace rtspcomponent
