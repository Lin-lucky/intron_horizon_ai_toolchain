/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2021-05-14
 * @Version: v0.0.1
 * @Brief: rtsp client vin module for video source system.
 */
#include "video_source/vin/rtsp_client/rtsp_client_vin_module.h"
#include <cstddef>
#include <cstdint>
#include <vector>
#include <fstream>
#include <ios>
#include <memory>
#include <string>
#include "hobotlog/hobotlog.hpp"
#include "rtsp_client_vin_module.h"

namespace videosource {

#define VIDEO_FB_H264_AVC1 0x31637661

int RtspClientVinModule::LoadConfig(const std::string &config_file) {
  int ret = -1;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;

  ret = LoadConfigFile(config_file, json_cfg);
  if (ret) {
    LOGE << "load config file failed, ret: " << ret;
    return ret;
  }

  vin_cfg_ = std::make_shared<RtspClientVinConfig>();
  vin_cfg_->rtsp_link = json_cfg->GetSTDStringValue("rtsp_link");
  vin_cfg_->is_tcp = json_cfg->GetBoolValue("tcp");
  vin_cfg_->frame_max_size = json_cfg->GetIntValue("frame_max_size");
  vin_cfg_->cached_buf_count = json_cfg->GetIntValue("cached_buf_count");
  vin_cfg_->probe_size = json_cfg->GetSTDStringValue("probe_size");
  vin_cfg_->analyze_duration = json_cfg->GetSTDStringValue("analyze_duration");
  vin_cfg_->is_save_stream = json_cfg->GetBoolValue("save_stream");
  vin_cfg_->is_log_debug = json_cfg->GetBoolValue("log_debug");
  vin_cfg_->max_reconnect_count = json_cfg->GetIntValue("max_reconnect_count");
  vin_cfg_->max_reconnect_err_count =
    json_cfg->GetIntValue("max_reconnect_err_count");

  LOGI << "rtsp_link: " << vin_cfg_->rtsp_link;
  LOGI << "is_tcp: " << vin_cfg_->is_tcp;
  LOGI << "frame_max_size: " << vin_cfg_->frame_max_size;
  LOGI << "cached_buf_count: " << vin_cfg_->cached_buf_count;
  LOGI << "probe_size: " << vin_cfg_->probe_size;
  LOGI << "analyze_duration: " << vin_cfg_->analyze_duration;
  LOGI << "is_save_stream: " << vin_cfg_->is_save_stream;
  LOGI << "is_log_debug: " << vin_cfg_->is_log_debug;
  LOGI << "max_reconnect_count: " << vin_cfg_->max_reconnect_count;
  LOGI << "max_reconnect_err_count: " << vin_cfg_->max_reconnect_err_count;
  return 0;
}

int RtspClientVinModule::Init(const std::string &config_file) {
  LOGD << "Enter RtspClientVinModule Init...";
  int ret = -1;

  if (init_flag_ == true) {
    LOGW << "rtsp client vin module has been init!!!";
    return 0;
  }
  ret = LoadConfig(config_file);
  if (ret) {
    LOGE << "RtspClientVinModule LoadConfig failed!";
    return ret;
  }
  // set vin cached buffer count
  SetVinCachedBufCount(vin_cfg_->cached_buf_count);

  // set vin buffer config
  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  vin_buf_cfg.max_queue_len = vin_frame_depth_;
  vin_buf_cfg.use_vb = true;
  vin_buf_cfg.raw_pixel_len = 0;
  vin_buf_cfg.decode_en = true;
  this->SetVinBuf(vin_buf_cfg, false);

  init_flag_ = true;
  LOGD << "RtspClientVinModule Init success...";
  return 0;
}

int RtspClientVinModule::DeInit() {
  int ret = -1;

  if (init_flag_ == false) {
    LOGW << "feedback vin module has not init!!!";
    return 0;
  }

  ret = VinModule::VinDeInit();
  if (ret) {
    LOGE << "vin module deinit failed, ret: " << ret;
    return ret;
  }

  init_flag_ = false;
  return 0;
}

int RtspClientVinModule::Start() {
  int ret = -1;
  if (init_flag_ == false) {
    LOGW << "rtsp client vin module has not init";
    return 0;
  }

  ret = RtspClientStart();
  if (ret) {
    LOGE << "rtsp client start failed, ret: " << ret;
    return ret;
  }
  return 0;
}

int RtspClientVinModule::Stop() {
  int ret = -1;
  if (init_flag_ == false) {
    LOGW << "rtsp client vin module has not init";
    return 0;
  }

  is_running_ = false;
  ret = RtspClientStop();
  if (ret) {
    LOGE << "rtsp client stop failed, ret: " << ret;
    return ret;
  }
  ret = VinModule::VinStop();
  if (ret) {
    LOGE << "vin module stop failed, ret: " << ret;
    return ret;
  }

  return 0;
}

int RtspClientVinModule::RtspClientStart() {
  if (data_thread_) {
    LOGE << "vin get data thread has been create";
    return -1;
  }

  if (data_thread_ == nullptr) {
    data_thread_ = std::make_shared<std::thread>(
        &RtspClientVinModule::HbGetDataThread, this);
  }
  return 0;
}

int RtspClientVinModule::RtspClientStop() {
  if (data_thread_ != nullptr) {
    data_thread_->join();
    data_thread_ = nullptr;
  } else {
    LOGE << "data thread is nullptr!!!";
    return -1;
  }

  LOGI << "Quit VinDestoryDataThread, group_id: " << group_id_;
  return 0;
}

int RtspClientVinModule::InitFFMPEG() {
  int ret = -1;
  LOGD << "enter InitFFMPEG function...";
  auto start_time = std::chrono::system_clock::now();
  ffmpeg_stream_params_ = std::make_shared<FFMPEGStreamParams>();
  AVFormatContext* &pFormatCtx = ffmpeg_stream_params_->pFormatCtx;
  std::string video_path = vin_cfg_->rtsp_link;

  if (vin_cfg_->is_log_debug == true) {
    av_log_set_level(AV_LOG_DEBUG);
  }
  AVDictionary *options = NULL;
  av_dict_set(&options, "probesize", vin_cfg_->probe_size.c_str(), 0);
  av_dict_set(&options, "analyzeduration",
      vin_cfg_->analyze_duration.c_str(), 0);
  if (vin_cfg_->is_tcp == true) {
    av_dict_set(&options, "rtsp_transport", "tcp", 0);
  } else {
    av_dict_set(&options, "rtsp_transport", "udp", 0);
  }
  av_dict_set(&options, "stimeout", "2000000", 0);
  av_dict_set(&options, "max_delay", "500000", 0);

  pFormatCtx = avformat_alloc_context();
  if (!pFormatCtx) {
    LOGE << "avformat_alloc_context failed.";
    return -1;
  }

  if ((ret = avformat_open_input(
      &pFormatCtx, video_path.c_str(), NULL, &options)) < 0) {
    LOGE << "could not open rtsp link : " << video_path
         <<  " ret: " << ret;
    return -1;
  }

  if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
    LOGE << "could not find stream information: ";
    return -1;
  }

  ret = ParseVideoStreamInfo();
  if (ret) {
    LOGE << "parse video stream info failed, ret: " << ret;
    return ret;
  }
  auto curr_time = std::chrono::system_clock::now();
  auto cost_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(
        curr_time - start_time).count();
  LOGW << "ffmpeg init success, cost_time: " << cost_time << "ms";
  return 0;
}

int RtspClientVinModule::DeInitFFMPEG() {
  AVCodecContext* &pVideoDecCtx = ffmpeg_stream_params_->pVideoDecCtx;
  AVFormatContext* &pFormatCtx = ffmpeg_stream_params_->pFormatCtx;

  avcodec_free_context(&pVideoDecCtx);
  avformat_close_input(&pFormatCtx);
  ffmpeg_init_flag_ = false;
  return 0;
}

int RtspClientVinModule::ParseVideoStreamInfo() {
  int ret = -1;
  int &video_stream_idx = ffmpeg_stream_params_->video_stream_idx;
  AVFormatContext* &pFormatCtx = ffmpeg_stream_params_->pFormatCtx;
  AVCodecContext* &pVideoDecCtx = ffmpeg_stream_params_->pVideoDecCtx;
  AVStream* &pVideoStream = ffmpeg_stream_params_->pVideoStream;
  enum AVPixelFormat &PixFmt = ffmpeg_stream_params_->PixFmt;

  video_stream_idx = -1;
  ret = OpenCodecContext(&video_stream_idx, &pVideoDecCtx,
      pFormatCtx, AVMEDIA_TYPE_VIDEO);
  if (ret) {
    LOGE << "open codec context failed, ret: " << ret;
    return ret;
  }

  pVideoStream = pFormatCtx->streams[video_stream_idx];
  PixFmt = pVideoDecCtx->pix_fmt;
  image_width_ = pVideoDecCtx->width;
  image_height_ = pVideoDecCtx->height;
  if (!image_width_ || !image_height_) {
    LOGE << "image width: " << image_width_
      << " image_height: " << image_height_;
    return -1;
  }

  std::string pix_fmt = AVPixelFormatToStr(PixFmt);
  std::string video_codec_id =
    AVCodecIDToStr(pVideoStream->codecpar->codec_id);

  LOGI << "video width: " << image_width_ << " height: " << image_height_;
  LOGI << "video_stream_idx: " << video_stream_idx;
  LOGI << "PixFmt: " << pix_fmt;
  LOGI << "video_codec_id: " << video_codec_id;
  LOGI << "codec_tag: " << pVideoStream->codecpar->codec_tag;
  return 0;
}

int RtspClientVinModule::OpenCodecContext(
    int *stream_idx,
    AVCodecContext **dec_ctx, AVFormatContext *fmt_ctx,
    enum AVMediaType type) {
  int ret, stream_index;
  AVStream *st;
  AVCodec *dec = nullptr;

  ret = av_find_best_stream(
      fmt_ctx, AVMEDIA_TYPE_VIDEO,
      -1, -1, nullptr, 0);
  if (ret < 0) {
    LOGE << "failed to find the "
         << av_get_media_type_string(type)
         << " stream";
    return ret;
  } else {
    stream_index = ret;
    st = fmt_ctx->streams[stream_index];
    //  find decoder for the stream
    dec = avcodec_find_decoder(st->codecpar->codec_id);
    if (!dec) {
      LOGE << "failed to find " << av_get_media_type_string(type) << " codec";
      return AVERROR(EINVAL);
    }

    //  allocate a codec context for the decoder
    *dec_ctx = avcodec_alloc_context3(dec);
    if (!*dec_ctx) {
      LOGE << "failed to allocate the "
           << av_get_media_type_string(type)
           << " codec context";
      return AVERROR(ENOMEM);
    }

    //  copy codec parameters from input stream to putput codec context
    ret = avcodec_parameters_to_context(*dec_ctx, st->codecpar);
    if (ret < 0) {
      LOGE << "failed to copy "
           <<  av_get_media_type_string(type)
           << " codec parameters to decoder context";
      return ret;
    }

    *stream_idx = stream_index;
  }
  return 0;
}

std::string RtspClientVinModule::AVPixelFormatToStr(
    AVPixelFormat format_id) {
  std::string format_str;
  switch (format_id) {
    case AV_PIX_FMT_YUV420P:
      format_str = "AV_PIX_FMT_YUV420P";
      break;
    case AV_PIX_FMT_YUYV422:
      format_str = "AV_PIX_FMT_YUYV422";
      break;
    case AV_PIX_FMT_RGB24:
      format_str = "AV_PIX_FMT_RGB24";
      break;
    case AV_PIX_FMT_BGR24:
      format_str = "AV_PIX_FMT_BGR24";
      break;
    case AV_PIX_FMT_YUVJ420P:
      format_str = "AV_PIX_FMT_YUVJ420P";
      break;
    case AV_PIX_FMT_NV12:
      format_str = "AV_PIX_FMT_NV12";
      break;
    case AV_PIX_FMT_NV21:
      format_str = "AV_PIX_FMT_NV21";
      break;
    default:
      format_str = "Unsupport format_id: " +
        std::to_string(static_cast<int>(format_id));
  }
  return format_str;
}

std::string RtspClientVinModule::AVCodecIDToStr(AVCodecID codec_id) {
  std::string codec_str;
  switch (codec_id) {
    case AV_CODEC_ID_MJPEG:
      codec_str = "AV_CODEC_ID_MJPEG";
      buf_fmt_ = HorizonVisionPixelFormat::kHorizonVisionPixelFormatMJPEG;
      break;
    case AV_CODEC_ID_H264:
      codec_str = "AV_CODEC_ID_H264";
      buf_fmt_ = HorizonVisionPixelFormat::kHorizonVisionPixelFormatH264;
      break;
    case AV_CODEC_ID_H265:
      codec_str = "AV_CODEC_ID_H265";
      buf_fmt_ = HorizonVisionPixelFormat::kHorizonVisionPixelFormatH265;
      break;
    default:
      codec_str = "Unsupport codec_id: " +
        std::to_string(static_cast<int>(codec_id));
  }
  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  vin_buf_cfg.format = buf_fmt_;
  this->SetVinBuf(vin_buf_cfg, false);
  return codec_str;
}

int RtspClientVinModule::ConnectRtspLink() {
  int ret = -1;
  LOGD << "enter ConnectRtspLink function...";
  int max_try_time = vin_cfg_->max_reconnect_count;
  int try_time = 0;

  if (ffmpeg_init_flag_ == true) {
    DeInitFFMPEG();
  }

  do {
    ret = InitFFMPEG();
    if (ret) {
      LOGE <<  "init ffmpeg failed";
      DeInitFFMPEG();
    } else {
      LOGW << "connect rtsp link success, try_time: " << try_time
        << " max_try_limit_time: " << max_try_time;
      ffmpeg_init_flag_ = true;
      break;
    }
  } while (++try_time < max_try_time);

  if (ret) {
    LOGE << "connect rtsp link time: " << try_time
      << " exceeds max limit time: " << max_try_time;
    return ret;
  }
  return 0;
}

void RtspClientVinModule::HbGetDataThread() {
  int ret = -1;
  if (is_running_) {
    LOGW << "get data thread has start!";
    return;
  }

  is_running_ = true;
  AVFormatContext *pFormatCtx = NULL;
  int video_stream_idx = -1;
  AVPacket pFramePkt;
  uint64_t total_frame_cnt = 0;
  uint32_t gop_cnt = 0;
  int err_cnt = 0;
  int rtsp_fps = 0;

  auto start_time = std::chrono::system_clock::now();
  while (is_running_) {
    if (ffmpeg_init_flag_ == false) {
      ret = ConnectRtspLink();
      HOBOT_CHECK(ret == 0) << "init ffmpeg failed: ret: " << ret;
      av_init_packet(&pFramePkt);
      pFramePkt.data = nullptr;
      pFramePkt.size = 0;
      total_frame_cnt = 0;
      err_cnt = 0;
    }
    pFormatCtx = ffmpeg_stream_params_->pFormatCtx;
    video_stream_idx = ffmpeg_stream_params_->video_stream_idx;
    ret = av_read_frame(pFormatCtx, &pFramePkt);
    if (ret) {
      LOGE << "av read frame falied, ret: " << ret;
      err_cnt++;
      if (err_cnt == vin_cfg_->max_reconnect_err_count) {
        ffmpeg_init_flag_ = false;
      }
      continue;
    }
    LOGD << "start av read frame success...";
    total_frame_cnt++;
    gop_cnt++;
    LOGD << "av read frame count: " << total_frame_cnt;
    /* calculate rtsp fps */
    {
      rtsp_fps++;
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - start_time);
      if (cost_time.count() >= 1000) {
        LOGI << "group_id:" << group_id_ << " rtsp_fps:" << rtsp_fps;
        start_time = std::chrono::system_clock::now();
        rtsp_fps = 0;
      }
    }
    if (pFramePkt.stream_index != video_stream_idx) {
      LOGD << "detect the ignored packet stream_index : "
        << pFramePkt.stream_index
        << " video stream idx: " << video_stream_idx;
      av_packet_unref(&pFramePkt);
      continue;
    } else {
      if (pFramePkt.flags & AV_PKT_FLAG_KEY) {  // first idr frame
        LOGI << "find key frame, total_frame_count: " << total_frame_cnt
          << " gop_count: " << gop_cnt;
        gop_cnt = 1;
      }
      LOGD << "detect the video stream index: " << pFramePkt.stream_index
        << " packet pts: " << pFramePkt.pts
        << " packet data: " << reinterpret_cast<void*>(pFramePkt.data)
        << " packet size: " << pFramePkt.size;
    }
    ret = PushFrame(pFramePkt);
    if (ret) {
      LOGE << "push frame to vin module failed, ret: " << ret;
    }
    av_packet_unref(&pFramePkt);
  }

  ret = DeInitFFMPEG();
  if (ret) {
    LOGE << "deinit ffmpeg failed, ret: " << ret;
  }
  LOGI << "Quit vin get data thread, group_id: " << group_id_;
}

int RtspClientVinModule::PushFrame(AVPacket &av_frame) {
  int ret = -1;
  bool is_key_frame = false;
  // update max width and height
  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  if (vin_image_width_ != image_width_
      || vin_image_height_ != image_height_) {
    vin_image_width_ = image_width_;
    vin_image_height_ = image_height_;
    vin_buf_cfg.max_width = ALIGN(vin_image_width_, 16);
    vin_buf_cfg.max_height = ALIGN(vin_image_height_, 16);
    this->SetVinBuf(vin_buf_cfg, true);
  }

  // check frame is key or not
  if (av_frame.flags & AV_PKT_FLAG_KEY) {
    is_key_frame = true;
  }

  // Input frame data to vin module
  VinFrame vin_frame = { 0 };
  vin_frame.is_key_frame = is_key_frame;
  vin_frame.format = vin_buf_cfg.format;
  vin_frame.frame_id = frame_id_++;
  vin_frame.plane_count = 1;
  vin_frame.vaddr[0] = av_frame.data;
  vin_frame.vaddr[1] = 0;
  vin_frame.paddr[0] = 0;
  vin_frame.paddr[1] = 0;
  vin_frame.width = image_width_;
  vin_frame.height = image_height_;
  vin_frame.stride = image_width_;
  vin_frame.size = av_frame.size;
  ret = this->InputData(vin_frame);
  if (ret) {
    LOGE << "input data frame failed, ret: " << ret;
  }

  return 0;
}

}  // namespace videosource
