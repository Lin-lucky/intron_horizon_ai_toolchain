/**
 * Copyright (c) 2021, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2021-04-25
 * @Version: v0.0.1
 * @Brief: feedback vin module for video source system.
 */
#include "video_source/vin/feedback/feedback_vin_module.h"
#include <cerrno>
#include <cstdint>
#include <ios>
#include <unistd.h>
#include <fcntl.h>
#include <memory>
#include <string>
#include <fstream>
#include "feedback_vin_module.h"
#include "hobotlog/hobotlog.hpp"

namespace videosource {

#define VIDEO_FB_H264_AVC1 0x31637661

int FeedbackVinModule::LoadConfig(const std::string &config_file) {
  int ret = -1;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;

  ret = LoadConfigFile(config_file, json_cfg);
  if (ret) {
    LOGE << "load config file failed, ret: " << ret;
    return ret;
  }

  vin_cfg_ = std::make_shared<FeedbackVinConfig>();
  vin_cfg_->feedback_type = json_cfg->GetSTDStringValue("feedback_type");
  vin_cfg_->file_list_path = json_cfg->GetSTDStringValue("file_list_path");
  vin_cfg_->cached_buf_count = json_cfg->GetIntValue("cached_buf_count");
  vin_cfg_->file_list_loop = json_cfg->GetBoolValue("file_list_loop");
  vin_cfg_->image_interval = json_cfg->GetIntValue("image_interval");
  vin_cfg_->width = json_cfg->GetIntValue("width");
  vin_cfg_->height = json_cfg->GetIntValue("height");

  LOGI << "feedback_type: " << vin_cfg_->feedback_type;
  LOGI << "file_list_path: " << vin_cfg_->file_list_path;
  LOGI << "cached_buf_count: " << vin_cfg_->cached_buf_count;
  LOGI << "file_list_loop: " << vin_cfg_->file_list_loop;
  if (vin_cfg_->feedback_type != "video_feedback") {
    LOGI << "image_interval: " << vin_cfg_->image_interval;
    LOGI << "width: " << vin_cfg_->width;
    LOGI << "height: " << vin_cfg_->height;
  }
  return 0;
}

int FeedbackVinModule::ImageSourceInit() {
  VinBufferConfig vin_buf_cfg = { 0 };
  vin_buf_cfg.max_queue_len = vin_frame_depth_;
  vin_buf_cfg.use_vb = true;
  vin_buf_cfg.raw_pixel_len = 0;
  this->SetVinBuf(vin_buf_cfg, false);

  init_flag_ = true;
  return 0;
}

int FeedbackVinModule::Init(const std::string &config_file) {
  int ret = -1;

  // VinModule::VinInit() is dynamic load in vin module
  if (init_flag_ == true) {
    LOGW << "feedback vin module has been init!!!";
    return 0;
  }
  // image source init
  if (channel_id_ == -1) {
    ret = ImageSourceInit();
    if (ret) {
      LOGE << "image source init failed, ret: " << ret;
      return ret;
    }
    return 0;
  }

  ret = LoadConfig(config_file);
  if (ret) {
    LOGE << "FeedbackCamVinModule LoadConfig failed!";
    return ret;
  }
  // set vin cached buffer count
  SetVinCachedBufCount(vin_cfg_->cached_buf_count);

  VinBufferConfig vin_buf_cfg = { 0 };
  vin_buf_cfg.max_queue_len = vin_frame_depth_;
  vin_buf_cfg.use_vb = true;
  vin_buf_cfg.raw_pixel_len = 0;

  if (vin_cfg_->feedback_type == "jpeg_image_list") {
    vin_buf_cfg.decode_en = true;
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG;
  } else if (vin_cfg_->feedback_type == "nv12_image_list") {
    vin_buf_cfg.decode_en = false;
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12;
  } else if (vin_cfg_->feedback_type == "video_feedback") {
    vin_buf_cfg.decode_en = true;
    // default pixel format
    vin_buf_cfg.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatH264;
  } else {
    LOGE << "UnSupport feedback_type: " << vin_cfg_->feedback_type;
    return -1;
  }
  this->SetVinBuf(vin_buf_cfg, false);

  init_flag_ = true;
  return 0;
}

int FeedbackVinModule::DeInit() {
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

int FeedbackVinModule::Start() {
  int ret = -1;

  // VinModule::VinStart() is dynamic load in vin module
  if (channel_id_ >= 0) {
    ret = VinCreateDataThread();
    if (ret) {
      LOGE << "FeedbackVinModule create data thread failed!!!";
      return ret;
    }
  }
  return 0;
}

int FeedbackVinModule::Stop() {
  int ret = -1;

  if (channel_id_ >= 0) {
    is_running_ = false;
    ret = VinDestoryDataThread();
    if (ret) {
      LOGE << "FeedbackVinModule destroy data thread failed, channel_id: "
        << channel_id_;
      return ret;
    }
  }
  ret = VinModule::VinStop();
  if (ret) {
    LOGE << "vin module stop failed, ret: " << ret;
    return ret;
  }
  return 0;
}

int FeedbackVinModule::VinCreateDataThread() {
  LOGI << "Enter VinCreateDataThread, channel_id: " << channel_id_;

  if (data_thread_) {
    LOGW << "vin get data thread has been create";
    return 0;
  }

  if (vin_cfg_->feedback_type == "jpeg_image_list"
      || vin_cfg_->feedback_type == "nv12_image_list") {
    data_thread_ = std::make_shared<std::thread>(
        &FeedbackVinModule::ImageListDataThread, this);
  } else if (vin_cfg_->feedback_type == "video_feedback") {
    data_thread_ = std::make_shared<std::thread>(
        &FeedbackVinModule::VideoDataThread, this);
  } else {
    LOGE << "UnSupport feedback_type: " << vin_cfg_->feedback_type;
    return -1;
  }
  return 0;
}

int FeedbackVinModule::VinDestoryDataThread() {
  if (data_thread_ != nullptr) {
    data_thread_->join();
    data_thread_ = nullptr;
  } else {
    LOGE << "data thread is nullptr!!!";
    return -1;
  }

  LOGI << "Quit VinDestoryDataThread, channel_id: " << channel_id_;
  return 0;
}

int FeedbackVinModule::ParseImageListFile() {
  std::string image_path;
  std::string image_list_file = vin_cfg_->file_list_path;

  std::ifstream ifs(image_list_file);
  if (!ifs.good()) {
    LOGF << "Open file failed: " << image_list_file;
    return -1;
  }

  while (std::getline(ifs, image_path)) {
    // trim the spaces in the beginning
    image_path.erase(0, image_path.find_first_not_of(' '));
    // trim the spaces in the end
    image_path.erase(image_path.find_last_not_of(' ') + 1, std::string::npos);

    image_source_list_.emplace_back(image_path);
  }
  all_img_count_ = image_source_list_.size();
  ifs.close();

  LOGI << "Finish importing image_num:" << all_img_count_;
  return 0;
}

int FeedbackVinModule::FillVIOImageByImagePath(
    const std::string &image_name) {
  int ret = -1;
  int len, y_img_len, uv_img_len;
  char *data = nullptr;
  std::string fb_type = vin_cfg_->feedback_type;

  if (access(image_name.c_str(), F_OK) != 0) {
    LOGE << "File not exist: " << image_name;
    return -1;
  }
  std::ifstream ifs(image_name, std::ios::in | std::ios::binary);
  if (!ifs) {
    LOGE << "Failed load " << image_name;
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  data = new char[len];
  ifs.read(data, len);

  // update max width and height
  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  if (vin_image_width_ != vin_cfg_->width
      || vin_image_height_ != vin_cfg_->height) {
    vin_image_width_ = vin_cfg_->width;
    vin_image_height_ = vin_cfg_->height;
    vin_buf_cfg.max_width = ALIGN(vin_image_width_, 16);
    vin_buf_cfg.max_height = ALIGN(vin_image_height_, 16);
    this->SetVinBuf(vin_buf_cfg, true);
  }

  if (fb_type == "jpeg_image_list") {
    VinFrame frame = { 0 };
    frame.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG;
    frame.frame_id = ++frame_id_;
    frame.plane_count = 1;
    frame.vaddr[0] = data;
    frame.paddr[0] = 0;
    frame.paddr[1] = 0;
    frame.width = vin_cfg_->width;
    frame.height = vin_cfg_->height;
    frame.stride = vin_cfg_->width;
    frame.size = len;
    ret = InputData(frame);
    if (ret) {
      LOGE << "input data frame failed, ret: " << ret;
    }
  } else if (fb_type == "nv12_image_list") {
    y_img_len = len / 3 * 2;
    uv_img_len = len / 3;
    HOBOT_CHECK(len == (y_img_len + uv_img_len));

    VinFrame frame = { 0 };
    frame.format =
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12;
    frame.frame_id = ++frame_id_;
    frame.plane_count = 2;
    frame.vaddr[0] = data;
    frame.vaddr[1] = data + y_img_len;
    frame.paddr[0] = 0;
    frame.paddr[1] = 0;
    frame.width = vin_cfg_->width;
    frame.height = vin_cfg_->height;
    frame.stride = vin_cfg_->width;
    frame.size = len;
    ret = InputData(frame);
    if (ret) {
      LOGE << "input data frame failed, ret: " << ret;
    }
  } else {
    LOGF << "Don't support fb type: " << fb_type;
    ret = -1;
  }
  delete[] data;
  ifs.close();

  LOGD << "image_list feedback send frame_id: " << frame_id_;
  return ret;
}

void FeedbackVinModule::ImageListDataThread() {
  int ret = -1;
  uint32_t img_num = 0;
  bool name_list_loop = vin_cfg_->file_list_loop;
  int interval_ms = vin_cfg_->image_interval;
  int max_vin_queue_len = 1;
  std::string image_name;

  ret = ParseImageListFile();
  HOBOT_CHECK(ret == 0) << "parse image list file error!!!";

  if (is_running_) {
    LOGW << "image list data thread has start!";
    return;
  }
  LOGI << "Enter image list Data Thread, channel_id: " << channel_id_
    << " feedback_type: " << vin_cfg_->feedback_type
    << " name_list_loop: " << name_list_loop
    << " interval_ms: " << interval_ms;

  is_running_ = true;
  while (is_running_) {
    // interval ms
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    /**
     * Attention:
     * If encode size is greater then max_vin_queue_len, stream will be stop
     * right now which send to vin module for decoding int vin sync mode.
     */
    if ((vin_sync_state_ == true) &&
        (send_stream_num_ >= max_vin_queue_len)) {
        LOGI << "video source stream can not send,"
        << " send_stream_num: " << send_stream_num_
        << " max_vin_queue_len: " << max_vin_queue_len;
      continue;
    }
    if (img_num >= all_img_count_) {
      if (name_list_loop == true) {
        img_num = 0;
      } else {
        is_running_ = false;
        if (manager_cb_func_) {
          is_source_stop_ = true;
          manager_cb_func_(is_source_stop_);
        }
      }
      continue;
    }
    image_name = image_source_list_[img_num++];
    ret = FillVIOImageByImagePath(image_name);
    if (ret) {
      LOGE << "file vio image by image path failed, ret: " << ret;
      continue;
    }
  }

  is_running_ = false;
  LOGI << "Quit image list data thread, channel_id: " << channel_id_;
}

int FeedbackVinModule::InitFFMPEG() {
  int ret = -1;
  ffmpeg_stream_params_ = std::make_shared<FFMPEGStreamParams>();
  int &video_stream_idx = ffmpeg_stream_params_->video_stream_idx;
  AVCodecContext* &pVideoDecCtx = ffmpeg_stream_params_->pVideoDecCtx;
  AVFormatContext* &pFormatCtx = ffmpeg_stream_params_->pFormatCtx;
  AVFrame* &pFrame = ffmpeg_stream_params_->pFrame;
  AVStream* &pVideoStream = ffmpeg_stream_params_->pVideoStream;
  AVBSFContext* &absCtx = ffmpeg_stream_params_->absCtx;
  enum AVPixelFormat &PixFmt = ffmpeg_stream_params_->PixFmt;
  bool &is_annexb = ffmpeg_stream_params_->is_annexb;

  const AVBitStreamFilter *absFilter = nullptr;
  std::string video_path = vin_cfg_->file_list_path;

  av_log_set_level(AV_LOG_DEBUG);
  pFormatCtx = avformat_alloc_context();
  if (!pFormatCtx) {
    LOGE << "avformat_alloc_context failed.";
    return -1;
  }

  if ((ret = avformat_open_input(
      &pFormatCtx, video_path.c_str(), NULL, NULL)) < 0) {
    LOGE << "could not open video path: " << video_path
         <<  " ret: " << ret;
    return -1;
  }

  if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
    LOGE << "could not find stream information: " << video_path;
    return -1;
  }

  pFrame = av_frame_alloc();
  if (!pFrame) {
    LOGE << "av_frame_alloc failed.";
    return -1;
  }

  ret = OpenCodecContext(&video_stream_idx, &pVideoDecCtx,
      pFormatCtx, AVMEDIA_TYPE_VIDEO);
  if (ret) {
    LOGE << "open codec context failed, ret: " << ret;
    return ret;
  }

  pVideoStream = pFormatCtx->streams[video_stream_idx];
  PixFmt = pVideoDecCtx->pix_fmt;
  is_annexb = (pVideoStream->codecpar->codec_tag == VIDEO_FB_H264_AVC1);
  image_width_ = pVideoDecCtx->width;
  image_height_ = pVideoDecCtx->height;
  HOBOT_CHECK(image_width_ > 0);
  HOBOT_CHECK(image_height_ > 0);

  if (is_annexb) {
    absFilter = av_bsf_get_by_name("h264_mp4toannexb");
    av_bsf_alloc(absFilter, &absCtx);
    avcodec_parameters_copy(absCtx->par_in, pVideoStream->codecpar);
    av_bsf_init(absCtx);
  }
  std::string pix_fmt = AVPixelFormatToStr(PixFmt);
  std::string video_codec_id =
    AVCodecIDToStr(pVideoStream->codecpar->codec_id);

  LOGI << "video width: " << image_width_;
  LOGI << "video height: " << image_height_;
  LOGI << "video_stream_idx: " << video_stream_idx;
  LOGI << "video_pix_fmt: " << pix_fmt;
  LOGI << "video_codec_id: " << video_codec_id;
  LOGI << "codec_tag: " << pVideoStream->codecpar->codec_tag;
  LOGI << "format_context: " << pFormatCtx;
  LOGI << "is_annexb: " << is_annexb;
  LOGI << "video_stream: " << pVideoStream;
  return 0;
}

int FeedbackVinModule::DeInitFFMPEG() {
  HOBOT_CHECK(ffmpeg_stream_params_);
  AVBSFContext* absCtx = ffmpeg_stream_params_->absCtx;
  AVCodecContext* pVideoDecCtx = ffmpeg_stream_params_->pVideoDecCtx;
  AVFormatContext* pFormatCtx = ffmpeg_stream_params_->pFormatCtx;
  AVFrame* pFrame = ffmpeg_stream_params_->pFrame;
  bool is_annexb = ffmpeg_stream_params_->is_annexb;

  if (is_annexb) {
    if (absCtx) av_bsf_free(&absCtx);
  }
  if (pVideoDecCtx) avcodec_free_context(&pVideoDecCtx);
  if (pFormatCtx) avformat_close_input(&pFormatCtx);
  if (pFrame) av_frame_free(&pFrame);
  return 0;
}

int FeedbackVinModule::OpenCodecContext(
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

std::string FeedbackVinModule::AVPixelFormatToStr(
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

std::string FeedbackVinModule::AVCodecIDToStr(AVCodecID codec_id) {
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

void FeedbackVinModule::VideoDataThread() {
  int ret = -1;
  bool is_repeat = true;
  int max_vin_queue_len = vin_frame_depth_;

  if (is_running_) {
    LOGW << "image list data thread has start!";
    return;
  }

  while (is_repeat) {
    is_repeat = false;

    ret = InitFFMPEG();
    HOBOT_CHECK(ret == 0) << "init ffmpeg failed: ret: " << ret;

    AVFormatContext* pFormatCtx = ffmpeg_stream_params_->pFormatCtx;
    AVStream* pVideoStream = ffmpeg_stream_params_->pVideoStream;
    int video_stream_idx = ffmpeg_stream_params_->video_stream_idx;
    bool is_annexb = ffmpeg_stream_params_->is_annexb;
    AVBSFContext* absCtx = ffmpeg_stream_params_->absCtx;
    auto frame_rate = pFormatCtx->streams[video_stream_idx]->avg_frame_rate;
    int frame_fps = frame_rate.num / frame_rate.den;
    int interval_ms = 1000 / frame_fps;

    LOGI << "Enter image list Data Thread, channel_id: " << channel_id_
      << " feedback_type: " << vin_cfg_->feedback_type
      << " file_list_loop: " << vin_cfg_->file_list_loop
      << " feedback_fps: " << frame_fps
      << " interval: " << interval_ms << "ms"
      << " video_stream: " << pVideoStream
      << " format_context: " << pFormatCtx;

    AVPacket pFramePkt;
    av_init_packet(&pFramePkt);
    pFramePkt.data = nullptr;
    pFramePkt.size = 0;

    is_running_ = true;
    while (is_running_) {
      /**
       * Attention:
       * If encode size is greater then max_vin_queue_len, stream will be stop
       * right now which send to vin module for decoding int vin sync mode.
       */
      if ((vin_sync_state_ == true) &&
          (send_stream_num_ >= max_vin_queue_len)) {
        LOGI << "video source stream can not send,"
          << " send_stream_num: " << send_stream_num_
          << " max_vin_queue_len: " << max_vin_queue_len;
        continue;
      }
      ret = av_read_frame(pFormatCtx, &pFramePkt);
      if (ret) {
        // check read frame is end of file
        if (ret == AVERROR_EOF) {
          LOGW << "av_read_frame return eof,"
            << " is_sync_mode: " << vin_sync_state_
            << " file_list_loop: " << vin_cfg_->file_list_loop;
          is_running_ = false;
          if (vin_cfg_->file_list_loop == true) {
            is_repeat = true;
          } else {
            if (manager_cb_func_) {
              is_source_stop_ = true;
              manager_cb_func_(is_source_stop_);
            }
          }
#if 0
            // seek video to start position
            ret = av_seek_frame(pFormatCtx, video_stream_idx,
                0, AVSEEK_FLAG_BACKWARD);
            if (ret) {
              LOGE << "av_seek_frame error, ret: " << ret;
            }
#endif
        } else {
          LOGE << "av read frame falied, ret: " << ret;
        }
        av_packet_unref(&pFramePkt);
        continue;
      }
      if (pFramePkt.stream_index != video_stream_idx) {
        LOGD << "detect the ignored packet stream_index : "
          << pFramePkt.stream_index
          << " video stream idx: " << video_stream_idx;
        av_packet_unref(&pFramePkt);
        continue;
      } else {
        LOGD << "detect the video stream index: " << pFramePkt.stream_index
          << " packet pts: " << pFramePkt.pts
          << " packet data: " << reinterpret_cast<void*>(pFramePkt.data)
          << " packet size: " << pFramePkt.size;
      }
      if (is_annexb) {
        av_bsf_send_packet(absCtx, &pFramePkt);
        av_bsf_receive_packet(absCtx, &pFramePkt);
      }
      ret = PushFrame(pFramePkt);
      if (ret) {
        LOGE << "push frame to vin module failed, ret: " << ret;
        av_packet_unref(&pFramePkt);
        continue;
      }
      av_packet_unref(&pFramePkt);
      // interval ms
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }  // quit is_running while
    ret = DeInitFFMPEG();
    if (ret) {
      LOGE << "deinit ffmpeg failed, ret: " << ret;
    }
    is_running_ = false;
  }
  LOGI << "Quit video data thread, channel_id: " << channel_id_;
}

int FeedbackVinModule::PushFrame(AVPacket &av_frame) {
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
  vin_frame.format = buf_fmt_;
  vin_frame.frame_id = ++frame_id_;
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
  LOGD << "video feedback send frame_id: " << frame_id_
    << " stream_size: " << av_frame.size;
  return 0;
}

int FeedbackVinModule::ReadImage(char* data, const uint32_t &len,
    const uint32_t &width, const uint32_t &height,
    const HorizonVisionPixelFormat &pixel_format) {
  int ret = -1;
  int image_width = width;
  int image_height = height;

  /* 1. update vin buffer pixel format and decode status */
  VinBufferConfig vin_buf_cfg = { 0 };
  this->GetVinBuf(vin_buf_cfg);
  vin_buf_cfg.format = pixel_format;
  if (pixel_format == \
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatJPEG) {
    vin_buf_cfg.decode_en = true;
  } else if (pixel_format == \
      HorizonVisionPixelFormat::kHorizonVisionPixelFormatNV12) {
    vin_buf_cfg.decode_en = false;
  } else {
    LOGE << "UnSupport pixel format:" << pixel_format;
    return -1;
  }
  this->SetVinBuf(vin_buf_cfg, false);

  /* 2. update vin buffer size, need reinit vinmodule */
  if (vin_image_width_ != image_width
      || vin_image_height_ != image_height) {
    vin_image_width_ = image_width;
    vin_image_height_ = image_height;
    vin_buf_cfg.max_width = ALIGN(vin_image_width_, 16);
    vin_buf_cfg.max_height = ALIGN(vin_image_height_, 16);
    this->SetVinBuf(vin_buf_cfg, true);
  }

  /* 3. input frame data to vin module */
  VinFrame frame = { 0 };
  frame.format = pixel_format;
  frame.frame_id = ++frame_id_;
  frame.plane_count = 1;
  frame.vaddr[0] = data;
  frame.vaddr[1] = 0;
  frame.paddr[0] = 0;
  frame.paddr[1] = 0;
  frame.width = width;
  frame.height = height;
  frame.stride = width;
  frame.size = len;

  ret = InputData(frame);
  if (ret) {
    LOGE << "input data frame failed, ret: " << ret;
    return ret;
  }

  return 0;
}

}  // namespace videosource
