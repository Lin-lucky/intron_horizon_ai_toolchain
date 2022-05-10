/*
 * @Copyright 2020 Horizon Robotics, Inc.
 */
#include "visual_plugin/visual_plugin.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include "hobotlog/hobotlog.hpp"
#include "utils/time_helper.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#ifdef X3_MEDIA_CODEC
#include "guvc/uvc_gadget.h"
// undef problematic defines sometimes defined by system headers (uvc_gadget.h)
#undef max
#include "media_codec/media_codec_manager.h"
#endif

using hobot::Timer;
namespace xproto {
using horizon::vision::MediaCodecManager;
const char *fifo_name = "/tmp/h264_fifo";

VisualPlugin::VisualPlugin(const std::string &config_file)
    : config_file_(config_file) {
  stop_flag_ = false;
  Reset();
}

VisualPlugin::~VisualPlugin() { config_ = nullptr; }

int VisualPlugin::Init() {
  LOGI << __FUNCTION__;
  // load config
  config_ = std::make_shared<VisualConfig>(config_file_);
  if (!config_ || !config_->LoadConfig()) {
    LOGE << "failed to load config file";
    return -1;
  }
  LOGI << "load config " << config_file_;
  // live555 init
  /*
  send_buf_ = (char*)malloc(send_buf_size_);
  if (nullptr == send_buf_) {
    LOGE << "failed to init send_buf_";
    return -1;
  }
  */

  memset(&server_param_, 0, sizeof(SERVER_PARAM_S));
  if (config_->auth_mode_ > 0) {
    server_param_.hasConnAuth = true;
    snprintf(server_param_.username, sizeof(server_param_.username), "%s",
             config_->user_.c_str());
    snprintf(server_param_.password, sizeof(server_param_.password), "%s",
             config_->password_.c_str());
  } else {
    server_param_.hasConnAuth = false;
  }
  server_param_.data_buf_size = config_->data_buf_size_;
  server_param_.packet_size = config_->packet_size_;
  snprintf(server_param_.input_filename, sizeof(server_param_.input_filename),
           "%s", config_->input_h264_filename_.c_str());

  if (!config_->local_forward_) {
    RegisterMsg(TYPE_IMAGE_MESSAGE, std::bind(&VisualPlugin::FeedVideo, this,
                                              std::placeholders::_1));
    if (!config_->vlc_support_) {
      RegisterMsg(TYPE_SMART_MESSAGE, std::bind(&VisualPlugin::FeedSmart, this,
                                                std::placeholders::_1));
    }
  }

  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();

  LOGI << __FUNCTION__;
  return 0;
}

int VisualPlugin::Start() {
  LOGI << __FUNCTION__;
  if (access(fifo_name, F_OK) == -1) {
    LOGI << "Create the fifo pipe";
    int res = mkfifo(fifo_name, 0777);
    if (res != 0) {
      LOGI << "mkdir fifo failed!!!!";
    }
  }

  server_run(&server_param_);
  usleep(500);
  if (nullptr == worker_ && !config_->local_forward_) {
    stop_flag_ = false;
    fifo_open_flag_ = false;
    worker_ = std::make_shared<std::thread>(&VisualPlugin::EncodeThread, this);
  }
  return 0;
}

int VisualPlugin::Stop() {
  LOGI << __FUNCTION__;
  if (worker_ && worker_->joinable()) {
    stop_flag_ = true;

    if (!fifo_open_flag_) {
      int fd = open(fifo_name, O_RDONLY);
      if (fd != -1) {
        LOGI << "open OK";
      }
    }
    worker_->join();
    worker_ = nullptr;
    LOGI << "VisualPlugin stop worker";
  }
  server_stop();
  return 0;
}

int VisualPlugin::Reset() {
  LOGI << __FUNCTION__ << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
  std::unique_lock<std::mutex> lock(map_mutex_);
  input_frames_.clear();
  input_frames_.reserve(cache_size_);
  // to do: reset the work thread
  return 0;
}

void VisualPlugin::PushFrame(InputType type, XProtoMessagePtr frame_msg) {
  LOGI << __FUNCTION__;
  if (nullptr == frame_msg) {
    LOGW << "VisualPlugin::PushFrame frame_msg is null";
    return;
  }
  std::unique_lock<std::mutex> lock(map_mutex_);
  VisualInput *input = new VisualInput();
  input->type = type;
  input->frame_msg = frame_msg;
  input_frames_.push_back(input);
  if (input_frames_.size() > cache_size_)
    LOGW << "the cache is full, maybe the encode thread is slowly";
  return;
}

int VisualPlugin::EncodeJPG(tjhandle handle_tj,                       // NOLINT
                            const unsigned char *yuv_buf,             // NOLINT
                            int width, int height,                    // NOLINT
                            unsigned char **jpeg_buf,                 // NOLINT
                            unsigned long *jpeg_size, int quality) {  // NOLINT
  int padding = 1;
  int subsample = TJSAMP_420;
  int flags = 0;
  return tjCompressFromYUV(handle_tj, yuv_buf, width, padding, height,
                           subsample, jpeg_buf, jpeg_size, quality, flags);
}

int VisualPlugin::EncodeThread() {
  LOGI << __FUNCTION__;
  int rv;
  bool bret;
  std::size_t i = 0;
  std::string send_data;
  cv::Mat yuv_img;
  std::vector<uchar> img_buf;
  x3::FrameMessage perception;
  std::vector<VisualInput *> frames;

  send_data.reserve(1024);
  frames.reserve(cache_size_);
  img_buf.clear();
  img_buf.reserve(500 * 1024);
  h264_sps_frame_.clear();
  h264_sps_frame_.reserve(500 * 1024);

  if (access(fifo_name, F_OK) == -1) {
    int res = mkfifo(fifo_name, 0777);
    if (res != 0) {
      LOGI << "mkdir fifo failed!!!!";
      return -1;
    }
  }

  int pipe_fd = -1;
  pipe_fd = open(fifo_name, O_WRONLY);
  if (pipe_fd != -1) {
    LOGI << "open fifo success";
    fifo_open_flag_ = true;
  }

#ifdef X3_MEDIA_CODEC
  /* 1. media codec init */
  VideoEncodeSourceBuffer *frame_buf = nullptr;
  VideoEncodeSourceBuffer src_buf = {0};
  VideoEncodeStreamBuffer *stream_buf = nullptr;
  /* 1.1 get media codec manager and module init */
  MediaCodecManager &manager = MediaCodecManager::Get();
  rv = manager.ModuleInit();
  HOBOT_CHECK(rv == 0);
  /* 1.2 get media codec venc chn */
  int chn = manager.GetEncodeChn();
  /* 1.3 media codec venc chn init */
  int pic_width = config_->image_width_;
  int pic_height = config_->image_height_;
  int frame_buf_depth = config_->frame_buf_depth_;
  int is_cbr = config_->is_cbr_;
  int bitrate = config_->bitrate_;
  if (config_->vlc_support_) {
    rv = manager.EncodeChnInit(chn, PT_H264, pic_width, pic_height,
                               frame_buf_depth, HB_PIXEL_FORMAT_NV12, is_cbr,
                               bitrate);
  } else {
    rv = manager.EncodeChnInit(chn, PT_JPEG, pic_width, pic_height,
                               frame_buf_depth, HB_PIXEL_FORMAT_NV12, is_cbr,
                               bitrate);
    rv = manager.SetUserQfactorParams(chn, config_->jpeg_quality_);
  }
  //  HOBOT_CHECK(rv == 0);
  /* 1.4 set media codec venc jpg chn qfactor params */
  HOBOT_CHECK(rv == 0);
  /* 1.5 set media codec venc jpg chn qfactor params */
  rv = manager.EncodeChnStart(chn);
  HOBOT_CHECK(rv == 0);
  if (config_->use_vb_) {
    /* 1.6 alloc media codec vb buffer init */
    int vb_num = frame_buf_depth;
    int pic_stride = config_->image_width_;
    int pic_size = pic_stride * pic_height * 3 / 2;  // nv12 format
    int vb_cache_enable = 1;
    rv = manager.VbBufInit(chn, pic_width, pic_height, pic_stride, pic_size,
                           vb_num, vb_cache_enable);
    HOBOT_CHECK(rv == 0);
  }
#endif

  while (!stop_flag_) {
    frames.clear();
    std::unique_lock<std::mutex> lock(map_mutex_);
    frames.assign(input_frames_.begin(), input_frames_.end());
    input_frames_.clear();
    lock.unlock();
    for (i = 0; i < frames.size(); ++i) {
      VisualInput *frame = frames[i];
      if (frame->type == VT_VIDEO) {
        // get yuv with target layer
        auto vframe = std::dynamic_pointer_cast<VioMessage>(frame->frame_msg);
#ifndef X3_MEDIA_CODEC
        rv = Convertor::GetYUV(yuv_img, vframe.get(), config_->layer_);
#else
        /* 2. start encode yuv to jpeg */
        /* 2.1 get media codec vb buf for store src yuv data */
        if (config_->use_vb_) {
          rv = manager.GetVbBuf(chn, &frame_buf);
          HOBOT_CHECK(rv == 0);
        } else {
          frame_buf = &src_buf;
          memset(frame_buf, 0x00, sizeof(VideoEncodeSourceBuffer));
        }
        /* 2.2 get src yuv data */
        rv = Convertor::GetYUV(frame_buf, vframe.get(), config_->layer_,
                               config_->use_vb_);
        HOBOT_CHECK(rv == 0);
        frame_buf->frame_info.pts = vframe->time_stamp_;
#endif
        if (0 == rv) {
          // jpeg encode
#ifndef X3_MEDIA_CODEC
          bret = Convertor::YUV2JPG(img_buf, yuv_img, config_->jpeg_quality_);
#else
          /* 2.3. encode yuv data to jpg */
          auto ts0 = Timer::current_time_stamp();
          rv = manager.EncodeYuvToJpg(chn, frame_buf, &stream_buf);
          if (config_->jpg_encode_time_ == 1) {
            auto ts1 = Timer::current_time_stamp();
            LOGI << "******Encode yuv to jpeg cost: " << ts1 - ts0 << "ms";
          }
          if (rv == 0) {
            bret = true;
            auto data_ptr = stream_buf->stream_info.pstPack.vir_ptr;
            auto data_size = stream_buf->stream_info.pstPack.size;
            img_buf.assign(data_ptr, data_ptr + data_size);
            /* dump jpg picture */
            if (config_->dump_jpg_num_-- > 0) {
              static int frame_id = 0;
              std::string file_name =
                  "out_stream_" + std::to_string(frame_id++) + ".jpg";
              std::fstream fout(file_name, std::ios::out | std::ios::binary);
              fout.write((const char *)data_ptr, data_size);
              fout.close();
            }
          } else {
            bret = false;
            LOGE << "X3 media codec jpeg encode failed!";
          }
#endif
#ifdef X3_MEDIA_CODEC
          /* 2.4 free jpg stream buf */
          rv = manager.FreeStream(chn, stream_buf);
          HOBOT_CHECK(rv == 0);
          /* 2.5 free media codec vb buf */
          if (config_->use_vb_) {
            rv = manager.FreeVbBuf(chn, frame_buf);
            HOBOT_CHECK(rv == 0);
          }
#endif
          if (bret) {
            perception.set_timestamp_(vframe->time_stamp_);
          }
        }
      } else {
        std::string body;
        auto sframe = std::dynamic_pointer_cast<SmartMessage>(frame->frame_msg);
        rv = Convertor::PackSmartMsg(body, sframe.get(), config_->smart_type_);
        if (0 == rv) {
          perception.set_timestamp_(sframe->time_stamp_);
        }
      }

      if (!config_->vlc_support_) {
        bret = perception.SerializeToString(&send_data);
        if (bret) {
          server_send((unsigned char *)send_data.c_str(), send_data.size(),
                      MsgSmart);
        }
      } else {
#ifdef X3_MEDIA_CODEC
        if (img_buf.size() > 4) {
          int nal_type = static_cast<int>(img_buf[4] & 0x1F);
          LOGD << "nal_type = " << nal_type;
          if (nal_type == H264_NALU_SPS) {
            if (!h264_sps_frame_.size()) {
              h264_sps_frame_.clear();
            }
            h264_sps_frame_.assign(
                (unsigned char *)img_buf.data(),
                (unsigned char *)img_buf.data() + img_buf.size());
          } else if (nal_type == H264_NALU_IDR) {
            if (h264_sps_frame_.size()) {
              write(pipe_fd, (unsigned char *)h264_sps_frame_.data(),
                    h264_sps_frame_.size());
            }
          }
        }
        write(pipe_fd, (unsigned char *)img_buf.data(), img_buf.size());
#endif
      }
      LOGI << "visualplugin begin delete one frame";
      delete frame;
    }
    frames.clear();
  }
#ifdef X3_MEDIA_CODEC
  /* 3. media codec deinit */
  /* 3.1 media codec chn stop */
  rv = manager.EncodeChnStop(chn);
  /* 3.2 media codec chn deinit */
  rv = manager.EncodeChnDeInit(chn);
  /* 3.3 media codec vb buf deinit */
  if (config_->use_vb_) {
    rv = manager.VbBufDeInit(chn);
  }
  /* 3.4 media codec module deinit */
  rv = manager.ModuleDeInit();
#endif
  return 0;
}

int VisualPlugin::FeedVideo(XProtoMessagePtr msg) {
  LOGI << __FUNCTION__;
  if (fifo_open_flag_) PushFrame(VT_VIDEO, msg);
  return 0;
}

int VisualPlugin::FeedSmart(XProtoMessagePtr msg) {
  LOGI << __FUNCTION__;
  PushFrame(VT_SMART, msg);
  return 0;
}
}  // namespace xproto
