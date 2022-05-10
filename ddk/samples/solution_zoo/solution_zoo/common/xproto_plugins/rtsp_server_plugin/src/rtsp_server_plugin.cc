/*
 * @Copyright 2020 Horizon Robotics, Inc.
 */
#include "rtsp_server_plugin.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "hobotlog/hobotlog.hpp"
#include "media_codec/media_codec_manager.h"
#include "smart_message/smart_message.h"
#include "utils/time_helper.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "xproto/msg_type/vio_message.h"
#include "xproto/xproto_world.h"

using hobot::Timer;
namespace xproto {
using horizon::vision::MediaCodecManager;

using xproto::message::SmartMessage;
using xproto::message::VioMessage;
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_SMART_MESSAGE)
using std::chrono::milliseconds;
bool write_flag = true;

RTSPServerPlugin::RTSPServerPlugin(const std::string& config_file) {
  config_file_ = config_file;
  LOGE << "RTSP Plugin smart config file:" << config_file_;
  smart_stop_flag_ = false;
  video_stop_flag_ = false;
  Reset();
}

RTSPServerPlugin::~RTSPServerPlugin() {
  config_ = nullptr;
}

int RTSPServerPlugin::Init() {
  LOGD << "RTSPServerPlugin::Init";
  // load config
  config_ = std::make_shared<RTSPPluginConfig>(config_file_);
  if (!config_ || !config_->LoadConfig()) {
    LOGE << "failed to load config file" << config_file_;
    return -1;
  }

  if (config_->enable_smart_) {
    RegisterMsg(GetSmartMessageType(), std::bind(&RTSPServerPlugin::FeedSmart,
                                               this, std::placeholders::_1));
  }
  RegisterMsg(TYPE_IMAGE_MESSAGE, std::bind(&RTSPServerPlugin::FeedVideo, this,
                                            std::placeholders::_1));
  // 调用父类初始化成员函数注册信息
  XPluginAsync::Init();
  video_encode_thread_.CreatThread(1);
  data_send_thread_.CreatThread(1);
  return 0;
}

int RTSPServerPlugin::Reset() {
  std::unique_lock<std::mutex> lock(map_smart_mutex_);
  while (!x3_frames_.empty()) {
    x3_frames_.pop();
  }
  while (!x3_smart_msg_.empty()) {
    x3_smart_msg_.pop();
  }
  return 0;
}

int RTSPServerPlugin::Start() {
  rtsp_server_ = std::make_shared<RtspServer>(config_->rtsp_server_config_);
  if (rtsp_server_->Init()) {
    LOGE << "Init rtsp server failed";
    return -1;
  }
  if (rtsp_server_->Start()) {
    LOGE << "Start rtsp server failed";
    return -1;
  }


  /* 1. media codec init */
  /* 1.1 get media codec manager and module init */
  MediaCodecManager &manager = MediaCodecManager::Get();
  auto rv = manager.ModuleInit();
  HOBOT_CHECK(rv == 0);
  /* 1.2 get media codec venc chn */
  chn_ = manager.GetEncodeChn();
  /* 1.3 media codec venc chn init */
  int pic_width = config_->image_width_;
  int pic_height = config_->image_height_;
  int frame_buf_depth = config_->frame_buf_depth_;
  int is_cbr = config_->is_cbr_;
  int bitrate = config_->bitrate_;

  VideoEncodeStreamParam venc_param;
  venc_param.width = config_->image_width_;
  venc_param.height = config_->image_height_;
  venc_param.frame_buf_depth = config_->frame_buf_depth_;
  venc_param.is_cbr = config_->is_cbr_;
  venc_param.bitrate = config_->bitrate_;
  venc_param.pix_fmt = HB_PIXEL_FORMAT_NV12;

  int video_type = config_->video_type_;
  int rotation = config_->rotation_;
  int mirror = config_->mirror_;

  if (video_type == H264) {
    venc_param.type = PT_H264;
  } else if (video_type == H265) {
    venc_param.type = PT_H265;
  } else {
    venc_param.type = PT_H264;
  }

  if (rotation == 0) {
    venc_param.rotation = CODEC_ROTATION_0;
  } else if (rotation == 90) {
    venc_param.rotation = CODEC_ROTATION_90;
  } else if (rotation == 180) {
    venc_param.rotation = CODEC_ROTATION_180;
  } else if (rotation == 270) {
    venc_param.rotation = CODEC_ROTATION_270;
  } else {
    venc_param.rotation = CODEC_ROTATION_0;
  }

  if (mirror == 0) {
    venc_param.mirror =  DIRECTION_NONE;
  } else if (mirror == 1) {
    venc_param.mirror = VERTICAL;
  } else if (mirror == 2) {
    venc_param.mirror = HORIZONTAL;
  } else if (mirror == 3) {
    venc_param.mirror = HOR_VER;
  } else {
    venc_param.mirror = DIRECTION_NONE;
  }

  rv = manager.EncodeChnInit(chn_, &venc_param);
  HOBOT_CHECK(rv == 0);
  /* 1.5 set media codec venc jpg chn qfactor params */
  rv = manager.EncodeChnStart(chn_);
  HOBOT_CHECK(rv == 0);
  /* 1.6 alloc media codec vb buffer init */
  if (config_->use_vb_) {
    int vb_num = frame_buf_depth;
    int pic_stride = config_->image_width_;
    int pic_size = pic_stride * pic_height * 3 / 2;  // nv12 format
    int vb_cache_enable = 1;
    rv = manager.VbBufInit(chn_, pic_width, pic_height, pic_stride,
        pic_size, vb_num, vb_cache_enable);
    HOBOT_CHECK(rv == 0);
  }

  if (!worker_ && config_->enable_smart_) {
    worker_ = std::make_shared<std::thread>(
        std::bind(&RTSPServerPlugin::MapSmartProc, this));
  }
  return 0;
}

void RTSPServerPlugin::MapSmartProc() {
  static uint64_t pre_frame_id = 0;
  while (!map_stop_) {
    std::unique_lock<std::mutex> lock(map_smart_mutex_);
    map_smart_condition_.wait(lock);
    if (x3_frames_.size() > 3 || x3_smart_msg_.size() > 3) {
      LOGW << "map_proc, frames_image size = " << x3_frames_.size()
           << ", smart_msg size = " << x3_smart_msg_.size();
    }
    if (map_stop_) {
      break;
    }
    while (!x3_smart_msg_.empty() && !x3_frames_.empty()) {
      auto msg = x3_smart_msg_.top();
      auto frame = x3_frames_.top();
      if (!config_->enable_smart_) {
        int task_num = data_send_thread_.GetTaskNum();
          if (task_num < 3) {
            data_send_thread_.PostTask(
              std::bind(&RTSPServerPlugin::SendSmartMessage, this, msg, frame));
          }
        x3_smart_msg_.pop();
        x3_frames_.pop();
      } else {
      if (msg->time_stamp_ == frame.timestamp_()) {
        if (msg->frame_id_ > pre_frame_id ||
            (pre_frame_id - msg->frame_id_ > 300) ||
            pre_frame_id == 0) {  // frame_id maybe overflow reset to 0
          int task_num = data_send_thread_.GetTaskNum();
          if (task_num < 3) {
            data_send_thread_.PostTask(
              std::bind(&RTSPServerPlugin::SendSmartMessage, this, msg, frame));
          }
          pre_frame_id = msg->frame_id_;
        }
        x3_smart_msg_.pop();
        x3_frames_.pop();
      } else {
        // avoid smart or image result lost
        while (x3_smart_msg_.size() > 20) {
          auto msg_inner = x3_smart_msg_.top();
          auto frame_inner = x3_frames_.top();
          if (msg_inner->time_stamp_ < frame_inner.timestamp_()) {
            // 消息对应的图片一直没有过来，删除消息
            x3_smart_msg_.pop();
          } else {
            break;
          }
        }
        while (x3_frames_.size() > 20) {
          auto msg_inner = x3_smart_msg_.top();
          auto frame_inner = x3_frames_.top();
          if (frame_inner.timestamp_() < msg_inner->time_stamp_) {
            // 图像对应的消息一直没有过来，删除图像
            x3_frames_.pop();
          } else {
            break;
          }
        }
      }
        break;
      }
    }
    if (x3_smart_msg_.size() > 20) {
      LOGF << "rtsp plugin has cache smart message nun > 20";
    }
    if (x3_frames_.size() > 20) {
      LOGF << "rtsp plugin has cache image nun > 20";
    }
  }
}

int RTSPServerPlugin::Stop() {
  {
    std::lock_guard<std::mutex> smart_lock(smart_mutex_);
    smart_stop_flag_ = true;
  }
  {
    std::lock_guard<std::mutex> video_lock(video_mutex_);
    video_stop_flag_ = true;
  }
  {
    if (worker_ && worker_->joinable()) {
      map_stop_ = true;
      map_smart_condition_.notify_one();
      worker_->join();
      worker_ = nullptr;
    }
  }
  LOGD << "RTSPServerPlugin::Stop()";

  /* 3. media codec deinit */
  /* 3.1 media codec chn stop */
  MediaCodecManager &manager = MediaCodecManager::Get();
  manager.EncodeChnStop(chn_);
  /* 3.2 media codec chn deinit */
  manager.EncodeChnDeInit(chn_);
  /* 3.3 media codec vb buf deinit */
  if (config_->use_vb_) {
    manager.VbBufDeInit(chn_);
  }
  /* 3.4 media codec module deinit */
  manager.ModuleDeInit();

  rtsp_server_->Stop();
  rtsp_server_->DeInit();
  return 0;
}

int RTSPServerPlugin::FeedSmart(XProtoMessagePtr msg) {
    if (!config_->enable_smart_) {
      return 0;
    }
  {
    std::lock_guard<std::mutex> smart_lock(smart_mutex_);
    if (smart_stop_flag_) {
      LOGD << "Aleardy stop, RTSPServerPlugin FeedSmart return";
      return -1;
    }
  }
  auto smart_msg = std::static_pointer_cast<SmartMessage>(msg);
  if (smart_msg) {
    {
      std::lock_guard<std::mutex> smart_lock(map_smart_mutex_);
      x3_smart_msg_.push(smart_msg);
    }
    map_smart_condition_.notify_one();
  }

  return 0;
}

int RTSPServerPlugin::SendSmartMessage(SmartMessagePtr smart_msg,
                                       x3::FrameMessage &fm) {
  std::string smart_sei;
  if (fm.mutable_img_()->width_() == 0) {
    // drop
    LOGE << "image failed";
    return XPluginErrorCode::ERROR_CODE_OK;
  }

  RTSPServerPlugin::PackSmartMsg(smart_sei, smart_msg.get(),
                origin_image_width_, origin_image_height_,
                dst_image_width_, dst_image_height_);

  std::vector<unsigned char> img_buf;
  std::vector<unsigned char> tmp;
  if (config_->enable_smart_) {
    FillSeiPacket(sei_buf_, is_annexb_, (const char*)smart_sei.c_str(),
                smart_sei.size());
    tmp.assign(sei_buf_, sei_buf_ + sei_len_);
    img_buf.assign(tmp.begin(), tmp.end());
    img_buf.insert(img_buf.end(), fm.img_().buf_().begin(),
            fm.img_().buf_().end());
    rtsp_server_->SendData(img_buf.data(), img_buf.size(), H264, 0);
    sei_len_ = 0;
  } else {
    img_buf.assign((unsigned char*)fm.img_().buf_().c_str(),
         (unsigned char*)fm.img_().buf_().c_str() + fm.img_().buf_().size());
      rtsp_server_->SendData(img_buf.data(), img_buf.size(), H264, 0);
  }

  return XPluginErrorCode::ERROR_CODE_OK;
}

int RTSPServerPlugin::FeedVideo(XProtoMessagePtr msg) {
  video_encode_thread_.PostTask(
      std::bind(&RTSPServerPlugin::EncodeStream, this, msg));
  return 0;
}

void RTSPServerPlugin::EncodeStream(XProtoMessagePtr msg) {
  int rv;
  bool bret;
  std::vector<unsigned char> img_buf;
  x3::FrameMessage x3_frame_msg;
  std::string smart_result;
  VideoEncodeSourceBuffer *frame_buf = nullptr;
  VideoEncodeSourceBuffer src_buf = { 0 };
  VideoEncodeStreamBuffer *stream_buf = nullptr;
  {
    std::lock_guard<std::mutex> video_lock(video_mutex_);
    if (video_stop_flag_) {
      LOGD << "Aleardy stop, RTSPServerPlugin Feedvideo return";
      return;
    }
  }

  LOGD << "RTSPServerPlugin Feedvideo";
  auto frame = std::dynamic_pointer_cast<VioMessage>(msg);
  VioMessage *vio_msg = frame.get();
  auto timestamp = frame->time_stamp_;
  // get pyramid size
  auto pym_image = vio_msg->image_[0];
  origin_image_width_ = pym_image->img_.down_scale[0].width;
  origin_image_height_ = pym_image->img_.down_scale[0].height;
  dst_image_width_ = pym_image->img_.down_scale[config_->layer_].width;
  dst_image_height_ = pym_image->img_.down_scale[config_->layer_].height;

  /* 2. start encode yuv to jpeg */
  /* 2.1 get media codec vb buf for store src yuv data */
  MediaCodecManager &manager = MediaCodecManager::Get();
  if (config_->use_vb_) {
    rv = manager.GetVbBuf(chn_, &frame_buf);
    HOBOT_CHECK(rv == 0);
  } else {
    frame_buf = &src_buf;
    memset(frame_buf, 0x00, sizeof(VideoEncodeSourceBuffer));
  }
  frame_buf->frame_info.pts = frame->time_stamp_;
  /* 2.2 get src yuv data */
  rv = RTSPServerPlugin::GetYUV(frame_buf,
             vio_msg, config_->layer_, config_->use_vb_);
  HOBOT_CHECK(rv == 0);

  if (0 == rv) {
    /* 2.3. encode yuv data to H264/H265 */
    auto ts0 = Timer::current_time_stamp();
    rv = manager.EncodeYuvToH26X(chn_, frame_buf, &stream_buf);
    if (config_->debug_encode_cost_ == 1) {
        auto ts1 = Timer::current_time_stamp();
        LOGW << "******Encode yuv to h264 cost: " << ts1 - ts0 << "ms";
    }
    if (rv == 0) {
        bret = true;
        auto data_ptr = stream_buf->stream_info.pstPack.vir_ptr;
        auto data_size = stream_buf->stream_info.pstPack.size;
        img_buf.assign(data_ptr, data_ptr + data_size);
        if (!config_->enable_smart_) {
          rtsp_server_->SendData((unsigned char *)img_buf.data(),
                             img_buf.size(), H264, 0);
        }
        if (config_->debug_dump_stream_) {
          std::string file_name = "dump_stream.264";
          std::fstream fout(file_name, std::ios::out |
                            std::ios::binary | std::ios::app);
          fout.write((const char *)img_buf.data(), img_buf.size());
          fout.close();
       }
    } else {
        bret = false;
        LOGE << "X3 media encode failed!";
    }
    /* 2.4 free jpg stream buf */
    if (stream_buf != nullptr) {
      rv = manager.FreeStream(chn_, stream_buf);
      HOBOT_CHECK(rv == 0);
    }
    /* 2.5 free media codec vb buf */
    if (config_->use_vb_) {
      rv = manager.FreeVbBuf(chn_, frame_buf);
      HOBOT_CHECK(rv == 0);
    }
  if (config_->enable_smart_) {
    if (bret) {
      auto image = x3_frame_msg.mutable_img_();
      x3_frame_msg.set_timestamp_(timestamp);
      image->set_buf_((const char *)img_buf.data(), img_buf.size());
      image->set_type_("h264");
      image->set_width_(dst_image_width_);
      image->set_height_(dst_image_height_);
      {
        std::lock_guard<std::mutex> lock(map_smart_mutex_);
        x3_frames_.push(x3_frame_msg);
        if (x3_frames_.size() > cache_size_)
          LOGW << "the cache is full, maybe the encode thread is slowly";
      }
      map_smart_condition_.notify_one();

    } else {
      LOGW << "encode stream failed, push empty frame to queue";
      auto image = x3_frame_msg.mutable_img_();
      x3_frame_msg.set_timestamp_(timestamp);
      image->set_width_(0);
      image->set_height_(0);
      {
        std::lock_guard<std::mutex> lock(map_smart_mutex_);
        x3_frames_.push(x3_frame_msg);
      }
      map_smart_condition_.notify_one();
    }
  }
  }
  return;
}

uint32_t RTSPServerPlugin::ReverseBytes(uint32_t value) {
  return (value & 0x000000FFU) << 24 | (value & 0x0000FF00U) << 8 |
    (value & 0x00FF0000U) >> 8 | (value & 0xFF000000U) >> 24;
}

uint32_t RTSPServerPlugin::GetSeiNaluSize(uint32_t content) {
  // SEI payload size
  uint32_t sei_payload_size = content + UUID_SIZE;
  // NALU + payload类型 + 数据长度 + 数据
  uint32_t sei_size = 1 + 1 + (sei_payload_size / 0xFF +
          (sei_payload_size % 0xFF != 0 ? 1 : 0)) + sei_payload_size;
  // 截止码
  uint32_t tail_size = 2;
  if (sei_size % 2 == 1) {
    tail_size -= 1;
  }
  sei_size += tail_size;

  return sei_size;
}

uint32_t RTSPServerPlugin::GetSeiPacketSize(uint32_t size) {
  return GetSeiNaluSize(size) + 4;
}

int RTSPServerPlugin::FillSeiPacket(unsigned char * packet, bool isAnnexb,
          const char * content, uint32_t size) {
  unsigned char * data = (unsigned char*)packet;
  unsigned int nalu_size = (unsigned int)GetSeiNaluSize(size);
  uint32_t sei_size = nalu_size;
  // 大端转小端
  nalu_size = ReverseBytes(nalu_size);

  // NALU开始码
  unsigned int * size_ptr = &nalu_size;
  if (isAnnexb) {
    memcpy(data, start_code, sizeof(unsigned int));
  } else {
    memcpy(data, size_ptr, sizeof(unsigned int));
  }

  data += sizeof(unsigned int);
  sei_len_ += sizeof(unsigned int);
  unsigned char * sei = data;
  // NAL header
  *data++ = 6;  // SEI
  sei_len_++;
  // sei payload type
  *data++ = 5;  // unregister
  sei_len_++;
  size_t sei_payload_size = size + UUID_SIZE;
  // 数据长度
  while (true) {
    *data++ = (sei_payload_size >= 0xFF ?
      0xFF : static_cast<char>(sei_payload_size));
    if (sei_payload_size < 0xFF) break;
    sei_payload_size -= 0xFF;
  }
  // UUID
  memcpy(data, uuid, UUID_SIZE);
  data += UUID_SIZE;
  sei_len_ += UUID_SIZE;
  // 数据
  memcpy(data, content, size);
  data += size;
  sei_len_ += size;
  // tail 截止对齐码
  if (sei + sei_size - data == 1) {
    *data = 0x80;
  } else if (sei + sei_size - data == 2) {
    *data++ = 0x00;
    *data++ = 0x80;
    sei_len_ += 2;
  }
  return true;
}

int RTSPServerPlugin::GetYUV(VideoEncodeSourceBuffer *frame_buf,
        VioMessage *vio_msg,
        int level, int use_vb) {
  LOGI << "visualplugin x3 mediacodec: " << __FUNCTION__;
  if (!vio_msg || vio_msg->num_ == 0)
    return -1;
  auto pym_image = vio_msg->image_[0];
  auto height = pym_image->img_.down_scale[level].height;
  auto width = pym_image->img_.down_scale[level].width;
  auto stride = pym_image->img_.down_scale[level].step;
  auto y_vaddr = pym_image->img_.down_scale[level].y_vaddr;
  auto y_paddr = pym_image->img_.down_scale[level].y_paddr;
  auto c_vaddr = pym_image->img_.down_scale[level].c_vaddr;
  auto c_paddr = pym_image->img_.down_scale[level].c_paddr;
  HOBOT_CHECK(height) << "width = " << width << ", height = " << height;
  auto img_y_size = height * stride;
  auto img_uv_size = img_y_size / 2;

  if (use_vb) {
    HOBOT_CHECK(frame_buf != nullptr);
    HOBOT_CHECK(frame_buf->frame_info.vir_ptr[0] != NULL);
    HOBOT_CHECK(frame_buf->frame_info.vir_ptr[1] != NULL);
    memcpy(frame_buf->frame_info.vir_ptr[0],
        reinterpret_cast<uint8_t*>(y_vaddr), img_y_size);
    memcpy(frame_buf->frame_info.vir_ptr[1],
        reinterpret_cast<uint8_t*>(c_vaddr), img_uv_size);
  } else {
    frame_buf->frame_info.width = width;
    frame_buf->frame_info.height = height;
    frame_buf->frame_info.stride = stride;
    frame_buf->frame_info.size = stride * height * 3 / 2;
    frame_buf->frame_info.vir_ptr[0] = reinterpret_cast<char *>(y_vaddr);
    frame_buf->frame_info.phy_ptr[0] = (uint32_t)y_paddr;
    frame_buf->frame_info.vir_ptr[1] = reinterpret_cast<char *>(c_vaddr);
    frame_buf->frame_info.phy_ptr[1] = (uint32_t)c_paddr;
    frame_buf->frame_info.pix_format = HB_PIXEL_FORMAT_NV12;
  }

#if 0  // dump yuv data
  static bool first = true;
  if (first) {
    std::fstream fout("1.yuv", std::ios::out | std::ios::binary);
    fout.write((const char *)frame_buf->frame_info.vir_ptr[0], img_y_size);
    fout.write((const char *)frame_buf->frame_info.vir_ptr[0], img_uv_size);
    fout.close();
    first = false;
  }
#endif
  return 0;
}

int RTSPServerPlugin::PackSmartMsg(std::string &data, SmartMessage *smart_msg,
                            int ori_w, int ori_h, int dst_w, int dst_h) {
  if (!smart_msg)
    return -1;
  if (smart_msg) {
    data = smart_msg->Serialize(ori_w, ori_h, dst_w, dst_h);
  }
  return 0;
}
}  // namespace xproto
