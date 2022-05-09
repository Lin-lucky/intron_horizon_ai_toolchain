/**
 *  Copyright (C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *  @Author: yong.wu
 *  @Email: <yong.wu@horizon.ai>
 *  @Date: 2021-04-08
 *  @Version: v0.0.1
 *  @Brief: implemenation of video input system.
 */

#include "video_source/vin/vin_module.h"
#include <cstddef>
#include <stdlib.h>
#include <assert.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ios>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <vector>
#include <utility>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include "hobotlog/hobotlog.hpp"
#include "video_source/decode/decode_manager.h"
#include "video_source/video_buffer/buffer_manager.h"
#include "utils/blocking_queue.h"
#include "vin_module.h"

namespace videosource {

int VinModule::LoadConfigFile(
    const std::string &input_config_file,
    std::shared_ptr<JsonConfigWrapper> &output_json_cfg) {
  LOGI << "Load vin module config file:" << input_config_file;
  std::ifstream ifs(input_config_file.c_str());
  if (!ifs) {
    LOGE << "Open config file " << input_config_file << " failed";
    return -1;
  }

  Json::Value cfg_jv;
  std::shared_ptr<JsonConfigWrapper> json_cfg = nullptr;
  ifs >> cfg_jv;
  json_cfg.reset(new JsonConfigWrapper(cfg_jv));
  output_json_cfg = json_cfg;
  return 0;
}

int VinModule::TimeoutWait(sem_t &sem, const uint64_t &timeout_ms) {
  int32_t ret = -1;
  uint64_t end_time = 0;
  struct timeval currtime;
  struct timespec ts;

  if (timeout_ms != 0) {
    gettimeofday(&currtime, NULL);
    end_time = (((((uint64_t) currtime.tv_sec) * TIME_MICRO_SECOND) +
          currtime.tv_usec) + (timeout_ms * 1000));
    ts.tv_sec = (end_time / TIME_MICRO_SECOND);
    ts.tv_nsec = ((end_time % TIME_MICRO_SECOND) * 1000);
    ret = sem_timedwait(&sem, &ts);
  } else {
    ret = sem_wait(&sem);
  }

  return ret;
}

int VinModule::VinInit() {
  int ret = -1;
  LOGI << "enter VinModule VinInit...";

  if (vin_init_flag_ == true) {
    LOGW << "vin module has been init";
    return 0;
  }

  ret = BufferQueueInit();
  if (ret) {
    LOGE << "buffer queue init failed, ret: " << ret;
    return ret;
  }
  bool decode_en = vin_buf_cfg_.decode_en;
  if (decode_en == true) {
    ret = DecodeInit();
    if (ret) {
      LOGE << "decode init failed, ret: " << ret;
      return ret;
    }
  }
  if (vin_sync_state_ == true) {
    sem_init(&nv12_free_queue_sem_, 0, 0);
    sem_init(&send_sync_sem_, 0, 0);
  }
  vin_init_flag_ = true;
  LOGI << "vin module init success,"
    <<  " decode_en: " << decode_en
    <<  " vin_sync_state: " << vin_sync_state_;
  return 0;
}

void VinModule::VinReset() {
  memset(&vin_buf_cfg_, 0x0, sizeof(VinBufferConfig));
  frame_id_ = 0;
  vin_image_width_ = 0;
  vin_image_height_ = 0;
  input_dump_count_ = 0;
  output_dump_count_ = 0;
  update_flag_ = false;
  vin_sync_state_ = false;
  check_first_idr_ = false;
  vin_start_flag_ = false;
  encode_queue_.clear();
  raw_queue_.clear();
}

int VinModule::VinDeInit() {
  int ret = -1;

  if (vin_init_flag_ == false) {
    LOGW << "vin module has not init!";
    return 0;
  }

  bool decode_en = vin_buf_cfg_.decode_en;
  if (decode_en == true) {
    ret = DecodeDeInit();
    if (ret) {
      LOGE << "decode deinit failed, ret: " << ret;
      return ret;
    }
  }

  ret = BufferQueueDeInit();
  if (ret) {
    LOGE << "buffer queue deinit failed, ret: " << ret;
    return ret;
  }
  if (vin_sync_state_ == true) {
    sem_destroy(&nv12_free_queue_sem_);
    sem_destroy(&send_sync_sem_);
  }
  VinReset();
  vin_init_flag_ = false;
  LOGI << "VinDeInit success...";
  return 0;
}

int VinModule::VinStart() {
  int ret = -1;

  if (vin_start_flag_ == true) {
    LOGW << "vin module has been start!";
    return 0;
  }

  bool decode_en = vin_buf_cfg_.decode_en;
  if (decode_en == true) {
    ret = DecodeDataToNv12Thread();
    if (ret) {
      LOGE << "decode data to nv12 thread failed, ret: " << ret;
      return ret;
    }
  } else {
    ret = RawDataToNv12Thread();
    if (ret) {
      LOGE << "raw data to nv12 thread failed, ret: " << ret;
      return ret;
    }
  }
  {
    std::unique_lock<std::mutex> lock(vin_mutex_);
    vin_start_flag_ = true;
    vin_condition_.notify_all();
  }
  start_time_ = std::chrono::system_clock::now();
  return 0;
}

int VinModule::VinStop() {
  LOGD << "enter vin module stop...";
  if (vin_start_flag_ == false) {
    LOGW << "vin module has not start!";
    return 0;
  }

  bool decode_en = vin_buf_cfg_.decode_en;
  if (decode_en == true) {
    decode_is_running_ = false;
    if (sp_feed_decoder_task_) {
      sp_feed_decoder_task_->join();
    }
    if (sp_get_decoder_task_) {
      sp_get_decoder_task_->join();
    }
  } else {
    parse_raw_is_running_ = false;
    if (sp_parse_raw_task_) {
      sp_parse_raw_task_->join();
    }
  }
  if (sp_update_vinmodule_task_) {
    sp_update_vinmodule_task_->join();
  }
  vin_start_flag_ = false;
  LOGD << "vin module stop success...";
  return 0;
}

int VinModule::BufferQueueInit() {
  LOGD << "enter BufferQueueInit...";
  int ret = -1;
  int max_vin_buf_width = vin_buf_cfg_.max_width;
  int max_vin_buf_height = vin_buf_cfg_.max_height;
  int max_queue_len = vin_buf_cfg_.max_queue_len;
  HorizonVisionPixelFormat fmt = vin_buf_cfg_.format;
  bool decode_en = vin_buf_cfg_.decode_en;

  auto start_time = std::chrono::system_clock::now();
  if (max_queue_len <= 0 || max_queue_len > MAX_QUEUE_SIZE
      || max_vin_buf_width <= 0 || max_vin_buf_height <= 0
      || max_vin_buf_width > 8192 || max_vin_buf_height > 8192) {
    LOGE << "BufferQueueInit parameter is error!"
      << " max_queue_len: " << max_queue_len
      << " buf width: " << max_vin_buf_width
      << " buf height: " << max_vin_buf_height;
    return -1;
  }

  nv12_total_queue_ =
    std::make_shared<BlockingQueue<std::shared_ptr<ImageFrame>>>();
  nv12_free_queue_ =
    std::make_shared<BlockingQueue<std::shared_ptr<ImageFrame>>>();
  nv12_work_queue_ =
    std::make_shared<BlockingQueue<std::shared_ptr<ImageFrame>>>();
  nv12_done_queue_ =
    std::make_shared<BlockingQueue<std::shared_ptr<ImageFrame>>>();
  nv12_user_queue_ =
    std::make_shared<BlockingQueue<std::shared_ptr<ImageFrame>>>();

  BufferManager &manager = BufferManager::Get();
  ret = manager.Init();
  if (ret) {
    LOGE << "buffer manager init failed, ret: " << ret;
    return ret;
  }
  max_vin_queue_len_ = static_cast<size_t>(max_queue_len);
  uint32_t size_y = static_cast<uint32_t>(
      max_vin_buf_width * max_vin_buf_height);
  uint32_t size_uv = size_y / 2;

  for (size_t index = 0; index < max_vin_queue_len_; index++) {
    std::shared_ptr<ImageFrame> image_buf = nullptr;
    if (decode_en == false && vin_buf_cfg_.use_vb == false) {
      buf_lane_num_ = 0;
      image_buf = std::make_shared<ImageFrame>();
      LOGW <<  "not alloc image buffer and disable decoder...";
      // TODo
    } else {
      if (kHorizonVisionPixelFormatNV12 == fmt
          || kHorizonVisionPixelFormatYUY2 == fmt
          || (decode_en == true && (kHorizonVisionPixelFormatJPEG <= fmt))) {
        std::shared_ptr<ImageFrame> nv12_buf = nullptr;
        buf_lane_num_ = 2;
        ret = manager.AllocBuf2Lane(size_y, size_uv, nv12_buf);
        if (ret) {
          LOGE << "alloc buffer 2 lane failed, ret: " << ret;
          return ret;
        }
        image_buf = nv12_buf;
      } else if ((decode_en == false && kHorizonVisionPixelFormatJPEG <= fmt)
          || kHorizonVisionPixelFormatRaw == fmt) {
        std::shared_ptr<ImageFrame> raw_buf = nullptr;
        buf_lane_num_ = 1;
        // encode data less than nv12 data length
        // raw format pix_length: 0:8 、1:10 、2:12、3:14 、4:16 5:20
        uint32_t size;
        if (vin_buf_cfg_.raw_pixel_len == 0) {
          size = size_y;
        } else if (vin_buf_cfg_.raw_pixel_len == 1) {
          size = size_y * 10 / 8;
        } else if (vin_buf_cfg_.raw_pixel_len == 2) {
          size = size_y * 12 / 8;
        } else if (vin_buf_cfg_.raw_pixel_len == 3) {
          size = size_y * 14 / 8;
        } else if (vin_buf_cfg_.raw_pixel_len == 4) {
          size = size_y * 16 / 8;
        } else if (vin_buf_cfg_.raw_pixel_len == 5) {
          size = size_y * 20 / 8;
        } else {
          LOGE << "UnSupport raw pixel len: " << vin_buf_cfg_.raw_pixel_len;
          return -1;
        }
        ret = manager.AllocBufLane(size, raw_buf);
        if (ret) {
          LOGE << "alloc buffer lane failed, ret: " << ret;
          return ret;
        }
        image_buf = raw_buf;
      } else {
        LOGE << "not support pixel format: " << fmt;
        return -1;
      }
    }

  auto *vin_frame_ctx = reinterpret_cast<FrameContext*>(
      std::calloc(1, sizeof(FrameContext)));
    image_buf->src_context_ = reinterpret_cast<void*>(vin_frame_ctx);
    vin_frame_ctx->vin_buf_index = static_cast<int>(index);
    vin_frame_ctx->vps_ipu_chn = -1;
    image_buf->channel_id_ = channel_id_;
    nv12_free_queue_->push(image_buf);
    nv12_total_queue_->push(image_buf);
    LOGD << "alloc vin image frame buffer success, index: " << index
      << " channel_id: " << image_buf->channel_id_
      << " frame_id: " << image_buf->frame_id_
      << " time_stamp: " << image_buf->time_stamp_
      << " system_time_stamp: " << image_buf->system_time_stamp_
      << " buf_index: " << index
      << std::hex << " y_paddr: " << "0x" << image_buf->src_info_.y_paddr
      << std::hex << " c_paddr: " << "0x" << image_buf->src_info_.c_paddr
      << " y_vaddr: " << image_buf->src_info_.y_vaddr
      << " c_vaddr: " << image_buf->src_info_.c_vaddr;
  }
  auto curr_time = std::chrono::system_clock::now();
  auto cost_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(
        curr_time - start_time).count();
  LOGW << "vin buffer queue init success, cost_time: " << cost_time << "ms";
  return 0;
}

int VinModule::BufferQueueDeInit() {
  LOGD << "enter BufferQueueDeInit...";
  int ret = -1;
  size_t cnt = nv12_total_queue_->size();

  BufferManager &manager = BufferManager::Get();
  for (size_t index = 0; index < cnt; index++) {
    std::shared_ptr<ImageFrame> image_buf = nullptr;
    image_buf = nv12_total_queue_->pop();
    if (buf_lane_num_ == 1) {
      ret = manager.FreeBufLane(image_buf);
      if (ret) {
        LOGE << "free buffer lane failed, ret: " << ret
          << " index: " << index;
      }
    } else if (buf_lane_num_ == 2) {
      ret = manager.FreeBuf2Lane(image_buf);
      if (ret) {
        LOGE << "free buffer 2 lane failed, ret: " << ret
          << " index: " << index;
      }
    }
    if (image_buf->src_context_) {
      std::free(image_buf->src_context_);
      image_buf->src_context_ = nullptr;
    }
  }

  ret = manager.DeInit();
  if (ret) {
    LOGE << "buffer manager deinit failed, ret: " << ret;
    return ret;
  }
  nv12_free_queue_->clear();
  nv12_work_queue_->clear();
  nv12_done_queue_->clear();
  nv12_user_queue_->clear();

  return 0;
}

int VinModule::convert_yuy2_to_nv12(
    void *in_frame,
    void *y_out_frame,
    void *c_out_frame,
    int width,
    int height) {
  int i, j, k, tmp;
  unsigned char *src, *y_dest, *c_dest;

  if (!in_frame || !y_out_frame || !c_out_frame || !width || !height) {
    LOGE << "some error happen... in_frame:" << in_frame
         << "  y_out_frame:" << y_out_frame
         << "  c_out_frame:" << c_out_frame
         << "  width: " << width
         << "  height:" << height;
    return -1;
  }

  src = reinterpret_cast<unsigned char *>(in_frame);
  y_dest = reinterpret_cast<unsigned char *>(y_out_frame);
  c_dest = reinterpret_cast<unsigned char *>(c_out_frame);

  /* convert y */
  for (i = 0, k = 0; i < width * height * 2 && k < width * height;
       i += 2, k++) {
    y_dest[k] = src[i];
  }

  /* convert u, v */
  for (j = 0, k = 0; j < height && k < width * height * 3 / 2;
       j += 2) {        /* 4:2:0, 1/2 u&v */
    for (i = 1; i < width * 2; i += 2) {
      tmp = i + j * width * 2;
      c_dest[k] = src[tmp];
    }
  }

  return 0;
}

bool VinModule::CheckBufferInfo(uint32_t width, uint32_t height,
    uint32_t stride) {
  int src_width = width;
  int src_height = height;
  int src_stride = stride;
  int dst_width = vin_buf_cfg_.max_width;
  int dst_height = vin_buf_cfg_.max_height;
  int align_image_width = ALIGN(vin_image_width_, 16);
  int align_image_height = ALIGN(vin_image_height_, 16);

  if (src_width <= 0 || src_height <= 0 || src_stride <= 0
      || src_width > 8192 || src_height > 8192 || src_stride > 8192) {
    LOGE << "frame parameter is error!"
      << " width: " << src_width
      << " height: " << src_height
      << " stride:" << src_stride;
    return false;
  }

  if (dst_width < src_width || dst_height < src_height) {
    LOGE << "dst_width: " << dst_width
      << " dst_height: " << dst_height
      << " src_width: " << src_width
      << " src_height: " << src_height;
    return false;
  }

  if (dst_width != align_image_width
      && dst_height != align_image_height) {
    LOGE  << "vin image buffer length align error,"
      << " dst_width: " << dst_width
      << " dst_height: " << dst_height
      << " src_width: " << src_width
      << " src_height: " << src_height;
    return false;
  }
  return true;
}

bool VinModule::CheckPixelFormat(HorizonVisionPixelFormat &fmt) {
  switch (fmt) {
    case kHorizonVisionPixelFormatRaw:
    case kHorizonVisionPixelFormatYUY2:
    case kHorizonVisionPixelFormatNV12:
    case kHorizonVisionPixelFormatJPEG:
    case kHorizonVisionPixelFormatMJPEG:
    case kHorizonVisionPixelFormatH264:
    case kHorizonVisionPixelFormatH265:
      break;
    default: {
      LOGE << "Unsupport pixel format: " << fmt;
      return false;
    }
  }
  return true;
}

int VinModule::RawDataToNv12Thread() {
  parse_raw_is_running_ = true;

  sp_parse_raw_task_ = std::make_shared<std::thread>(
      [this] () {
         while (parse_raw_is_running_) {
            int ret = -1;
            std::pair<VinFrame, std::shared_ptr<unsigned char>> frame_data;
            if (raw_queue_.try_pop(&frame_data,
                  std::chrono::milliseconds(200))) {
               auto vin_frame = frame_data.first;
               auto sp_buf = frame_data.second;
               // dump vin input data
               DumpVinInputData(vin_frame);
               // prase raw data
               ret = this->ParseRawData(vin_frame);
               if (ret) {
                  LOGE << "parse raw data failed, ret: " << ret;
                  continue;
               }
             }
           }
         });

  return 0;
}

int VinModule::CheckFirstKeyFrame(const VinFrame &frame) {
  int ret = 0;
  HorizonVisionPixelFormat fmt = frame.format;
  bool is_key_frame = frame.is_key_frame;

  if (fmt == kHorizonVisionPixelFormatH264
      || fmt == kHorizonVisionPixelFormatH265) {
    if (check_first_idr_ == false) {
      if (is_key_frame == true) {  // first idr frame
        check_first_idr_ = true;  // check success
        encode_queue_.clear();
        LOGW << "find key frame, drop frame in encode_queue before IDR frame";
      } else {
        ret = -1;  // check failure
      }
    }
  }
  return ret;
}

int VinModule::SendStreamToVdec() {
  int ret = -1;
  std::pair<VinFrame, std::shared_ptr<unsigned char>> frame_data;

  if (encode_queue_.try_pop(&frame_data,
        std::chrono::milliseconds(200))) {
    auto vin_frame = frame_data.first;
    auto sp_buf = frame_data.second;
    // check key frame
    auto frame_size = vin_frame.size;
    uint64_t frame_id = vin_frame.frame_id;
    ret = CheckFirstKeyFrame(vin_frame);
    if (ret) {
      LOGW << "this frame: " << frame_id << " is not key frame, will drop";
      return 0;
    }
    // dump vin input data
    DumpVinInputData(vin_frame);
    // decode input data
    ret = decode_module_->Input(frame_id,
        sp_buf.get(), frame_size);
    if (ret) {
      LOGE << "decode module input failed, ret: " << ret;
      return ret;
    }
    send_stream_num_++;
    if (vin_sync_state_ == true) {
      sem_post(&send_sync_sem_);
    }
    LOGD << "vin module send stream success..."
      << " frame_id: " << frame_id
      << " frame_size: " << frame_size
      << " send_stream_num: " << send_stream_num_;
  }
  return 0;
}

int VinModule::DecodeDataToNv12Thread() {
  if (decode_module_ == nullptr) {
    LOGE << "decode module is nullptr!";
    return -1;
  }
  decode_is_running_ = true;

  sp_feed_decoder_task_ = std::make_shared<std::thread>(
      [this] () {
          while (decode_is_running_) {
          int ret = -1;
          ret = SendStreamToVdec();
          if (ret) {
            LOGE << "send stream to vdec failed,ret: " << ret;
            continue;
          }
        }
     });

  sp_get_decoder_task_ = std::make_shared<std::thread>(
      [this] () {
         while (decode_is_running_) {
         std::shared_ptr<ImageFrame> sp_nv12_frame = nullptr;
         int ret = -1;
         uint64_t timeout = 2000;  // 2000ms

         LOGD << "nv12_free_queue size: " << nv12_free_queue_->size()
           << " nv12_work_queue size: " << nv12_work_queue_->size()
           << " nv12_done_queue size: " << nv12_done_queue_->size()
           << " nv12_user_queue size: " << nv12_user_queue_->size();
         if (false == nv12_free_queue_->try_pop(&sp_nv12_frame,
               std::chrono::microseconds(1000))) {
            /**
             * auto from done_queue get buffer to free_queue in
             * video source(async mode) or in image source(sync mode)
             */
            if ((channel_id_ == -1) || (vin_sync_state_ == false)) {
               if (nv12_done_queue_->size() > 0) {
                  auto nv12_frame_tmp = nv12_done_queue_->pop();
                  nv12_free_queue_->push(nv12_frame_tmp);
               } else {
                  LOGE << "nv12_done_queue is empty, chn_id: " << channel_id_
                    << " nv12_free_queue size: " << nv12_free_queue_->size()
                    << " nv12_work_queue size: " << nv12_work_queue_->size()
                    << " nv12_user_queue size: " << nv12_user_queue_->size();
               }
            } else {
              // vin module sync mode
              LOGD << "enter wait nv12_free_queue TimeoutWait...";
              ret = TimeoutWait(nv12_free_queue_sem_, timeout);
              if (ret < 0) {
                 LOGE << "wait nv12_free_queue timeout: " << timeout << "ms"
                  << " please call FreeFrame interface quickly!"
                  << " nv12_free_queue size: " << nv12_free_queue_->size()
                  << " nv12_work_queue size: " << nv12_work_queue_->size()
                  << " nv12_done_queue size: " << nv12_done_queue_->size()
                  << " nv12_user_queue size: " << nv12_user_queue_->size();
              }
              LOGD << "wait nv12_free_queue TimeoutWait success...";
            }
            continue;
         }
         LOGD << "find free buffer for decoder task";
         void* dst_y_addr =
           reinterpret_cast<void*>(sp_nv12_frame->src_info_.y_vaddr);
         void* dst_c_addr =
           reinterpret_cast<void*>(sp_nv12_frame->src_info_.c_vaddr);

         // sync wait send_sync_sem signal
         if (vin_sync_state_ == true) {
           ret = TimeoutWait(send_sync_sem_, timeout);
           if (ret < 0) {
              LOGE << "wait send_sync_sem timeout: " << timeout <<"ms"
              << " send_stream_num: " << send_stream_num_;
              nv12_free_queue_->push(sp_nv12_frame);
              continue;
           }
         }
         // get frame from vdec
         std::shared_ptr<ImageFrame> output_frame = nullptr;
         ret = decode_module_->Output(output_frame);
         if (ret) {
            LOGE << "decode module output failed, ret: " << ret
              << " send_stream_num: " << send_stream_num_;
            nv12_free_queue_->push(sp_nv12_frame);
            continue;
         }
         send_stream_num_--;
         LOGD << "decode frame suceess, frame_id: " << output_frame->frame_id_
           << " send_stream_num: " << send_stream_num_;
         void* src_y_addr =
           reinterpret_cast<void*>(output_frame->src_info_.y_vaddr);
         void* src_c_addr =
           reinterpret_cast<void*>(output_frame->src_info_.c_vaddr);
         uint32_t src_width =
           static_cast<uint32_t>(output_frame->src_info_.width);
         uint32_t src_height =
           static_cast<uint32_t>(output_frame->src_info_.height);
         uint32_t src_stride =
           static_cast<uint32_t>(output_frame->src_info_.stride);
         uint32_t src_y_size = src_stride * src_height;
         uint32_t src_c_size = src_y_size / 2;

         if (false == CheckBufferInfo(src_width, src_height, src_stride)) {
           LOGE << "check buffer info failed";
           ret = decode_module_->Free(output_frame);
           HOBOT_CHECK(ret == 0) << "decode mododule free failed";
           nv12_free_queue_->push(sp_nv12_frame);
           continue;
         }

         // update src buffer info to buffer queue
         sp_nv12_frame->src_info_.width =
           static_cast<uint16_t>(vin_image_width_);
         sp_nv12_frame->src_info_.height =
           static_cast<uint16_t>(vin_image_height_);
         sp_nv12_frame->src_info_.stride =
           static_cast<uint16_t>(src_stride);

         // copy y image
         HOBOT_CHECK(src_y_addr && dst_y_addr && src_y_size)
           << " src_y_addr: " << src_y_addr
           << " src_y_size: " << src_y_size
           << " dst_y_addr: " << dst_y_addr;
         memcpy(dst_y_addr, src_y_addr, src_y_size);
         // copy uv image
         HOBOT_CHECK(src_c_addr && dst_c_addr && src_c_size)
           << " src_c_addr: " << src_c_addr
           << " src_c_size: " << src_c_size
           << " dst_c_addr: " << dst_c_addr;
         memcpy(dst_c_addr, src_c_addr, src_c_size);

         // update pixel_format, frame_id
         sp_nv12_frame->pixel_format_ = output_frame->pixel_format_;
         sp_nv12_frame->frame_id_ = output_frame->frame_id_;
         // decode moudle free frame
         ret = decode_module_->Free(output_frame);
         HOBOT_CHECK(ret == 0) << "decode mododule free failed";

         // update hardware timestamp and system timestamp
         struct timeval tv;
         gettimeofday(&tv, NULL);
         auto tv_ms = ((uint64_t)tv.tv_sec * 1000) +
           ((uint64_t)tv.tv_usec / 1000);
         sp_nv12_frame->system_time_stamp_ = tv_ms;  // System timestamp
         // non-mipi_camera has no hardware timestamp
         sp_nv12_frame->time_stamp_ = 0;  // HW timestamp

         FrameContext* vin_frame_ctx =
           reinterpret_cast<FrameContext*>(sp_nv12_frame->src_context_);
         HOBOT_CHECK(vin_frame_ctx);
         LOGD << "vin module get nv12 frame success, "
           << " channel_id: " << sp_nv12_frame->channel_id_
           << " frame_id: " << sp_nv12_frame->frame_id_
           << " time_stamp: " << sp_nv12_frame->time_stamp_
           << " system_time_stamp: " << sp_nv12_frame->system_time_stamp_
           << " buf_index: " << vin_frame_ctx->vin_buf_index
           << std::hex << " y_paddr: " << "0x" \
           << sp_nv12_frame->src_info_.y_paddr
           << std::hex << " c_paddr: " << "0x" \
           << sp_nv12_frame->src_info_.c_paddr
           << " y_vaddr: " << sp_nv12_frame->src_info_.y_vaddr
           << " c_vaddr: " << sp_nv12_frame->src_info_.c_vaddr;

         // callback, send nv12_frame to next module
         if (cb_func_) {
           // push nv12 frame to work queue
           nv12_work_queue_->push(sp_nv12_frame);
           LOGD << "push nv12 frame to nv12_work_queue...";
           cb_func_(sp_nv12_frame);
         } else {
           // push nv12 frame to done queue if next module has not enable
           nv12_done_queue_->push(sp_nv12_frame);
           LOGD << "push nv12 frame to nv12_done_queue...";
         }
         // dump vin out frame data
         DumpVinOutputData(sp_nv12_frame);
       }
     });

  return 0;
}

int VinModule::SetFrameFromWorkedToCompleted(const uint64_t &frame_id) {
  int ret = -1;
  std::shared_ptr<ImageFrame> sp_nv12_frame = nullptr;

  if (vin_init_flag_ == false) {
    return 0;
  }
  // 1. querry first element in work queue list
  HOBOT_CHECK(nv12_work_queue_) << "nv12_work_queue is empty";
  if (false == nv12_work_queue_->try_peek(&sp_nv12_frame)) {
    LOGE << "nv12_work_queue is try peek failed!";
    return -1;
  }
  if (frame_id == sp_nv12_frame->frame_id_) {
    LOGD << "set frame to completed success, frame_id: " << frame_id;
    std::shared_ptr<ImageFrame> nv12_frame_tmp = nullptr;
    nv12_frame_tmp = nv12_work_queue_->pop();
    nv12_done_queue_->push(nv12_frame_tmp);
    return 0;
  }

  // 2. frame is not first element in work queue list, need traverse list
  std::deque<std::shared_ptr<ImageFrame>> &frame_list =
    nv12_work_queue_->get_data();
  for (std::deque<std::shared_ptr<ImageFrame>>::iterator it = \
      frame_list.begin(); it != frame_list.end();) {
    auto id = (*it)->frame_id_;
    if (frame_id == id) {
      // push frame_id frame to nv12_done_queue
      nv12_done_queue_->push(*it);
      it = frame_list.erase(it);
      ret = 0;
      break;
    } else if (id < frame_id) {
      // push previous frame before frame_id to nv12_done_queue
      nv12_done_queue_->push(*it);
      it = frame_list.erase(it);
    } else {
      it++;
    }
  }
  if (ret) {
    LOGE << "set frame to completed failed!"
      << " can not find frame_id: " << frame_id
      << " in nv12_work_queue";
    return ret;
  }
  LOGD << "set frame to completed success, frame_id: " << frame_id;
  return 0;
}

int VinModule::InputRawDataWithoutVb(VinFrame frame) {
  LOGD << "enter InputRawDataWithoutVb function...";
  std::shared_ptr<ImageFrame> sp_nv12_frame = nullptr;
  uint64_t frame_id = frame.frame_id;

  sp_nv12_frame = std::make_shared<ImageFrame>();
  // update pixel_format and frame_id
  sp_nv12_frame->pixel_format_ = frame.format;
  sp_nv12_frame->frame_id_ = frame_id;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  auto time_ms = ((uint64_t)tv.tv_sec * 1000) + ((uint64_t)tv.tv_usec / 1000);
  sp_nv12_frame->system_time_stamp_ = time_ms;  // System timestamp
  if (frame.time_stamp) {
    sp_nv12_frame->time_stamp_ = frame.time_stamp;  // Hardware timestamp
  }
  sp_nv12_frame->time_stamp_ = 0;  // non-mipi_camera has no hardware timestamp

  sp_nv12_frame->src_info_.width = static_cast<uint16_t>(frame.width);
  sp_nv12_frame->src_info_.height = static_cast<uint16_t>(frame.height);
  sp_nv12_frame->src_info_.stride = static_cast<uint16_t>(frame.stride);
  sp_nv12_frame->src_info_.y_vaddr =
    reinterpret_cast<uint64_t>(frame.vaddr[0]);
  sp_nv12_frame->src_info_.c_vaddr =
    reinterpret_cast<uint64_t>(frame.vaddr[1]);
  sp_nv12_frame->src_info_.y_paddr = frame.paddr[0];
  sp_nv12_frame->src_info_.c_paddr = frame.paddr[1];
  if (frame.paddr[0]  == 0 || frame.paddr[1] == 0) {
    LOGE << "frame phy addr is zero.";
    return -1;
  }

  if (cb_func_) {
    cb_func_(sp_nv12_frame);  // send frame to next module
  }
  // dump vin out frame data
  DumpVinOutputData(sp_nv12_frame);

  return 0;
}

int VinModule::ParseRawData(VinFrame &frame) {
  int ret = -1;
  int chret = 0;
  bool get_queue_buf = false;
  std::shared_ptr<ImageFrame> sp_nv12_frame = nullptr;
  HorizonVisionPixelFormat fmt = frame.format;
  uint64_t frame_id = frame.frame_id;
  uint64_t timeout = 2000;  // 2000ms

  do {
    if (vin_start_flag_ == false) {
      LOGI << "vin module need quit...";
      return 0;
    }
    if (false == nv12_free_queue_->try_pop(&sp_nv12_frame,
          std::chrono::microseconds(1000))) {
      /**
       * auto from done_queue get buffer to free_queue in
       * video source(async mode) or in image source(sync mode)
       */
      if ((channel_id_ == -1) || (vin_sync_state_ == false)) {
        if (nv12_done_queue_->size() > 0) {
          auto nv12_frame_tmp = nv12_done_queue_->pop();
          nv12_free_queue_->push(nv12_frame_tmp);
        } else {
          LOGE << "nv12_done_queue is empty!";
        }
      } else {
        ret = TimeoutWait(nv12_free_queue_sem_, timeout);
        if (ret < 0) {
          LOGE << "wait nv12_free_queue timeout: " << timeout << "ms"
            << " please call FreeFrame interface quickly";
        }
      }
      continue;
    }
    get_queue_buf = true;
  } while (get_queue_buf == false);

  LOGD << "find free buffer for parse raw task";

  uint32_t src_width = static_cast<uint32_t>(frame.width);
  uint32_t src_height = static_cast<uint32_t>(frame.height);
  uint32_t src_stride = static_cast<uint32_t>(frame.stride);

  if (false == CheckBufferInfo(src_width, src_height, src_stride)) {
    LOGE << "check buffer info failed";
    nv12_free_queue_->push(sp_nv12_frame);
    chret = -1;
  }
  HOBOT_CHECK(chret == 0) << "check buffer info failed";

  void* y_dest = reinterpret_cast<void*>(sp_nv12_frame->src_info_.y_vaddr);
  void* c_dest = reinterpret_cast<void*>(sp_nv12_frame->src_info_.c_vaddr);
  sp_nv12_frame->src_info_.width = static_cast<uint16_t>(frame.width);
  sp_nv12_frame->src_info_.height = static_cast<uint16_t>(frame.height);
  sp_nv12_frame->src_info_.stride = static_cast<uint16_t>(frame.stride);

  switch (fmt) {
    case kHorizonVisionPixelFormatNV12: {
      uint32_t dest_size = frame.width * frame.height * 3 / 2;
      HOBOT_CHECK(dest_size == frame.size)
        << " dest_size: " << dest_size
        << " frame_size: " << frame.size;
      int y_size = frame.stride * frame.height;
      int c_size = y_size / 2;
      char* y_src = reinterpret_cast<char*>(frame.vaddr[0]);
      char* c_src = nullptr;
      if (frame.plane_count == 1) {
        c_src = y_src + y_size;
      } else if (frame.plane_count == 2) {
        c_src = reinterpret_cast<char*>(frame.vaddr[1]);
      } else {
        LOGE << "UnSupport plane_count: " << frame.plane_count;
      }
      HOBOT_CHECK(y_src && c_src && y_dest && c_dest);
      memcpy(y_dest, y_src, y_size);
      memcpy(c_dest, c_src, c_size);
      break;
    }
    case kHorizonVisionPixelFormatYUY2: {
      uint32_t dest_size = frame.width * frame.height * 3 / 2;
      HOBOT_CHECK(dest_size != frame.size);
      if (convert_yuy2_to_nv12(frame.vaddr[0], y_dest, c_dest,
            frame.width, frame.height) < 0) {
        LOGE << "convert data from yuy2 to nv12 failed...";
        nv12_free_queue_->push(sp_nv12_frame);
        chret = -1;
      }
      break;
    }
    case kHorizonVisionPixelFormatRaw: {
      HOBOT_CHECK(frame.plane_count == 1);
      char* raw_addr = reinterpret_cast<char*>(frame.vaddr[0]);
      uint32_t frame_size = frame.size;
      uint32_t raw_size = static_cast<uint32_t>(frame.stride * frame.height);
      HOBOT_CHECK(raw_addr);
      HOBOT_CHECK(raw_size == frame_size);
      memcpy(y_dest, raw_addr, raw_size);
      break;
    }
    default: {
      LOGE << "Unsupport pixel format: " << fmt;
      nv12_free_queue_->push(sp_nv12_frame);
      chret = -1;
    }
  }
  HOBOT_CHECK(chret == 0);

  // update pixel_format and frame_id
  sp_nv12_frame->pixel_format_ = fmt;
  sp_nv12_frame->frame_id_ = frame_id;

  // update hardware timestamp and system timestamp
  struct timeval tv;
  gettimeofday(&tv, NULL);
  auto time_ms = ((uint64_t)tv.tv_sec * 1000) + ((uint64_t)tv.tv_usec / 1000);
  sp_nv12_frame->system_time_stamp_ = time_ms;  // System timestamp
  if (frame.time_stamp) {
    sp_nv12_frame->time_stamp_ = frame.time_stamp;  // Hardware timestamp
  }
  sp_nv12_frame->time_stamp_ = 0;  // non-mipi_camera has no hardware timestamp


  // callback, send nv12_frame to next module
  if (cb_func_ && (fmt != kHorizonVisionPixelFormatRaw)) {
    // push nv12 frame to work queue
    nv12_work_queue_->push(sp_nv12_frame);
    LOGD << "push nv12 frame to nv12_work_queue...";
    cb_func_(sp_nv12_frame);
  } else {
    // push nv12 frame to done queue
    nv12_done_queue_->push(sp_nv12_frame);
    LOGD << "push nv12 frame to nv12_done_queue...";
  }
  // dump vin out frame data
  DumpVinOutputData(sp_nv12_frame);
  return 0;
}

void VinModule::DumpVinInputData(const VinFrame &frame) {
  HorizonVisionPixelFormat fmt = frame.format;
  uint64_t frame_id = frame.frame_id;
  std::string file_name;
  bool dump_en = false;
  int dump_num = 0;

  GetDumpNum("./vin_input.txt", dump_en, dump_num);
  if (dump_en == true && (output_dump_count_ < dump_num || dump_num < 0)) {
    std::ofstream ofs;
    if (fmt == kHorizonVisionPixelFormatJPEG
        || fmt == kHorizonVisionPixelFormatMJPEG) {
      file_name = "dump_" + std::to_string(frame_id) + ".jpg";
      ofs.open(file_name);
    } else if (fmt == kHorizonVisionPixelFormatH264) {
      if (dump_num < 0)
        file_name = "dump.h264";
      else
        file_name = "dump_" + std::to_string(dump_num) + ".h264";
      ofs.open(file_name, std::ios::app);
    } else if (fmt == kHorizonVisionPixelFormatH265) {
      if (dump_num < 0)
        file_name = "dump.h265";
      else
        file_name = "dump_" + std::to_string(dump_num) + ".h265";
      ofs.open(file_name, std::ios::app);
    } else if (fmt == kHorizonVisionPixelFormatNV12
        || fmt == kHorizonVisionPixelFormatYUY2) {
      file_name = "dump_" + std::to_string(frame_id) + ".yuv";
      ofs.open(file_name);
    } else if (fmt == kHorizonVisionPixelFormatRaw) {
      file_name = "dump_" + std::to_string(frame_id) + ".raw";
      ofs.open(file_name);
    }
    output_dump_count_++;
    LOGW << "dump fname: " << file_name
      << " frame_id: " << frame_id
      << " frame_addr: " << reinterpret_cast<void*>(frame.vaddr[0])
      << " frame_size: " << frame.size;

    ofs.write((const char*)frame.vaddr[0], frame.size);
    ofs.flush();
    ofs.close();
  }
}

void VinModule::DumpVinOutputData(std::shared_ptr<ImageFrame> &nv12_frame) {
  std::string file_name;
  uint64_t frame_id = nv12_frame->frame_id_;
  HorizonVisionPixelFormat fmt = nv12_frame->pixel_format_;
  bool dump_en = false;
  int dump_num = 0;
  void *y_buf, *c_buf, *raw_buf;
  uint32_t y_size, c_size, raw_size;

  if (nv12_frame == nullptr) {
    return;
  }
  GetDumpNum("./vin_output.txt", dump_en, dump_num);

  uint32_t width = static_cast<uint32_t>(nv12_frame->src_info_.width);
  uint32_t height = static_cast<uint32_t>(nv12_frame->src_info_.height);
  uint32_t stride = static_cast<uint32_t>(nv12_frame->src_info_.stride);

  if (fmt == kHorizonVisionPixelFormatNV12) {
    y_buf = reinterpret_cast<void*>(nv12_frame->src_info_.y_vaddr);
    c_buf = reinterpret_cast<void*>(nv12_frame->src_info_.c_vaddr);
    y_size = stride * height;
    c_size = y_size / 2;
    if (dump_en == true && (input_dump_count_ < dump_num || dump_num < 0)) {
      input_dump_count_++;
      file_name = "vin_output" + std::to_string(frame_id) + ".yuv";
      LOGW << "dump fname: " << file_name
        << " width: " << width
        << " height: " << height
        << " stride: " << stride;
      std::ofstream ofs(file_name);
      ofs.write((const char*)y_buf, y_size);
      ofs.write((const char*)c_buf, c_size);
      ofs.flush();
      ofs.close();
    }
  } else if (fmt == kHorizonVisionPixelFormatRaw) {
    raw_buf = reinterpret_cast<void*>(nv12_frame->src_info_.y_vaddr);
    raw_size = stride * height;
    int raw_bits = ((stride * 1.0) / width) * 8;
    if (dump_en == true && (input_dump_count_ < dump_num || dump_num < 0)) {
      input_dump_count_++;
      file_name = "vin_output" + std::to_string(frame_id) + ".raw" +
        std::to_string(raw_bits);
      LOGW << "dump fname: " << file_name
        << " width: " << width
        << " height: " << height
        << " stride: " << stride;
      std::ofstream ofs(file_name);
      ofs.write((const char*)raw_buf, raw_size);
      ofs.flush();
      ofs.close();
    }
  } else {
    LOGE << "UnSupport image frame format.";
    return;
  }
}

int VinModule::DecodeInit() {
  int ret = -1;
  int width = vin_buf_cfg_.max_width;
  int height = vin_buf_cfg_.max_height;
  HorizonVisionPixelFormat fmt = vin_buf_cfg_.format;
  bool decode_en = vin_buf_cfg_.decode_en;
  int max_queue_len = vin_buf_cfg_.max_queue_len;
  DecodeHandle handle;

  if (decode_en == false) {
    LOGW << "decode has not enable";
    return 0;
  }

  DecodeManager &manager = DecodeManager::Get();
  handle = manager.CreateDecodeHandle(fmt);
  if (handle == nullptr) {
    LOGE << "create decode handle failed, ret: " << ret;
    return ret;
  }
  decode_module_ = std::make_shared<DecodeModule>(handle);
  ret = decode_module_->Init(width, height, max_queue_len);
  if (ret) {
    LOGE << "decode module init failed, ret: " << ret;
    return ret;
  }

  return 0;
}

int VinModule::DecodeDeInit() {
  int ret = -1;
  bool decode_en = vin_buf_cfg_.decode_en;

  if (decode_en == false) {
    LOGW << "decode has not enable!";
    return 0;
  }

  if (decode_module_) {
    ret = decode_module_->DeInit();
    if (ret) {
      LOGE << "decode module deinit failed, ret: " << ret;
      return ret;
    }
  }
  return 0;
}

int VinModule::InputRawData(VinFrame frame) {
  LOGD << "enter InputRawData function...";
  if (frame.vaddr[0] == nullptr) {
    LOGE << "frame data is nullptr";
    return -1;
  }

  unsigned char *dest =
    reinterpret_cast<unsigned char *>(calloc(1, frame.size));
  assert(dest != NULL);
  // clear phy address
  frame.paddr[0] = 0;
  frame.paddr[1] = 0;
  frame.paddr[2] = 0;
  if (frame.plane_count == 1) {
    memcpy(dest, frame.vaddr[0], frame.plane_size[0]);
    // use dest data instead frame data
    frame.vaddr[0] = reinterpret_cast<void*>(dest);
  } else if (frame.plane_count == 2) {
    memcpy(dest, frame.vaddr[0], frame.plane_size[0]);
    memcpy(dest + frame.plane_size[0], frame.vaddr[1], frame.plane_size[1]);
    // use dest data instead frame data
    frame.vaddr[0] = reinterpret_cast<void*>(dest);
    frame.vaddr[1] = reinterpret_cast<void*>(dest + frame.plane_size[0]);
  } else if (frame.plane_count == 3) {
    memcpy(dest, frame.vaddr[0], frame.plane_size[0]);
    memcpy(dest + frame.plane_size[0], frame.vaddr[1], frame.plane_size[1]);
    memcpy(dest + frame.plane_size[0] + frame.plane_size[1],
        frame.vaddr[2], frame.plane_size[2]);
    // use dest data instead frame data
    frame.vaddr[0] = reinterpret_cast<void*>(dest);
    frame.vaddr[1] = reinterpret_cast<void*>(dest + frame.plane_size[0]);
    frame.vaddr[2] = reinterpret_cast<void*>(dest + frame.plane_size[0] +
        frame.plane_size[1]);
  } else {
    LOGE << "Unsupport frame plane_count: " << frame.plane_count;
    std::free(dest);
    return -1;
  }
  // use dest data instead frame data
  frame.vaddr[0] = reinterpret_cast<void*>(dest);
  auto sp_frame_data = std::shared_ptr<unsigned char>(dest,
      [] (unsigned char* p) {
      if (p) {
        free(p);
        p = NULL;
      }
  });

  auto frame_data = std::make_pair(frame, sp_frame_data);
  raw_queue_.push(std::move(frame_data));

  if (vin_sync_state_ == false &&
      (raw_queue_.size() > max_cached_queue_len_)) {
    auto frame_data = raw_queue_.pop();
    VinFrame vin_frame = frame_data.first;
    uint64_t frame_id = vin_frame.frame_id;
    LOGW << "raw queue size " << raw_queue_.size()
      << " exceeds limit " << max_cached_queue_len_
      << " frame_id: " << frame_id
      << " will be discard";
  }

  return 0;
}

int VinModule::InputEncodeData(VinFrame frame) {
  LOGD << "enter InputEncodeData function...";
  if (frame.vaddr[0] == nullptr) {
    LOGE << "frame data is nullptr";
    return -1;
  }
  if (frame.format < kHorizonVisionPixelFormatJPEG
      || frame.format > kHorizonVisionPixelFormatH265) {
    LOGE << "UnSupport input format: " << frame.format;
    return -1;
  }

  unsigned char *dest =
    reinterpret_cast<unsigned char *>(calloc(1, frame.size));
  assert(dest != NULL);
  memcpy(dest, frame.vaddr[0], frame.size);
  // use dest data instead frame data
  frame.vaddr[0] = reinterpret_cast<void*>(dest);
  auto sp_frame_data = std::shared_ptr<unsigned char>(dest,
      [] (unsigned char* p) {
      if (p) {
        free(p);
        p = NULL;
      }
  });

  auto frame_data = std::make_pair(frame, sp_frame_data);
  encode_queue_.push(std::move(frame_data));

  if (vin_sync_state_ == false &&
      (encode_queue_.size() > max_cached_queue_len_)) {
    auto frame_data = encode_queue_.pop();
    VinFrame vin_frame = frame_data.first;
    uint64_t frame_id = vin_frame.frame_id;
    LOGW << "encode queue size " << encode_queue_.size()
      << " exceeds limit " << max_cached_queue_len_
      << " frame_id: " << frame_id
      << " will be discard";
  }
  return 0;
}

void VinModule::GetDumpNum(const std::string &input,
    bool &dump_en, int &dump_num) {
  if (0 == access(input.c_str(), F_OK)) {
    dump_en = true;
    std::string str;
    std::ifstream ifs(input);
    std::getline(ifs, str);
    if (str.empty()) {
      dump_num = -1;
    } else {
      // trim the spaces in the beginning
      str.erase(0, str.find_first_not_of(' '));
      // trim the spaces in the end
      str.erase(str.find_last_not_of(' ') + 1, std::string::npos);
      dump_num = std::stoi(str);
    }
  }
}

void VinModule::UpdateVinModule(VinFrame &frame) {
  int ret = -1;
  HorizonVisionPixelFormat fmt = frame.format;
  LOGW << "enter update vin module: " << fmt
    << " vin_buf_cfg format: " << vin_buf_cfg_.format
    << " vin_sync_state: " << vin_sync_state_
    << " channel_id: " << channel_id_;

  auto start_time = std::chrono::system_clock::now();
  if (vin_init_flag_ == false || vin_start_flag_ == false) {
    ret = VinInit();
    HOBOT_CHECK(ret == 0) << "VinModule init failed, ret: " << ret;
    ret = VinStart();
    HOBOT_CHECK(ret == 0) << "VinModule start failed, ret: " << ret;
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time).count();
    LOGW << "update init vinmodule success,"
      << " cost_time: " << cost_time << "ms"
      << " new vin image width: " << vin_image_width_
      << " new vin image height: " << vin_image_height_;
  } else {
    // deinit current vin module for channge different resolution
    ret = VinStop();
    HOBOT_CHECK(ret == 0) << "VinModule stop failed, ret: " << ret;
    ret = VinDeInit();
    HOBOT_CHECK(ret == 0) << "VinModule deinit failed, ret: " << ret;
    ret = VinInit();
    HOBOT_CHECK(ret == 0) << "VinModule init failed, ret: " << ret;
    ret = VinStart();
    HOBOT_CHECK(ret == 0) << "VinModule start failed, ret: " << ret;
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time).count();
    LOGW << "update deinit and init vinmodule success,"
      << " cost_time: " << cost_time << "ms"
      << " new vin image width: " << vin_image_width_
      << " new vin image height: " << vin_image_height_;
  }
}

int VinModule::InputData(VinFrame &frame) {
  int ret = -1;
  VinFrame frame_data = frame;
  HorizonVisionPixelFormat fmt = frame.format;
  uint64_t frame_id = frame.frame_id;
  bool decode_en = vin_buf_cfg_.decode_en;
  bool use_vb = vin_buf_cfg_.use_vb;

  if (false == CheckPixelFormat(fmt)) {
    LOGE << "check pixel format info failed, fmt: " << fmt;
    return -1;
  }

  LOGD << "Enter InputData, update_flag: " << update_flag_
    << " init_flag: " << vin_init_flag_
    << " start_flag: " << vin_start_flag_
    << " decode_en: " << decode_en
    << " frame_id: " << frame_id
    << " use_vb: " << use_vb
    << " vin_fps_out_en: " << vin_fps_out_en_;

  if (update_flag_ == true && use_vb == true) {
    update_flag_ = false;
    if (vin_sync_state_ == true) {
      UpdateVinModule(frame_data);
    } else {
      sp_update_vinmodule_task_ = std::make_shared<std::thread>(
         [&, this] () {
          UpdateVinModule(frame_data);
      });
    }
  }
  /* calculate vin fps */
  {
    if (vin_fps_out_en_ == true) {
      vin_fps_++;
      auto curr_time = std::chrono::system_clock::now();
      auto cost_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - start_time_);
      if (cost_time.count() >= 1000) {
        LOGW << "channel_id:" << channel_id_ << " vin_fps:" << vin_fps_;
        start_time_ = std::chrono::system_clock::now();
        vin_fps_ = 0;
      }
    }
  }

  if (false == decode_en) {
    if (use_vb == true) {
      ret = InputRawData(frame);
      if (ret) {
        LOGE << "input raw data failed, ret: " << ret;
        return ret;
      }
    } else {  // use internal vio buffer
      ret = InputRawDataWithoutVb(frame);
      if (ret) {
        LOGE << "input raw data without vb failed, ret: " << ret;
        return ret;
      }
    }
  } else {
    ret = InputEncodeData(frame);
    if (ret) {
      LOGE << "input encode data failed, ret: " << ret;
      return ret;
    }
  }

  return 0;
}

bool VinModule::VinStartTimeoutWait(
    const std::chrono::milliseconds &timeout) {
  std::unique_lock<std::mutex> lock(vin_mutex_);
  if (!vin_condition_.wait_for(lock, timeout,
        [this] { return (vin_start_flag_ == true); })) {
    return false;
  }
  return true;
}

int VinModule::GetFrame(std::shared_ptr<ImageFrame> &output_image) {
  if (vin_out_en_ == false) {
    LOGE << "vin module output not enable";
    return -1;
  }
  if (nv12_user_queue_->size() == max_vin_queue_len_) {
    LOGE << "nv12 user queue is full, need FreeFrame";
    return -1;
  }

  std::shared_ptr<ImageFrame> sp_nv12_frame = nullptr;
  if (false == nv12_done_queue_->try_pop(&sp_nv12_frame,
        std::chrono::milliseconds(2000))) {
    if (nv12_done_queue_->size() <= 0) {
      LOGE << "nv12_done_queue is empty!";
      return -1;
    }
  }
  nv12_user_queue_->push(sp_nv12_frame);
  output_image = sp_nv12_frame;
  LOGD << "get vin frame success, frame_id: " << output_image->frame_id_;
  return 0;
}

int VinModule::FreeFrame(const std::shared_ptr<ImageFrame> &input_image) {
  int ret = -1;
  std::shared_ptr<ImageFrame> sp_nv12_frame = nullptr;
  FrameContext* input_frame_ctx =
    reinterpret_cast<FrameContext*>(input_image->src_context_);
  HOBOT_CHECK(input_frame_ctx) << "input context is nullptr";
  int input_buf_index = input_frame_ctx->vin_buf_index;

  if (vin_init_flag_ == false) {
    LOGW << "vin module has not init!";
    return -1;
  }
  if (vin_start_flag_ == false) {
    LOGE << "vin module has not start!";
    return -1;
  }
  if (vin_out_en_ == false) {
    LOGE << "vin module output not enable";
    return -1;
  }

  // 1. free frame is first element in user queue list
  if (false == nv12_user_queue_->try_peek(&sp_nv12_frame)) {
    LOGE << "nv12_user_queue is try peek failed!";
    return -1;
  }
  FrameContext* nv12_frame_ctx =
    reinterpret_cast<FrameContext*>(sp_nv12_frame->src_context_);
  HOBOT_CHECK(nv12_frame_ctx) << "nv12 frame context is nullptr";
  if (input_buf_index == nv12_frame_ctx->vin_buf_index) {
    std::shared_ptr<ImageFrame> nv12_frame_tmp = nullptr;
    nv12_frame_tmp = nv12_user_queue_->pop();
    nv12_free_queue_->push(nv12_frame_tmp);
    auto free_queue_size = nv12_free_queue_->size();
    LOGD << "push frame_id: " << nv12_frame_tmp->frame_id_
      << " to nv12_free_queue success"
      << " nv12_free_queue size: " << free_queue_size;
    ret = 0;
  } else {
    // 2. free frame is not first element in user queue list,
    // need traverse list
    std::deque<std::shared_ptr<ImageFrame>> &frame_list =
      nv12_user_queue_->get_data();
    for (std::deque<std::shared_ptr<ImageFrame>>::iterator it = \
        frame_list.begin(); it != frame_list.end();) {
      auto frame_ctx =
        reinterpret_cast<FrameContext*>((*it)->src_context_);
      if (input_buf_index == frame_ctx->vin_buf_index) {
        nv12_free_queue_->push(*it);
        it = frame_list.erase(it);
        ret = 0;
        break;
      }
      it++;
    }
  }
  if (ret) {
    LOGE << "can not find input_buf_index: " << input_buf_index
      << " in nv12_user_queue";
    return ret;
  }
  if (vin_sync_state_ == true) {
    sem_post(&nv12_free_queue_sem_);
  }
  LOGD << "vin module free nv12 frame success, "
    << " channel_id: " << input_image->channel_id_
    << " frame_id: " << input_image->frame_id_
    << " time_stamp: " << input_image->time_stamp_
    << " system_time_stamp: " << input_image->system_time_stamp_
    << " buf_index: " << input_buf_index
    << std::hex << " y_paddr: " << "0x" \
    << input_image->src_info_.y_paddr
    << std::hex << " c_paddr: " << "0x" \
    << input_image->src_info_.c_paddr
    << " y_vaddr: " << input_image->src_info_.y_vaddr
    << " c_vaddr: " << input_image->src_info_.c_vaddr;
  return 0;
}

}  // namespace videosource
