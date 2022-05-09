/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "video_source_plugin/video_source_produce.h"
#include <cstdint>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "hobotlog/hobotlog.hpp"
#include "utils/executor.h"

#include "video_source/video_source.h"
#include "video_source/video_source_type.h"
#include "video_source_plugin/video_source_message.h"
#include "video_source_plugin/video_source_process.h"
#include "video_source_produce.h"


namespace xproto {

std::shared_ptr<VideoSourceProduce>
  VideoSourceProduce::CreateVideoSourceProduce(
    const std::string &produce_name,
    const int &channel_id,
    const std::string &cfg_file) {
    LOGD << "enter CreateVideoSourceProduce, "
      << " produce_name: " << produce_name
      << " channel_id: " << channel_id
      << " cfg_file: " << cfg_file;
  std::shared_ptr<VideoSourceProduce> produce;
  if ("mipi_camera" == produce_name) {
    produce = std::make_shared<MipiCamera>(channel_id, cfg_file);
  } else if ("panel_camera" == produce_name) {
    produce = std::make_shared<PanelCamera>(channel_id, cfg_file);
  } else if ("usb_camera" == produce_name) {
    produce = std::make_shared<UsbCamera>(channel_id, cfg_file);
  } else if ("feedback" == produce_name) {
    produce = std::make_shared<Feedback>(channel_id, cfg_file);
  } else if ("rtsp_client" == produce_name) {
    produce = std::make_shared<RtspClient>(channel_id, cfg_file);
  } else {
    LOGE << "produce_name " << produce_name << " is unsupported";
    return nullptr;
  }
  return produce;
}

int VideoSourceProduce::SetConfig(const ProduceConfig &cfg) {
  produce_cfg_list_.push_back(cfg);

  max_vio_buffer_ = cfg.max_vio_buffer;
  VideoSourceType src_type = video_source_->GetSourceType();
  if (src_type == VideoSourceType::kSOURCE_TYPE_MIPI_CAM
      || src_type == VideoSourceType::kSOURCE_TYPE_USB_CAM) {
    source_type_ = kHorizonVideoSourceTypeCam;
  } else if (src_type == VideoSourceType::kSOURCE_TYPE_FEEDBACK
      || src_type == VideoSourceType::kSOURCE_TYPE_RTSP_CLIENT) {
    source_type_ = kHorizonVideoSourceTypeFb;
  } else {
    LOGE << "video source type: " << src_type << " is unsupported";
    return -1;
  }

  enum VideoSourceLogLevel level;
  if (cfg.log_level == "-i") {
    level = VideoSourceLogLevel::kLOG_LEVEL_INFO;
    video_source_->SetLoggingLevel(level);
  } else if (cfg.log_level == "-d") {
    level = VideoSourceLogLevel::kLOG_LEVEL_DEBUG;
    video_source_->SetLoggingLevel(level);
  } else if (cfg.log_level == "-w") {
    level = VideoSourceLogLevel::kLOG_LEVEL_WARN;
    video_source_->SetLoggingLevel(level);
  } else if (cfg.log_level == "-e") {
    level = VideoSourceLogLevel::kLOG_LEVEL_ERROR;
    video_source_->SetLoggingLevel(level);
  } else if (cfg.log_level == "-f") {
    level = VideoSourceLogLevel::kLOG_LEVEL_FATAL;
    video_source_->SetLoggingLevel(level);
  } else {
    int Log_level = GetLogLevel();
    if (Log_level == 7) {  // HOBOT_LOG_NULL
      SetLogLevel(HOBOT_LOG_WARN);
      level = VideoSourceLogLevel::kLOG_LEVEL_WARN;
      LOGW << "set default log level: [-w] ";
      video_source_->SetLoggingLevel(level);
    } else {
      LOGW << "log level has been set, log_level: " << Log_level;
    }
  }
  return 0;
}

MipiCamera::MipiCamera(const int &channel_id, const std::string &cfg_file) {
  channel_id_ = channel_id;
  if (nullptr == video_source_) {
    video_source_ = std::make_shared<VideoSource>(channel_id, cfg_file);
    LOGI << "MipiCamera create video source channel_id: " << channel_id;
  }
}

MipiCamera::~MipiCamera() {
  if (video_source_) {
    video_source_ = nullptr;
  }
}

UsbCamera::UsbCamera(const int &channel_id, const std::string &cfg_file) {
  channel_id_ = channel_id;
  if (nullptr == video_source_) {
    video_source_ = std::make_shared<VideoSource>(channel_id, cfg_file);
    LOGI << "UsbCamera create video source channel_id: " << channel_id;
  }
}

UsbCamera::~UsbCamera() {
  if (video_source_) {
    video_source_ = nullptr;
  }
}

Feedback::Feedback(const int &channel_id, const std::string &cfg_file) {
  channel_id_ = channel_id;
  if (nullptr == video_source_) {
    video_source_ = std::make_shared<VideoSource>(channel_id, cfg_file);
    LOGI << "Feedback create video source channel_id: " << channel_id;
  }
}

Feedback::~Feedback() {
  if (video_source_) {
    video_source_ = nullptr;
  }
}

RtspClient::RtspClient(const int &channel_id, const std::string &cfg_file) {
  channel_id_ = channel_id;
  if (nullptr == video_source_) {
    video_source_ = std::make_shared<VideoSource>(channel_id, cfg_file);
    LOGI << "RtspClient create video source channel_id: " << channel_id;
  }
}

RtspClient::~RtspClient() {
  if (video_source_) {
    video_source_ = nullptr;
  }
}

void VideoSourceProduce::WaitUntilAllDone() {
  LOGW << "Enter WaitUntilAllDone, "
    << "pipe_id: " << channel_id_
    << " consumed_vio_buffers: " << consumed_vio_buffers_;
  int try_step = 0;
  int max_try_num = 5;
  while (consumed_vio_buffers_ > 0 && try_step < max_try_num) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    try_step++;
    LOGW << "pipe_id: " << channel_id_
      << " consumed_vio_buffers: " << consumed_vio_buffers_
      << " wait time: " << try_step << " seconds"
      << " max_wait_time: " << max_try_num << " seconds";
  }
  if (try_step == max_try_num) {
    LOGE << "WaitUntilAllDone timeout, force quit";
  }
}

bool VideoSourceProduce::AllocBuffer() {
  std::lock_guard<std::mutex> lk(vio_buffer_mutex_);
  LOGD << "Enter AllocBuffer(), consumed_vio_buffers: "
    << consumed_vio_buffers_;
  if (consumed_vio_buffers_ < max_vio_buffer_) {
    ++consumed_vio_buffers_;
    LOGD << "alloc buffer success, consumed_vio_buffers_: "
         << consumed_vio_buffers_;
    return true;
  }
  return false;
}

void VideoSourceProduce::FreeBuffer() {
  std::lock_guard<std::mutex> lk(vio_buffer_mutex_);
  LOGD << "Enter FreeBuffer(), consumed_vio_buffers: "
    << consumed_vio_buffers_;
  if (0 >= consumed_vio_buffers_) {
    LOGF << "should not happen!";
    return;
  }
  --consumed_vio_buffers_;
  LOGD << "free buffer success, consumed_vio_buffers_: "
       << consumed_vio_buffers_;
}

int VideoSourceProduce::SetListener(const Listener &callback) {
  push_data_cb_ = callback;
  return 0;
}

int VideoSourceProduce::Finish() {
  if (is_running_) {
    is_running_ = false;
  }
  WaitUntilAllDone();
  return 0;
}

int VideoSourceProduce::Init() {
  int ret = -1;
  if (is_inited_) {
    return 0;
  }
  if (video_source_) {
    LOGD << "video source init ready";
    ret = video_source_->Init();
    HOBOT_CHECK_EQ(ret, 0) << "video source init failed";
    LOGI << "video source init success, channel_id: " << channel_id_;
  }
  is_inited_ = true;
  return 0;
}

int VideoSourceProduce::DeInit() {
  if (video_source_) {
    video_source_->DeInit();
    LOGI << "video source deinit success, channel_id: " << channel_id_;
  }
  is_inited_ = false;
  return 0;
}

int VideoSourceProduce::Start() {
  LOGI << "enter video source produce start,"
    << " is_running: " << is_running_
    << " videosource: " << video_source_;
  if (is_running_) {
    return 0;
  }
  int ret = -1;
  if (video_source_) {
    ret = video_source_->Start();
    HOBOT_CHECK_EQ(ret, 0) << "video source start failed";
    LOGI << "video source start success, channel_id: " << channel_id_;
  }
  auto func = std::bind(&VideoSourceProduce::Run, this);
  task_future_ = Executor::GetInstance(pipe_num_)->AddTask(func);
  // produce_pym_thread_ = std::make_shared<std::thread>(func);
  return 0;
}

int VideoSourceProduce::Stop() {
  if (!is_running_) {
    return 0;
  }
  is_running_ = false;
  this->Finish();
  LOGW << "wait task to finish";
  if (produce_pym_thread_) {
    produce_pym_thread_->join();
  }

  if (video_source_) {
    video_source_->Stop();
  }
  task_future_.get();
  LOGD << "task done";
  return 0;
}

int MipiCamera::Run() {
  int ret = -1;
  if (is_running_) {
    return 0;
  }
  is_running_ = true;
  LOGI << "Enter mipi camera run thread";
  while (is_running_) {
#if 1
    ret = PushPyramidFrameMsg();
    if (ret) {
      LOGE << "push pyramid frame msg thread failed, ret: " << ret;
      continue;
    }
#else
    ret = PushSourceFrameMsg();
    if (ret) {
      LOGE << "push source frame msg thread failed, ret: " << ret;
      continue;
    }
#endif
  }
  return 0;
}

int Feedback::Run() {
  int ret = -1;
  if (is_running_) {
    return 0;
  }
  is_running_ = true;
  LOGI << "Enter feedback run thread";
  while (is_running_) {
    ret = PushPyramidFrameMsg();
    if (ret) {
      LOGE << "push pyramid frame msg thread failed, ret: " << ret;
      if (ret == -VideoSourceErrorCode::kERROR_CODE_SOURCE_IS_STOP) {
        if (video_source_) {
          video_source_->Stop();
        }
        is_running_ = false;
      }
      continue;
    }
  }
  return 0;
}

int UsbCamera::Run() {
  int ret = -1;
  if (is_running_) {
    return 0;
  }
  is_running_ = true;
  LOGI << "Enter usb camera run thread";
  while (is_running_) {
    ret = PushPyramidFrameMsg();
    if (ret) {
      LOGE << "push pyramid frame msg thread failed, ret: " << ret;
      continue;
    }
  }
  return 0;
}

int RtspClient::Run() {
  int ret = -1;
  if (is_running_) {
    return 0;
  }
  is_running_ = true;
  LOGI << "Enter rtsp client run thread";
  while (is_running_) {
    ret = PushPyramidFrameMsg();
    if (ret) {
      LOGE << "push pyramid frame msg thread failed, ret: " << ret;
      continue;
    }
  }
  return 0;
}

int VideoSourceProduce::PushSourceFrameMsg() {
  int ret = -1;
  std::vector<std::shared_ptr<ImageFrame>> ipu_image_list;
  ret = video_source_->GetVpsImageFrame(ipu_image_list);
  if (ret) {
    LOGE << "get vps image frame failed, ret: " << ret;
  }
  ret = video_source_->FreeVpsImageFrame(ipu_image_list);
  if (ret) {
    LOGE << "free vps image frame failed, ret: " << ret;
  }
  return 0;
}

int VideoSourceProduce::PushPyramidFrameMsg() {
  int ret = -1;
  uint32_t img_num = 1;
  static uint32_t frame_id = 0;
  bool send_vio_msg = false;
  bool send_drop_msg = false;
  std::shared_ptr<ImageFrame> vin_image = nullptr;
  std::shared_ptr<PyramidFrame> pym_image = nullptr;
  std::shared_ptr<xstream::PyramidImageFrame> pym_msg_image;
  bool vin_en = video_source_->GetVinEnable();
  bool vps_en = video_source_->GetVpsEnable();
  bool is_sync_mode = video_source_->GetSyncMode();
  bool vin_out_en = video_source_->GetVinOutEnable();

  // push pyramid msg, vin and vps must enable
  if (vin_en == false || vps_en == false) {
    LOGE << "video source vin or vps has not enable"
      << " vin_en: " << vin_en
      << " vps_en: " << vps_en
      << " need quit produce thread";
    is_running_ = false;
    return -1;
  }

  // sync mode
  if (is_sync_mode == true) {
    if (vin_out_en == false) {
      LOGE << "vin_out_en must enable in sync mode";
      return -1;
    }
    if (frame_id % sample_freq_ == 0 && AllocBuffer()) {
      // has free vio buffer, send vio msg
      // 1.get vin image frame
      ret = video_source_->GetVinImageFrame(vin_image);
      if (ret) {
        LOGE << "get vin image frame failed, ret: " << ret;
        FreeBuffer();
        goto err;
      }
      // 2.send vin frame to vps
      ret = video_source_->SendFrameToVps(vin_image);
      if (ret) {
        LOGE << "send frame to vps failed, ret: " << ret;
        FreeBuffer();
        goto err;
      }
      // 3.get pyramid frame
      ret = video_source_->GetPyramidFrame(pym_image);
      if (ret) {
        LOGE << "get pyramid frame failed in sync mode, ret: " << ret;
        {
          std::lock_guard<std::mutex> lk(vio_buffer_mutex_);
          LOGE << "GetPyramidFrame failed, ret=" << ret
            << " consumed_vio_buffers_= " << consumed_vio_buffers_;
        }
        // system("cat /proc/meminfo | grep MemAvailable > mem.log");
        FreeBuffer();
        goto err;
      }
      // 4.free vin frame
      if (vin_image) {
        ret = video_source_->FreeVinImageFrame(vin_image);
        if (ret) {
          LOGE << "free vin frame failed, ret: " << ret;
          FreeBuffer();
          goto err;
        }
        vin_image = nullptr;
      }
      send_vio_msg = true;
    } else {
      // has no free vio buffer, not send drop msg
      LOGD << "no drop frame in sync mode";
      return 0;
    }
  } else {
    // async mode
    // get pyramid frame
    ret = video_source_->GetPyramidFrame(pym_image);
    if (ret) {
      LOGE << "get pyramid frame failed in async mode, ret: " << ret;
      {
        std::lock_guard<std::mutex> lk(vio_buffer_mutex_);
        LOGE << "GetPyramidFrame failed, ret=" << ret
          << " consumed_vio_buffers_= " << consumed_vio_buffers_;
      }
      // system("cat /proc/meminfo | grep MemAvailable > mem.log");
      goto err;
    }
    // judge send vio msg or drop msg
    if (frame_id % sample_freq_ == 0 && AllocBuffer()) {
      // has free vio buffer, send vio msg
      send_vio_msg = true;
    } else {
      send_drop_msg = true;
    }
  }

  // judge is quit or not
  if (!is_running_) {
    LOGE << "stop video source job";
    if (send_vio_msg == true) {
      FreeBuffer();
    }
    ret = -1;
    goto err;
  }

  HOBOT_CHECK(pym_image) << "pym image is nullptr,"
    << " please check pym chn is enable or not";
  // drop repeat frame_id
  if (pym_image->frame_id_ == last_frame_id_) {
    LOGI << "vio has repeat frame_id: " << pym_image->frame_id_
      << " this frame will be drop...";
    if (send_vio_msg == true) {
      FreeBuffer();
    }
    ret = 0;
    goto err;
  }
  last_frame_id_ = pym_image->frame_id_;

  // send vio or drop msg
  ConvertPym2Msg(pym_image, pym_msg_image);
  if (send_vio_msg == true) {
    // push vio image message
    // convert pyramid to msg
    std::shared_ptr<VioMessage> input(
        new ImageVioMessage(video_source_, pym_msg_image, img_num),
        [&](ImageVioMessage *p) {
        if (p) {
          LOGI << "begin delete ImageVioMessage";
          FreeBuffer();
          delete (p);
        }
        p = nullptr;
        });
    if (push_data_cb_) {
      push_data_cb_(input);
      LOGD << "Push Image message,"
        << " chn_id: " << pym_msg_image->channel_id_
        << " frame_id: " << pym_msg_image->frame_id_;
    }
  } else {
    if (send_drop_msg == true) {
      // push drop image message
      std::shared_ptr<VioMessage> input(
          new DropImageVioMessage(video_source_, pym_msg_image, img_num),
          [&](DropImageVioMessage *p) {
          if (p) {
            LOGI << "begin delete DropImageVioMessage";
            delete (p);
          }
          p = nullptr;
          });
      if (push_data_cb_) {
        push_data_cb_(input);
        LOGW << "Push Drop Image message,"
          << " consumed_vio_buffer: " << consumed_vio_buffers_
          << " max_vio_buffer: " << max_vio_buffer_
          << " chn_id: " << pym_msg_image->channel_id_
          << " frame_id: " << pym_msg_image->frame_id_;
      }
    }
  }
  ++frame_id;
  return 0;

err:
  if (vin_image) {
    video_source_->FreeVinImageFrame(vin_image);
    vin_image = nullptr;
  }
  if (pym_image) {
    video_source_->FreePyramidFrame(pym_image);
    pym_image = nullptr;
  }
  return ret;
}

}  // namespace xproto
