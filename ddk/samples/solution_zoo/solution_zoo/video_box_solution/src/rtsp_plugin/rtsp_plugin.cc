/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-01 20:38:52
 * @Version: v0.0.1
 * @Brief: smartplugin impl based on xstream.
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-29 05:04:11
 */
#include "rtsp_plugin/rtsp_plugin.h"

#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "BasicUsageEnvironment.hh"
#include "hobotlog/hobotlog.hpp"
#include "liveMedia.hh"
#include "media_pipe_manager/media_pipe_manager.h"
#include "media_pipe_manager/media_pipeline.h"
#include "rtsp_client/sps_info_mgr.h"
#include "rtsp_plugin/rtsp_message.h"
#include "smart_plugin/display_info.h"
#include "unistd.h"
#include "video_box_common.h"
#include "xproto/message/flowmsg.h"
#include "xproto/message/msg_registry.h"
#include "xproto/msg_type/vio_message.h"
#include "xproto/plugin/xpluginasync.h"
#include "xstream/xstream_world.h"
// #include "mediapipemanager/meidapipelinetest.h"

// #define PIPE_TEST

namespace solution {
namespace video_box {

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;
using xproto::message::VioMessage;

using xstream::PyramidImageFrame;

using ImageFramePtr = std::shared_ptr<xstream::ImageFrame>;

using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

XPLUGIN_REGISTER_MSG_TYPE(TYPE_DECODE_IMAGE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(TYPE_DECODE_DROP_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_IMAGE_MESSAGE)
XPLUGIN_REGISTER_MSG_TYPE(XPLUGIN_DROP_MESSAGE)

int RtspPlugin::frame_count_ = 1;
std::mutex RtspPlugin::framecnt_mtx_;

void Convert(pym_buffer_t *pym_buffer, xstream::PyramidImageFrame &pym_img) {
  if (nullptr == pym_buffer) {
    return;
  }
  pym_img.img_.ds_pym_layer = DOWN_SCALE_MAX;
  pym_img.img_.us_pym_layer = UP_SCALE_MAX;
  pym_img.img_.frame_id = pym_buffer->pym_img_info.frame_id;
  pym_img.img_.timestamp = pym_buffer->pym_img_info.time_stamp;
  pym_img.context_ = static_cast<void *>(pym_buffer);
  for (int i = 0; i < DOWN_SCALE_MAX; ++i) {
    address_info_t *pym_addr = NULL;
    if (i % 4 == 0) {
      pym_addr = reinterpret_cast<address_info_t *>(&pym_buffer->pym[i / 4]);
    } else {
      pym_addr = reinterpret_cast<address_info_t *>(
          &pym_buffer->pym_roi[i / 4][i % 4 - 1]);
    }
    // std::cout << "dxd1 : " << pym_addr->width << std::endl;
    pym_img.img_.down_scale[i].width = pym_addr->width;
    pym_img.img_.down_scale[i].height = pym_addr->height;
    pym_img.img_.down_scale[i].step = pym_addr->stride_size;
    pym_img.img_.down_scale[i].y_paddr = pym_addr->paddr[0];
    pym_img.img_.down_scale[i].c_paddr = pym_addr->paddr[1];
    pym_img.img_.down_scale[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->addr[0]);
    pym_img.img_.down_scale[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_addr->addr[1]);
  }
  for (int i = 0; i < UP_SCALE_MAX; ++i) {
    pym_img.img_.up_scale[i].width = pym_buffer->us[i].width;
    pym_img.img_.up_scale[i].height = pym_buffer->us[i].height;
    pym_img.img_.up_scale[i].step = pym_buffer->us[i].stride_size;
    pym_img.img_.up_scale[i].y_paddr = pym_buffer->us[i].paddr[0];
    pym_img.img_.up_scale[i].c_paddr = pym_buffer->us[i].paddr[1];
    pym_img.img_.up_scale[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_buffer->us[i].addr[0]);
    pym_img.img_.up_scale[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_buffer->us[i].addr[1]);
  }
  for (int i = 0; i < DOWN_SCALE_MAIN_MAX; ++i) {
    // std::cout << "dxd2 : " << pym_buffer->pym[i].width << std::endl;
    pym_img.img_.down_scale_main[i].width = pym_buffer->pym[i].width;
    pym_img.img_.down_scale_main[i].height = pym_buffer->pym[i].height;
    pym_img.img_.down_scale_main[i].step = pym_buffer->pym[i].stride_size;
    pym_img.img_.down_scale_main[i].y_paddr = pym_buffer->pym[i].paddr[0];
    pym_img.img_.down_scale_main[i].c_paddr = pym_buffer->pym[i].paddr[1];
    pym_img.img_.down_scale_main[i].y_vaddr =
        reinterpret_cast<uint64_t>(pym_buffer->pym[i].addr[0]);
    pym_img.img_.down_scale_main[i].c_vaddr =
        reinterpret_cast<uint64_t>(pym_buffer->pym[i].addr[1]);
  }
}

char eventLoopWatchVariable = 0;

int RtspPlugin::Init() {
  running_ = false;
  GetConfigFromFile(config_file_);
  SPSInfoMgr::GetInstance().Init();
  MediaPipeManager::GetInstance().Init();
  for (int i = 0; i < channel_number_; ++i) {
    std::shared_ptr<solution::video_box::MediaPipeline> pipeline =
        std::make_shared<solution::video_box::MediaPipeline>(i, i);
    pipeline->SetFrameDropFlag(drop_frame_);
    pipeline->SetFrameDropInterval(drop_frame_interval_);
    MediaPipeManager::GetInstance().AddPipeline(pipeline);
  }

  if (not_run_smart_) {
    video_processor_ = std::make_shared<VideoProcessor>();
    smart_vo_cfg_t smart_vo_cfg_;
    video_processor_->Init(channel_number_, display_mode_, smart_vo_cfg_, false,
                           false, false, true);
  }
  return XPluginAsync::Init();
}

void RtspPlugin::ProcessData(const int channel_id, pym_buffer_t *pym_buffer) {
  int layer =
      DisplayInfo::computePymLayer(display_mode_, channel_number_, channel_id);
  address_info_t *pym_addr = NULL;
  if (layer % 4 == 0) {
    pym_addr = reinterpret_cast<address_info_t *>(&pym_buffer->pym[layer / 4]);
  } else {
    pym_addr = reinterpret_cast<address_info_t *>(
        &pym_buffer->pym_roi[layer / 4][layer % 4 - 1]);
  }
  // std::cout << "dowm scale layer:" << layer << " width:" << pym_addr->width
  //           << " height:" << pym_addr->height << std::endl;
  auto video_data = std::make_shared<VideoData>();
  video_data->smart_frame = nullptr;
  uint32_t width_tmp = pym_addr->width;
  uint32_t height_tmp = pym_addr->height;
  char *y_addr = pym_addr->addr[0];
  char *uv_addr = pym_addr->addr[1];
  video_data->channel = channel_id;
  video_data->width = width_tmp;
  video_data->height = height_tmp;
  video_data->data_len = width_tmp * height_tmp * 3 / 2;
  video_data->buffer =
      static_cast<char *>(malloc(width_tmp * height_tmp * 3 / 2));
  for (uint32_t i = 0; i < height_tmp; ++i) {
    memcpy(video_data->buffer + i * width_tmp, y_addr + i * width_tmp,
           width_tmp);
  }

  for (uint32_t i = 0; i < (height_tmp / 2); ++i) {
    memcpy(video_data->buffer + (i + height_tmp) * width_tmp,
           uv_addr + i * width_tmp, width_tmp);
  }

  // frame_count_++;
  video_processor_->Input(video_data);
}

void RtspPlugin::GetDeocdeFrame(std::shared_ptr<MediaPipeline> pipeline,
                                int channel) {
  pym_buffer_t *out_pym_buf = nullptr;
  int ret = 0;

  while (running_) {
    ret = pipeline->Output((void **)(&out_pym_buf));
    if (ret != 0) {
      if (ret == -5) {  // not ready
        usleep(500 * 200);
        continue;
      }

      LOGI << "channel:" << channel << " Frame Drop";
      continue;
    }
    if (out_pym_buf == NULL) {
      LOGE << "mediapipeline output null pym buf, but not return error!";
      continue;
    }

    if (not_run_smart_) {
      ProcessData(channel, out_pym_buf);
      pipeline->OutputBufferFree(out_pym_buf);
      continue;
    }

    std::vector<std::shared_ptr<PyramidImageFrame>> pym_images;
    auto pym_image_frame_ptr = std::make_shared<PyramidImageFrame>();
    if (pym_image_frame_ptr == NULL) {
      LOGE << "make shared ptr fail, return null pointer!";
      continue;
    }
    Convert(out_pym_buf, *pym_image_frame_ptr);
    pym_image_frame_ptr->channel_id_ = channel;
    {
      std::lock_guard<std::mutex> lg(framecnt_mtx_);
      pym_image_frame_ptr->frame_id_ = frame_count_++;
    }
    pym_images.push_back(pym_image_frame_ptr);
    std::shared_ptr<VioMessage> input(
        new ImageVioMessage(pym_images, 1, 1, channel, pipeline, out_pym_buf),
        [&](ImageVioMessage *p) {
          if (p) {
            if (p->pipeline_ != nullptr) {
              p->pipeline_->OutputBufferFree(p->slot_data_);
            }

            delete p;
          }
          p = nullptr;
        });

    LOGD << "channel:" << channel
         << "image vio message construct  grp:" << pipeline->GetGrpId()
         << "  frame_id:" << pym_image_frame_ptr->frame_id_;
    PushMsg(input);
  }
}

void RtspPlugin::Process() {
  // Begin by setting up our usage environment:
  scheduler_ = BasicTaskScheduler::createNew();
  env_ = BasicUsageEnvironment::createNew(*scheduler_);

  for (int i = 0; i < channel_number_; ++i) {
    ourRTSPClient *client = nullptr;
    client = openURL(*env_, "RTSPClient", rtsp_url_[i].url.c_str(),
                     rtsp_url_[i].tcp_flag, rtsp_url_[i].frame_max_size,
                     ("channel" + std::to_string(i) + ".stream"),
                     rtsp_url_[i].save_stream, i);
    rtsp_clients_.push_back(client);
  }

  const std::vector<std::shared_ptr<MediaPipeline>> &pipelines =
      MediaPipeManager::GetInstance().GetPipeline();
  LOGE << "\n\nPipeline size : " << pipelines.size();
  running_ = true;
  std::thread *t[channel_number_];
  for (uint32_t i = 0; i < pipelines.size(); ++i) {
    t[i] = new std::thread(&RtspPlugin::GetDeocdeFrame, this, pipelines[i], i);
  }

  // All subsequent activity takes place within the event loop:
  env_->taskScheduler().doEventLoop(&eventLoopWatchVariable);
  for (int i = 0; i < channel_number_; ++i) {
    ourRTSPClient *client = rtsp_clients_[i];
    // operators cause crash if client is invalid
    if (rtsp_clients_stat_.at(i)) {
      client->sendTeardownCommand(*client->scs.session, NULL);
      Medium::close(client->scs.session);
    }
  }

  env_->reclaim();
  delete scheduler_;
  for (uint32_t i = 0; i < pipelines.size(); ++i) {
    if (t[i]) {
      if (t[i]->joinable()) {
        t[i]->join();
      }
      delete t[i];
    }
  }

  // This function call does not return, unless, at some point in time,
  // "eventLoopWatchVariable" gets set to something non-zero.

  // If you choose to continue the application past this point (i.e., if you
  // comment out the "return 0;" statement above), and if you don't intend to do
  // anything more with the "TaskScheduler" and "UsageEnvironment" objects, then
  // you can also reclaim the (small) memory used by these objects by
  // uncommenting the following code:
  /*
    env_->reclaim(); env_ = NULL;
    delete scheduler_; scheduler_ = NULL;
  */
}

void RtspPlugin::CheckRtspState() {
  const std::vector<std::shared_ptr<MediaPipeline>> &pipelines =
      MediaPipeManager::GetInstance().GetPipeline();

  sleep(10);  // wait for rtsp stream connect success
  LOGW << "Start CheckRtspState thread,running flag:" << running_;
  while (running_) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t time_now = (uint64_t)tv.tv_sec;
    for (uint32_t i = 0; i < pipelines.size(); ++i) {
      if (time_now - pipelines[i]->GetlastReadDataTime() < 10) {
        continue;
      }

      int channel = pipelines[i]->GetGrpId();
      LOGE << "RTSP channel:" << channel << " , 10 seconds no stream!";
      if (rtsp_clients_[i] && pipelines[i]->GetDecodeType() !=
          RTSP_Payload_NONE) {  // if not start, maybe the destructor
                                // has been called
        rtsp_clients_[i]->Stop();
      }

      // reopen rtsp url
      ourRTSPClient *client = nullptr;
      client = openURL(*env_, "RTSPClient", rtsp_url_[i].url.c_str(),
                       rtsp_url_[i].tcp_flag, rtsp_url_[i].frame_max_size,
                       ("channel" + std::to_string(i) + ".stream"),
                       rtsp_url_[i].save_stream, i);
      LOGI << "after reopen rtsp stream, channel:" << i;
      rtsp_clients_[i] = client;
      pipelines[i]->UpdateTime();
    }
    sleep(1);
  }
}

int RtspPlugin::Start() {
  if (not_run_smart_) {
    video_processor_->Start();
  }

  process_thread_ = std::make_shared<std::thread>(&RtspPlugin::Process, this);
  check_thread_ =
      std::make_shared<std::thread>(&RtspPlugin::CheckRtspState, this);
  return 0;
}

int RtspPlugin::Stop() {
  LOGW << "RtspPlugin Stop";
  running_ = false;
  LOGW << "process_thread_ Stop";
  const std::vector<std::shared_ptr<MediaPipeline>> &pipelines =
      MediaPipeManager::GetInstance().GetPipeline();
  LOGW << "pipe line size: " << pipelines.size();
  for (uint32_t i = 0; i < pipelines.size(); ++i) {
    pipelines[i]->Stop();
  }

  eventLoopWatchVariable = 1;
  check_thread_->join();
  process_thread_->join();
  if (not_run_smart_) {
    video_processor_->Stop();
  }
  return 0;
}

void RtspPlugin::GetConfigFromFile(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    LOGE << "Open config file " << path << " failed";
  }
  ifs >> config_;
  ifs.close();

  auto value_js = config_["rtsp_config_file"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: channel_num";
  }
  LOGW << value_js;
  std::string rtsp_config_file = value_js.asString();

  value_js = config_["drop_frame_config_file"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: channel_num";
  }
  LOGW << value_js;
  std::string drop_frame_config_file = value_js.asString();

  value_js = config_["display_config_file"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: channel_num";
  }
  LOGW << value_js;
  std::string display_config_file = value_js.asString();

  value_js = config_["run_smart"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: run_smart";
  } else {
    not_run_smart_ = !value_js.asBool();
  }
  LOGW << "video box config not run smart flag:" << not_run_smart_;

  std::ifstream rtsp_file(rtsp_config_file);
  if (!rtsp_file.is_open()) {
    LOGE << "Open config file " << path << " failed";
  }
  rtsp_file >> config_;
  rtsp_file.close();

  value_js = config_["channel_num"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: channel_num";
  }
  LOGW << value_js;
  channel_number_ = value_js.asInt();
  for (int i = 0; i < channel_number_; ++i) {
    std::string channel("channel" + std::to_string(i));
    std::string rtsp_url = config_[channel.c_str()]["rtsp_link"].asString();
    // bool use_tcp = config_[channel.c_str()]["tcp"].asBool();
    LOGW << channel << ": rtsp url: " << rtsp_url;
    Rtspinfo info;
    info.url = rtsp_url;
    info.tcp_flag = config_[channel.c_str()]["tcp"].asBool();
    info.frame_max_size = config_[channel.c_str()]["frame_max_size"].asInt();
    info.save_stream = config_[channel.c_str()]["save_stream"].asBool();
    LOGW << "channel: " << channel << " protocol tcp flag: " << info.tcp_flag
         << "max frame size:" << info.frame_max_size;
    rtsp_url_.push_back(info);
  }

  rtsp_clients_stat_.resize(channel_number_);

  std::ifstream drop_frame_file(drop_frame_config_file);
  if (!drop_frame_file.is_open()) {
    LOGE << "Open config file " << path << " failed";
  }
  drop_frame_file >> config_;
  drop_frame_file.close();

  value_js = config_["frame_drop"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: frame_drop";
  }

  drop_frame_ = config_["frame_drop"]["drop_frame"].asBool();
  drop_frame_interval_ = config_["frame_drop"]["interval_frames_num"].asInt();

  std::ifstream display_file(display_config_file);
  if (!display_file.is_open()) {
    LOGE << "Open config file " << path << " failed";
  }
  display_file >> config_;
  display_file.close();

  value_js = config_["vo"];
  if (value_js.isNull()) {
    LOGE << "Can not find key: vo";
  }
  display_mode_ = config_["vo"]["display_mode"].asInt();
}
}  // namespace rtspplugin
}  // namespace xproto
