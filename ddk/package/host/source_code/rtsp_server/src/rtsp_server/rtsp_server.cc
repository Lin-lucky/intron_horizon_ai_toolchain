#include "rtsp_server.h"
#include "horizon_rtsp_component.h"
#include "hobotlog/hobotlog.hpp"
#include <BasicUsageEnvironment.hh>
#include <InputFile.hh>
#include <RTSPServerSupportingHTTPStreaming.hh>
#include <liveMedia.hh>
#include "OnDemandServerMediaSubsession.hh"
#include <fcntl.h>
#include "rtsp_server_config.h"

namespace rtspcomponent {
RtspServer::RtspServer(std::string config_file) {
  config_ = new RtspServerConfig(config_file);
}

RtspServer::~RtspServer() {
  if (config_) {
    delete config_;
  }
}

int RtspServer::Init() {
  LOGD << "RtspServer Init";
  rtsp_server_thread_ = nullptr;
  auto ret = config_->LoadConfig();
  if (!ret) {
    LOGE << "rtsp server load config failed";
    return -1;
  }
  watch_variable_ = 0;
  return 0;
}

int RtspServer::Start() {
  LOGD << "RtspServer Start";
  if (nullptr == rtsp_server_thread_) {
    rtsp_server_thread_ =
      std::make_shared<std::thread>(&RtspServer::RTSPServerRun, this);
  }

  return 0;
}

int RtspServer::Stop() {
  LOGD << "RtspServer::Stop()";
  if (rtsp_server_thread_ && rtsp_server_thread_->joinable()) {
    LOGW << "rtsp server thread destroy";
    watch_variable_ = 1;
    rtsp_server_thread_->join();
    rtsp_server_thread_ = nullptr;
  }
  return 0;
}

int RtspServer::DeInit() {
  return 0;
}

int RtspServer::SendData(unsigned char* buf, int buf_len,
               int media_type, int chn_id) {
  std::string stream_name = config_->GetChnStreamName(chn_id);
  if (media_type == H264 || media_type == H265) {
    if (buf_len >= 4 && buf[0] == 0 && buf[1] == 0
        && buf[2] == 0 && buf[3] == 1) {
      HorizonH264Or5FramedSource::OnH264Or5Frame(
        buf+4, buf_len-4, stream_name.c_str());
    } else if (buf_len >= 4 && buf[0] == 0 && buf[1] == 0 && buf[2] == 1) {
      HorizonH264Or5FramedSource::OnH264Or5Frame(
        buf+3, buf_len-3, stream_name.c_str());
    }
  } else if (media_type == G711) {
    // TODO(zrh)
  }
  return 0;
}

int RtspServer::RTSPServerRun() {
  LOGD << "RtspServer::RTSPServerRun";
  scheduler_ = BasicTaskScheduler::createNew();
  env_ = BasicUsageEnvironment::createNew(*scheduler_);

  UserAuthenticationDatabase *authDB = NULL;

  if (config_->auth_mode_) {
    authDB = new UserAuthenticationDatabase;
    authDB->addUserRecord(config_->user_.c_str(), config_->password_.c_str());
  }

  *env_ << "RTSP server use auth: " << config_->auth_mode_
       << "  username: " << config_->user_.c_str() << " password "
       << config_->password_.c_str() << "\n";
  OutPacketBuffer::maxSize = 2000000;

  portNumBits rtspServerPortNum = config_->port_;
  rtsp_server_ = RTSPServer::createNew(*env_, rtspServerPortNum, authDB);
  if (rtsp_server_ == NULL) {
    *env_ << "Failed to create RTSP Server: " << env_->getResultMsg() << "\n";
    exit(1);
  }

  for (int i = 0; i < config_->chn_num_; i++) {
    std::string stream_name = config_->GetChnStreamName(i);
    LOGD << " rtsp url = " << stream_name;

    ServerMediaSession *sms = ServerMediaSession::createNew(
      *env_, stream_name.c_str(), "horizon rtsp server", "horizon session");

    if (config_->video_type_[i] == H264) {
      sms->addSubsession(HorizonH264ServerMediaSubsession::createNew(*env_,
        True));
    } else if (config_->video_type_[i] == H265) {
      sms->addSubsession(HorizonH265ServerMediaSubsession::createNew(*env_,
        True));
    }

    if (config_->audio_type_[i] == G711) {
      // TODO(zrh)
      // add audio subsession
    }
    rtsp_server_->addServerMediaSession(sms);

    char *url = rtsp_server_->rtspURL(sms);
    *env_ << "Play this stream using the URL \n\t" << url << "\n";
    delete[] url;
  }

  env_->taskScheduler().doEventLoop(&watch_variable_);
  LOGD << "reclaim and delete scheduler...";
  Medium::close(rtsp_server_);
  env_->reclaim();
  delete scheduler_;
  rtsp_server_ = NULL;
  env_ = NULL;
  scheduler_ = NULL;
  return 0;
}
}  // namespace rtspcomponent
