/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "rtsp_client/h265_sink.h"

#include "hb_comm_vdec.h"
#include "hb_vdec.h"
#include "hb_vp_api.h"
#include "hobotlog/hobotlog.hpp"
#include "rtsp_client/sps_info_mgr.h"

// Implementation of "H265Sink":
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 200000
void H265Sink::SetFileName(std::tuple<bool, std::string> file) {
  save_file_ = std::get<0>(file);
  file_name_ = std::get<1>(file);

  if (save_file_ && !file_name_.empty()) {
    outfile_.open(file_name_, std::ios::app | std::ios::out | std::ios::binary);
  }
}

int H265Sink::SaveToFile(void *data, const int data_size) {
  if (outfile_.is_open()) {
    outfile_.write(reinterpret_cast<char *>(data), data_size);
  }

  return 0;
}

void H265Sink::SetChannel(int channel) { channel_ = channel; }

int H265Sink::GetChannel(void) const { return channel_; }

H265Sink *H265Sink::createNew(UsageEnvironment &env,
                              MediaSubsession &subsession, char const *streamId,
                              int buffer_size, int buffer_count) {
  return new H265Sink(env, subsession, streamId, buffer_size, buffer_count);
}

H265Sink::H265Sink(UsageEnvironment &env, MediaSubsession &subsession,
                   char const *streamId, int buffer_size, int buffer_count)
    : MediaSink(env),
      subsession_(subsession),
      buffer_size_(buffer_size),
      buffer_count_(buffer_count),
      save_file_(false),
      channel_(-1),
      first_frame_(true),
      waiting_(true),
      frame_count_(0) {
  int ret = 0;
  stream_id_ = strDup(streamId);
  ret = HB_SYS_Alloc(&buffers_pyh_, reinterpret_cast<void **>(&buffers_vir_),
                     buffer_size_ * buffer_count_);
  if (ret != 0) {
    LOGE << "H265Sink sys alloc failed";
  }

  video_buffer_ = new char[buffer_size_];
  buffer_len_ = buffer_size_;
  data_len_ = 0;
  if (video_buffer_ == NULL) {
  } else {
    memset(video_buffer_, 0, buffer_len_);
  }
}

H265Sink::~H265Sink() {
  LOGI << "H265Sink::~H265Sink(), channel:" << channel_;
  HB_SYS_Free(buffers_pyh_, buffers_vir_);
  LOGI << "~H265Sink(), channel:" << channel_ << " after HB_SYS_Free";
  // delete[] buffers_vir_;
  delete[] stream_id_;
  delete[] video_buffer_;

  if (outfile_.is_open()) {
    outfile_.close();
  }
  LOGI << "leave ~H265Sink(), channel:" << channel_;
}

void H265Sink::afterGettingFrame(void *clientData, unsigned frameSize,
                                 unsigned numTruncatedBytes,
                                 struct timeval presentationTime,
                                 unsigned durationInMicroseconds) {
  H265Sink *sink = reinterpret_cast<H265Sink *>(clientData);
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then
// comment out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 0

void H265Sink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                                 struct timeval presentationTime,
                                 unsigned /*durationInMicroseconds*/) {
  // We've just received a frame of data.  (Optionally) print out information
  // about it:
#if DEBUG_PRINT_EACH_RECEIVED_FRAME
  if (stream_id_ != NULL) envir() << "Stream \"" << stream_id_ << "\"; ";
  envir() << subsession_.mediumName() << "/" << subsession_.codecName()
          << ":\tReceived " << frameSize << " bytes";
  if (numTruncatedBytes > 0)
    envir() << " (with " << numTruncatedBytes << " bytes truncated)";
  char uSecsStr[6 + 1];  // used to output the 'microseconds' part of the
                         // presentation time
  snprintf(uSecsStr, sizeof(uSecsStr), "%06u",
           (unsigned)presentationTime.tv_usec);
  envir() << ".\tPresentation time: "
          << reinterpret_cast<int>(presentationTime.tv_sec) << "." << uSecsStr;
  if (subsession_.rtpSource() != NULL &&
      !subsession_.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
    envir() << "!";  // mark the debugging output to indicate that this
                     // presentation time is not RTCP-synchronized
  }
#ifdef DEBUG_PRINT_NPT
  envir() << "\tNPT: " << subsession_.getNormalPlayTime(presentationTime);
#endif
  envir() << "\n";
#endif
  if (!buffers_vir_) return;
  // unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  waiting_ = isNeedToWait(frameSize);
  if (waiting_) {
    if (!first_frame_) {
      frame_count_++;
    }
    // Then continue, to request the next frame of data:
    continuePlaying();
    return;
  }

  if (first_frame_) {
    u_int8_t *buffer =
        buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
    uint64_t buffer_phy =
        buffers_pyh_ + (frame_count_ % buffer_count_) * buffer_size_;
    memcpy(buffer, video_buffer_, data_len_);
    VIDEO_STREAM_S pstStream;
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
    pstStream.pstPack.phy_ptr = buffer_phy;
    pstStream.pstPack.vir_ptr = reinterpret_cast<char *>(buffer);
    pstStream.pstPack.pts = frame_count_;
    pstStream.pstPack.src_idx = frame_count_ % buffer_count_;
    pstStream.pstPack.size = data_len_;
    pstStream.pstPack.stream_end = HB_FALSE;
    int ret = pipe_line_->Input(&pstStream);
    if (ret != 0) {
      LOGE << "HB_VDEC_SendStream failed, ret:" << ret;
    }

    // memset(video_buffer_, 0, buffer_len_);
    data_len_ = 0;
    frame_count_++;
    first_frame_ = false;
    continuePlaying();
    return;
  }

  if (batch_send_) {
    for (auto cache : buffer_stat_cache_) {
      LOGW << "batch_send_";
      u_int8_t *buffer = buffers_vir_ + (cache.buffer_idx) * buffer_size_;
      uint64_t buffer_phy = buffers_pyh_ + (cache.buffer_idx) * buffer_size_;
#if DEBUG_PRINT_EACH_RECEIVED_FRAME
      printf("recv h265 grp %d  data: %02x %02x %02x %02x %02x\n\n",
             pipe_line_->GetGrpId(), buffer[0], buffer[1], buffer[2], buffer[3],
             buffer[4]);
#endif
      VIDEO_STREAM_S pstStream;
      memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
      pstStream.pstPack.phy_ptr = buffer_phy;
      pstStream.pstPack.vir_ptr = reinterpret_cast<char *>(buffer);
      //      pstStream.pstPack.pts = frame_count_;
      pstStream.pstPack.src_idx = cache.buffer_idx;
      pstStream.pstPack.size = cache.frame_size + 4;
      pstStream.pstPack.stream_end = HB_FALSE;
      int ret = 0;
      ret = pipe_line_->Input(&pstStream);
      if (save_file_) {
        SaveToFile(buffer, cache.frame_size + 4);
      }
      if (ret != 0) {
        LOGE << "pipeline in failed  grp:" << pipe_line_->GetGrpId()
             << "  ret:" << ret;
      }
    }

    batch_send_ = false;
    buffer_stat_cache_.clear();
    frame_count_++;
    // Then continue, to request the next frame of data:
    continuePlaying();
    return;
  }

  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
  uint64_t buffer_phy =
      buffers_pyh_ + (frame_count_ % buffer_count_) * buffer_size_;

  VIDEO_STREAM_S pstStream;
  memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
  pstStream.pstPack.phy_ptr = buffer_phy;
  pstStream.pstPack.vir_ptr = reinterpret_cast<char *>(buffer);
  pstStream.pstPack.pts = frame_count_;
  pstStream.pstPack.src_idx = frame_count_ % buffer_count_;
  pstStream.pstPack.size = frameSize + 4;
  pstStream.pstPack.stream_end = HB_FALSE;
  int ret = 0;
  ret = pipe_line_->Input(&pstStream);
  if (save_file_) {
    SaveToFile(buffer, frameSize + 4);
  }
  if (ret != 0) {
    LOGE << "pipeline in failed  grp:" << pipe_line_->GetGrpId()
         << "  ret:" << ret;
  }

  frame_count_++;
  // Then continue, to request the next frame of data:
  continuePlaying();
}

void H265Sink::AddPipeline(
    std::shared_ptr<solution::video_box::MediaPipeline> pipe_line) {
  pipe_line_ = pipe_line;
}

Boolean H265Sink::isNeedToWait(unsigned frameSize) {
  if (first_frame_) {
    LOGW << "channel:" << channel_ << " recv stream";
    int nNal1 = 0;
    nNal1 = (video_buffer_[4] & 0x7E) >> 1;
    if (nNal1 != 32) {  // not vps
      data_len_ = 0;
      return true;
    }
    if (data_len_ == 4) {  // first nal
      LOGW << "channle:" << channel_
           << ", first frame recv h265 nal1 type:" << nNal1
           << " frame size:" << frameSize;
      data_len_ += frameSize;
      vps_len_ = data_len_;
      LOGW << "channle:" << channel_ << "vps len:" << vps_len_;
      return true;
    }

    int nNal2 = 0;
    char *buffer = video_buffer_ + vps_len_;
    nNal2 = (buffer[4] & 0x7E) >> 1;
    if (nNal2 != 33) {  // sps
      data_len_ = 0;
      vps_len_ = 0;
      sps_len_ = 0;
      return true;
    }

    if (sps_len_ == 0 && vps_len_ > 0) {
      LOGW << "channle:" << channel_
           << ", first frame recv h265 nal2 type:" << nNal2
           << " framesize:" << frameSize;
      int width = 0;
      int height = 0;
      solution::video_box::SPSInfoMgr::GetInstance().AnalyticsSps(
          (unsigned char *)video_buffer_ + data_len_, frameSize, width, height,
          "H265");
      pipe_line_->SetDecodeResolution(width, height);
      pipe_line_->Init();
      pipe_line_->Start();
      data_len_ += frameSize;
      sps_len_ = data_len_;
      LOGW << "channle:" << channel_ << "sps len:" << sps_len_;
      return true;
    }

    buffer = video_buffer_ + sps_len_;
    int nal_type = (buffer[4] & 0x7E) >> 1;
    data_len_ += frameSize;
    LOGW << "channle:" << channel_
         << ", first frame recv h265 nal3 type:" << nal_type;
    if (nal_type == 34) {  // pps
      return false;
    } else {
      data_len_ = 0;
      vps_len_ = 0;
      sps_len_ = 0;
      return true;
    }
  }

  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
  int nal_type = (buffer[4] & 0x7E) >> 1;
  // h265 nale type: VPS=32 SPS=33 PPS=34 SEI=39 IDR=19 P=1 B=0
  // h265 normal i frame: 32, 33, 34, (39), 19
  if (nal_type == 32 || nal_type == 33 || nal_type == 34) {
    buffer_stat_t stat;
    stat.buffer_idx = frame_count_ % buffer_count_;
    stat.frame_size = frameSize;
    buffer_stat_cache_.emplace_back(stat);
    if (nal_type == 34) {  // pps
      if (buffer_stat_cache_.size() == 3) {
        batch_send_ = true;
        LOGI << "stat done";
        return false;
      } else {
        LOGE << "stat error, cache size:" << buffer_stat_cache_.size();
        buffer_stat_cache_.clear();
        batch_send_ = false;
        return true;
      }
    }
    return true;
  }

  return false;
}

Boolean H265Sink::continuePlaying() {
  if (fSource == NULL) return False;  // sanity check (should not happen)
  static unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};

  if (first_frame_) {
    memcpy(video_buffer_ + data_len_, start_code, 4);
    data_len_ += 4;
    fSource->getNextFrame((unsigned char *)video_buffer_ + data_len_,
                          buffer_len_ - data_len_, afterGettingFrame, this,
                          onSourceClosure, this);
    return True;
  }

  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
  // Request the next frame of data from our input source. "afterGettingFrame()"
  // will get called later, when it arrives:
  memcpy(reinterpret_cast<void *>(buffer), start_code, 4);
  buffer += 4;

  fSource->getNextFrame(buffer, buffer_size_, afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}
