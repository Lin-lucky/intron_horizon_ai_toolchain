/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "rtsp_client/h264_sink.h"

#include "hb_comm_vdec.h"
#include "hb_vdec.h"
#include "hb_vp_api.h"
#include "hobotlog/hobotlog.hpp"
#include "rtsp_client/sps_info_mgr.h"

#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 200000
void H264Sink::SetFileName(std::tuple<bool, std::string> file) {
  save_file_ = std::get<0>(file);
  file_name_ = std::get<1>(file);

  if (save_file_ && !file_name_.empty()) {
    outfile_.open(file_name_, std::ios::app | std::ios::out | std::ios::binary);
  }
}

int H264Sink::SaveToFile(void *data, const int data_size) {
  if (outfile_.is_open()) {
    outfile_.write(reinterpret_cast<char *>(data), data_size);
  }

  return 0;
}

void H264Sink::SetChannel(int channel) { channel_ = channel; }

int H264Sink::GetChannel(void) const { return channel_; }

H264Sink *H264Sink::createNew(UsageEnvironment &env,
                              MediaSubsession &subsession, char const *streamId,
                              int buffer_size, int buffer_count) {
  return new H264Sink(env, subsession, streamId, buffer_size, buffer_count);
}

H264Sink::H264Sink(UsageEnvironment &env, MediaSubsession &subsession,
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
    LOGE << "H264Sink sys alloc failed";
  }

  video_buffer_ = new char[buffer_size_];
  buffer_len_ = buffer_size_;
  data_len_ = 0;
  if (video_buffer_ == NULL) {
  } else {
    memset(video_buffer_, 0, buffer_len_);
  }
}

H264Sink::~H264Sink() {
  LOGI << "H264Sink::~H264Sink(), channel:" << channel_;
  HB_SYS_Free(buffers_pyh_, buffers_vir_);
  LOGI << "~H264Sink(), channel:" << channel_ << " after HB_SYS_Free";
  // delete[] buffers_vir_;
  delete[] stream_id_;
  delete[] video_buffer_;

  if (outfile_.is_open()) {
    outfile_.close();
  }
  LOGI << "leave ~H264Sink(), channel:" << channel_;
}

void H264Sink::afterGettingFrame(void *clientData, unsigned frameSize,
                                 unsigned numTruncatedBytes,
                                 struct timeval presentationTime,
                                 unsigned durationInMicroseconds) {
  H264Sink *sink = reinterpret_cast<H264Sink *>(clientData);
  if (!sink) {
    LOGE << "H264Sink::afterGettingFrame get H264Sink is null, error!!!";
    return;
  }
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then
// comment out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 0

void H264Sink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
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
    if (save_file_) {
      SaveToFile(video_buffer_, data_len_);
    }

    if (ret != 0) {
      LOGE << "HB_VDEC_SendStream failed.";
    }

    data_len_ = 0;
    frame_count_++;
    first_frame_ = false;
    continuePlaying();
    return;
  }

  if (batch_send_) {
    for (auto cache : buffer_stat_cache_) {
      u_int8_t *buffer = buffers_vir_ + (cache.buffer_idx) * buffer_size_;
      uint64_t buffer_phy = buffers_pyh_ + (cache.buffer_idx) * buffer_size_;
#if DEBUG_PRINT_EACH_RECEIVED_FRAME
      printf("recv h264 grp %d  data: %02x %02x %02x %02x %02x\n\n",
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

void H264Sink::AddPipeline(
    std::shared_ptr<solution::video_box::MediaPipeline> pipe_line) {
  pipe_line_ = pipe_line;
}

Boolean H264Sink::isNeedToWait(unsigned frameSize) {
  if (first_frame_) {
    LOGW << "channel:" << channel_ << " recv stream";
    if (data_len_ == 4) {  // first nal
      data_len_ += frameSize;
      int nNal1 = 0;
      nNal1 |= (video_buffer_[4] & 0x1f);
      LOGW << "channel:" << channel_ << " recv nal 1:" << nNal1;
      if (nNal1 == 0x07) {
        LOGW << "channel:" << channel_ << " to analytics sps info";
        int width = 0;
        int height = 0;
        solution::video_box::SPSInfoMgr::GetInstance().AnalyticsSps(
            (unsigned char *)video_buffer_ + 4, data_len_ - 4, width, height);
        pipe_line_->SetDecodeResolution(width, height);
        pipe_line_->Init();
        pipe_line_->Start();
      }
      return true;
    }

    char *buffer = video_buffer_ + data_len_;
    int nNal1 = 0;
    int nNal2 = 0;
    nNal1 |= (video_buffer_[4] & 0x1f);
    nNal2 |= (buffer[0] & 0x1f);
    LOGW << "channle:" << channel_
         << ", first frame recv h264 nal type1:" << nNal1
         << "  type2:" << nNal2;

    if (nNal1 != 0x07 || nNal2 != 0x08) {
      data_len_ = 0;
      return true;
    }

    data_len_ += frameSize;
    return false;
  }

  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
  int nNalUnitType = 0;
  nNalUnitType |= (buffer[4] & 0x1f);
  if (nNalUnitType == 0x07 || nNalUnitType == 0x08) {
    buffer_stat_t stat;
    stat.buffer_idx = frame_count_ % buffer_count_;
    stat.frame_size = frameSize;
    buffer_stat_cache_.emplace_back(stat);
    if (nNalUnitType == 0x08) {
      if (buffer_stat_cache_.size() == 2) {
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

Boolean H264Sink::continuePlaying() {
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
