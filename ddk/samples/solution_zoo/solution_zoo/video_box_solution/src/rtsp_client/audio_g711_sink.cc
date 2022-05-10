/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "rtsp_client/audio_g711_sink.h"

#include "hobotlog/hobotlog.hpp"

// Implementation of "AudioG711Sink":
#define AUDIO_RECEIVE_BUFFER_SIZE 2 * 1024
void AudioG711Sink::SetFileName(std::tuple<bool, std::string> file) {
  save_file_ = std::get<0>(file);
  file_name_ = std::get<1>(file);

  if (save_file_ && !file_name_.empty()) {
    outfile_.open(file_name_, std::ios::app | std::ios::out | std::ios::binary);
  }
}

int AudioG711Sink::SaveToFile(void *data, const int data_size) {
  if (outfile_.is_open()) {
    outfile_.write(reinterpret_cast<char *>(data), data_size);
  }

  return 0;
}

void AudioG711Sink::SetChannel(int channel) { channel_ = channel; }

int AudioG711Sink::GetChannel(void) const { return channel_; }

AudioG711Sink *AudioG711Sink::createNew(UsageEnvironment &env,
                                        MediaSubsession &subsession,
                                        char const *streamId, int buffer_size,
                                        int buffer_count) {
  return new AudioG711Sink(env, subsession, streamId, AUDIO_RECEIVE_BUFFER_SIZE,
                           buffer_count);
}

AudioG711Sink::AudioG711Sink(UsageEnvironment &env, MediaSubsession &subsession,
                             char const *streamId, int buffer_size,
                             int buffer_count)
    : MediaSink(env),
      subsession_(subsession),
      buffer_size_(buffer_size),
      buffer_count_(buffer_count),
      save_file_(false),
      channel_(-1),
      first_frame_(true),
      waiting_(true),
      frame_count_(0) {
  buffer_ = new char[AUDIO_RECEIVE_BUFFER_SIZE];
  // data_len_ = 0;
  stream_id_ = strDup(streamId);
}

AudioG711Sink::~AudioG711Sink() {
  LOGI << "AudioG711Sink::~AudioG711Sink(), channel:" << channel_;
  delete[] stream_id_;
  delete buffer_;

  if (outfile_.is_open()) {
    outfile_.close();
  }
  LOGI << "leave ~AudioG711Sink(), channel:" << channel_;
}

void AudioG711Sink::afterGettingFrame(void *clientData, unsigned frameSize,
                                      unsigned numTruncatedBytes,
                                      struct timeval presentationTime,
                                      unsigned durationInMicroseconds) {
  AudioG711Sink *sink = reinterpret_cast<AudioG711Sink *>(clientData);
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then
// comment out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 0

void AudioG711Sink::afterGettingFrame(unsigned frameSize,
                                      unsigned numTruncatedBytes,
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

  // data_len_ = frameSize;

  LOGI << "channle:" << channel_ << " recv 711 audio data";
  frame_count_++;
  // Then continue, to request the next frame of data:
  continuePlaying();
}

void AudioG711Sink::AddPipeline(
    std::shared_ptr<solution::video_box::MediaPipeline> pipe_line) {
  pipe_line_ = pipe_line;
}

Boolean AudioG711Sink::continuePlaying() {
  if (fSource == NULL) return False;  // sanity check (should not happen)

  fSource->getNextFrame((unsigned char *)buffer_, buffer_size_,
                        afterGettingFrame, this, onSourceClosure, this);
  return True;
}
