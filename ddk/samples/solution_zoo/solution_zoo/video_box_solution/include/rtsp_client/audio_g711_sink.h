/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_RTSPCLIENT_AUDIOG711SINK_H_
#define INCLUDE_RTSPCLIENT_AUDIOG711SINK_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "BasicUsageEnvironment.hh"
#include "liveMedia.hh"
#include "media_pipe_manager/media_pipe_manager.h"
#include "media_pipe_manager/media_pipeline.h"

// Define a data sink (a subclass of "MediaSink") to receive the data for each
// subsession (i.e., each audio or video 'substream'). In practice, this might
// be a class (or a chain of classes) that decodes and then renders the incoming
// audio or video. Or it might be a "FileSink", for outputting the received data
// into a file (as is done by the "openRTSP" application). In this example code,
// however, we define a simple 'dummy' sink that receives incoming data, but
// does nothing with it.

class AudioG711Sink : public MediaSink {
 public:
  static AudioG711Sink *createNew(
      UsageEnvironment &env,
      MediaSubsession
          &subsession,  // identifies the kind of data that's being received
      char const *streamId = NULL, int buffer_size = 2048,
      int buffer_count = 2);  // identifies the stream itself (optional)

  virtual ~AudioG711Sink();
  void SetFileName(std::tuple<bool, std::string> file);
  int SaveToFile(void *data, const int data_siz);
  void SetChannel(int channel);
  int GetChannel(void) const;
  void AddPipeline(
      std::shared_ptr<solution::video_box::MediaPipeline> pipe_line);
  void Stop();

 private:
  AudioG711Sink(UsageEnvironment &env, MediaSubsession &subsession,
                char const *streamId, int buffer_size, int buffer_count);
  // called only by "createNew()"
  // virtual ~AudioSink();

  static void afterGettingFrame(void *clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                         struct timeval presentationTime,
                         unsigned durationInMicroseconds);

 private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();

 private:
  MediaSubsession &subsession_;
  int buffer_size_;
  int buffer_count_;

  char *stream_id_;
  std::string file_name_;
  bool save_file_;
  std::ofstream outfile_;
  int channel_;
  bool first_frame_;
  bool waiting_;
  uint64_t frame_count_;

  char *buffer_;
  // int data_len_;
  std::shared_ptr<solution::video_box::MediaPipeline> pipe_line_;
};
#endif
