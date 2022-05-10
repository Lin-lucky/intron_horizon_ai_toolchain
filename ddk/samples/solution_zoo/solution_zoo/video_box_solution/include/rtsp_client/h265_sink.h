/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_RTSPCLIENT_H265SINK_H_
#define INCLUDE_RTSPCLIENT_H265SINK_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "BasicUsageEnvironment.hh"
#include "H265VideoRTPSource.hh"
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

class H265Sink : public MediaSink {
 public:
  static H265Sink *createNew(
      UsageEnvironment &env,
      MediaSubsession
          &subsession,  // identifies the kind of data that's being received
      char const *streamId = NULL, int buffer_size = 200000,
      int buffer_count = 8);  // identifies the stream itself (optional)

  virtual ~H265Sink();
  void SetFileName(std::tuple<bool, std::string> file);
  int SaveToFile(void *data, const int data_siz);
  void SetChannel(int channel);
  int GetChannel(void) const;
  void AddPipeline(
    std::shared_ptr<solution::video_box::MediaPipeline> pipe_line);
  void Stop();

 private:
  H265Sink(UsageEnvironment &env, MediaSubsession &subsession,
           char const *streamId, int buffer_size, int buffer_count);
  // called only by "createNew()"
  // virtual ~H265Sink();

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
  Boolean isNeedToWait(unsigned frameSize);

 private:
  MediaSubsession &subsession_;
  int buffer_size_;
  int buffer_count_;
  u_int8_t *buffers_vir_;
  uint64_t buffers_pyh_;

  char *stream_id_;
  std::string file_name_;
  bool save_file_;
  std::ofstream outfile_;
  int channel_;
  bool first_frame_;
  bool waiting_;
  uint64_t frame_count_;
  typedef struct buffer_stat_s {
    int buffer_idx;
    int frame_size;
  } buffer_stat_t;
  std::vector<buffer_stat_t> buffer_stat_cache_;
  bool batch_send_ = false;
  std::shared_ptr<solution::video_box::MediaPipeline> pipe_line_;

  char *video_buffer_;
  int buffer_len_;
  int data_len_;
  int vps_len_ = 0;
  int sps_len_ = 0;
};
#endif
