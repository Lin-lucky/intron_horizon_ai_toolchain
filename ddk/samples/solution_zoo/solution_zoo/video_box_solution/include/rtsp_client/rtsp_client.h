/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#ifndef INCLUDE_RTSPCLIENT_SMARTPLUGIN_H_
#define INCLUDE_RTSPCLIENT_SMARTPLUGIN_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <tuple>

#include "BasicUsageEnvironment.hh"
#include "H264VideoRTPSource.hh"
#include "liveMedia.hh"
#include "media_pipe_manager/media_pipe_manager.h"
#include "media_pipe_manager/media_pipeline.h"
#include "media_pipe_manager/basic_media_module.h"

// Define a class to hold per-stream state that we maintain throughout each
// stream's lifetime:
class StreamClientState {
public:
  StreamClientState();
  virtual ~StreamClientState();

public:
  MediaSubsessionIterator *iter;
  MediaSession *session;
  MediaSubsession *subsession;
  TaskToken streamTimerTask;
  double duration;
};

// If you're streaming just a single stream (i.e., just from a single URL,
// once), then you can define and use just a single "StreamClientState"
// structure, as a global variable in your application.  However, because - in
// this demo application - we're showing how to play multiple streams,
// concurrently, we can't do that.  Instead, we have to have a separate
// "StreamClientState" structure for each "RTSPClient".  To do this, we subclass
// "RTSPClient", and add a "StreamClientState" field to the subclass:

class ourRTSPClient : public RTSPClient {
 public:
  static ourRTSPClient *createNew(UsageEnvironment &env, char const *rtspURL,
                                  int verbosityLevel = 0,
                                  char const *applicationName = NULL,
                                  portNumBits tunnelOverHTTPPortNum = 0);
  void SetOutputFileName(const bool save_stream, const std::string &file_name);
  std::tuple<bool, std::string> GetOutputFileName(void);
  void SetChannel(int channel);
  int GetChannel(void) const;
  // virtual ~ourRTSPClient();

  void SetTCPFlag(const bool flag) { tcp_flag_ = flag; }
  bool GetTCPFlag() { return tcp_flag_; }

  void SetFrameMaxSize(const int frame_max_size) {
    frame_max_size_ = frame_max_size;
  }
  int GetFrameMaxSize() { return frame_max_size_; }

  void Stop();

 protected:
  ourRTSPClient(UsageEnvironment &env, char const *rtspURL, int verbosityLevel,
                char const *applicationName, portNumBits tunnelOverHTTPPortNum);
  // called only by createNew();
  virtual ~ourRTSPClient();

 public:
  StreamClientState scs;
  int channel_;

 private:
  std::string file_name_;
  bool tcp_flag_;
  int frame_max_size_;
  bool has_shut_down;
  bool save_stream_;
};

// The main streaming routine (for each "rtsp://" URL):
ourRTSPClient *openURL(UsageEnvironment &env, char const *progName,
                       char const *rtspURL, const bool tcp_flag,
                       const int frame_max_size,
                       const std::string &file_name, const bool save_stream,
                       int channel);
void shutdownStream(RTSPClient *rtspClient, int exitCode = 1);

#endif  // INCLUDE_RTSPCLIENT_SMARTPLUGIN_H_