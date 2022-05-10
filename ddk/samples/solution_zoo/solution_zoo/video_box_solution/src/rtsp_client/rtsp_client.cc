/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */
#include "rtsp_client/rtsp_client.h"

#include "hobotlog/hobotlog.hpp"
#include "media_pipe_manager/media_pipe_manager.h"
#include "rtsp_client/audio_g711_sink.h"
#include "rtsp_client/h264_sink.h"
#include "rtsp_client/h265_sink.h"
#include "rtsp_client/sps_info_mgr.h"

// Forward function definitions:

// RTSP 'response handlers':
void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode,
                           char *resultString);
void continueAfterSETUP(RTSPClient *rtspClient, int resultCode,
                        char *resultString);
void continueAfterPLAY(RTSPClient *rtspClient, int resultCode,
                       char *resultString);

// Other event handler functions:
void subsessionAfterPlaying(
    void *clientData);  // called when a stream's subsession (e.g., audio or
                        // video substream) ends
void subsessionByeHandler(
    void *clientData);  // called when a RTCP "BYE" is received for a subsession
void streamTimerHandler(void *clientData);
// called at the end of a stream's expected duration (if the stream has not
// already signaled its end using a RTCP "BYE")

// Used to iterate through each stream's 'subsessions', setting up each one:
void setupNextSubsession(RTSPClient *rtspClient);

// Used to shut down and close a stream (including its "RTSPClient" object):
// void shutdownStream(RTSPClient* rtspClient, int exitCode = 1);

// A function that outputs a string that identifies each stream (for debugging
// output).  Modify this if you wish:
UsageEnvironment &operator<<(UsageEnvironment &env,
                             const RTSPClient &rtspClient) {
  return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for
// debugging output).  Modify this if you wish:
UsageEnvironment &operator<<(UsageEnvironment &env,
                             const MediaSubsession &subsession) {
  return env << subsession.mediumName() << "/" << subsession.codecName();
}

// Implementation of "StreamClientState":

StreamClientState::StreamClientState()
    : iter(NULL),
      session(NULL),
      subsession(NULL),
      streamTimerTask(NULL),
      duration(0.0) {}

StreamClientState::~StreamClientState() {
  delete iter;
  if (session != NULL) {
    // We also need to delete "session", and unschedule "streamTimerTask" (if
    // set)
    UsageEnvironment &env = session->envir();  // alias

    env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
    Medium::close(session);
  }
}

// Implementation of "ourRTSPClient":
void ourRTSPClient::SetOutputFileName(const bool save_stream,
                                      const std::string &file_name) {
  file_name_ = file_name;
  save_stream_ = save_stream;
};

std::tuple<bool, std::string> ourRTSPClient::GetOutputFileName(void) {
  return std::make_tuple(save_stream_, file_name_);
};

void ourRTSPClient::SetChannel(int channel) { channel_ = channel; };

int ourRTSPClient::GetChannel(void) const { return channel_; };

ourRTSPClient *ourRTSPClient::createNew(UsageEnvironment &env,
                                        char const *rtspURL, int verbosityLevel,
                                        char const *applicationName,
                                        portNumBits tunnelOverHTTPPortNum) {
  return new ourRTSPClient(env, rtspURL, verbosityLevel, applicationName,
                           tunnelOverHTTPPortNum);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment &env, char const *rtspURL,
                             int verbosityLevel, char const *applicationName,
                             portNumBits tunnelOverHTTPPortNum)
    : RTSPClient(env, rtspURL, verbosityLevel, applicationName,
                 tunnelOverHTTPPortNum, -1) {
  tcp_flag_ = false;
  has_shut_down = false;
  save_stream_ = false;
}

ourRTSPClient::~ourRTSPClient() {
  LOGI << "channel " << channel_ << " ~ourRTSPClient()";
  auto media =
  solution::video_box::MediaPipeManager::GetInstance().GetPipeline()[channel_];
  if (media) {
    media->Stop();
    LOGI << "~ourRTSPClient(), call media pipeline deinit, channel:"
         << channel_;
    media->DeInit();
  }
  LOGI << "leave ourRTSPClient::~ourRTSPClient(), channel:" << channel_;
}

void ourRTSPClient::Stop() {
  LOGI << "call ourRTSPClient::Stop(), channel:" << channel_;
  if (!has_shut_down) {
    shutdownStream(this);
    has_shut_down = true;
  }

  LOGI << "ourRTSPClient Stop(), call media pipeline deinit, channel:"
       << channel_;
  auto media =
  solution::video_box::MediaPipeManager::GetInstance().GetPipeline()[channel_];
  if (media) {
    media->Stop();
    media->DeInit();
  }
  LOGI << "leave call ourRTSPClient::Stop(), channel:" << channel_;
}

#define RTSP_CLIENT_VERBOSITY_LEVEL \
  1  // by default, print verbose output from each "RTSPClient"

static unsigned rtspClientCount =
    0;  // Counts how many streams (i.e., "RTSPClient"s) are currently in use.

static void copyUsernameOrPasswordStringFromURL(char *dest, char const *src,
                                                unsigned len) {
  // Normally, we just copy from the source to the destination.  However, if the
  // source contains
  // %-encoded characters, then we decode them while doing the copy:
  while (len > 0) {
    int nBefore = 0;
    int nAfter = 0;

    if (*src == '%' && len >= 3 &&
        sscanf(src + 1, "%n%2hhx%n", &nBefore, dest, &nAfter) == 1) {
      unsigned codeSize = nAfter - nBefore;  // should be 1 or 2

      ++dest;
      src += (1 + codeSize);
      len -= (1 + codeSize);
    } else {
      *dest++ = *src++;
      --len;
    }
  }
  *dest = '\0';
}

// Parse the URL username and password, return url without username and password
static void parseRTSPURL(char const *url, char *&username, char *&password,
                         char *&urlNoPassword) {
  unsigned prefixLength = 0;
  char const *from = url;

  do {
    // Parse the URL as
    // "rtsp://[<username>[:<password>]@]<server-address-or-name>[:<port>][/<stream-name>]"
    // (or "rtsps://...")
    char const *prefix1 = "rtsp://";
    unsigned const prefix1Length = 7;
    char const *prefix2 = "rtsps://";
    unsigned const prefix2Length = 8;

    if (strncasecmp(url, prefix1, prefix1Length) == 0) {
      prefixLength = prefix1Length;
    } else if (strncasecmp(url, prefix2, prefix2Length) == 0) {
      prefixLength = prefix2Length;
    } else {
      break;
    }

    from = &url[prefixLength];

    // Check whether "<username>[:<password>]@" occurs next.
    // We do this by checking whether '@' appears before the end of the URL, or
    // before the first '/'.
    username = password = NULL;  // default return values
    char const *colonPasswordStart = NULL;
    char const *lastAtPtr = NULL;
    for (char const *p = from; *p != '\0' && *p != '/'; ++p) {
      if (*p == ':' && colonPasswordStart == NULL) {
        colonPasswordStart = p;
      } else if (*p == '@') {
        lastAtPtr = p;
      }
    }

    if (lastAtPtr != NULL) {
      // We found <username> (and perhaps <password>).  Copy them into
      // newly-allocated result strings:
      if (colonPasswordStart == NULL || colonPasswordStart > lastAtPtr)
        colonPasswordStart = lastAtPtr;

      char const *usernameStart = from;
      unsigned usernameLen = colonPasswordStart - usernameStart;
      username = new char[usernameLen + 1];  // allow for the trailing '\0'
      copyUsernameOrPasswordStringFromURL(username, usernameStart, usernameLen);

      char const *passwordStart = colonPasswordStart;
      if (passwordStart < lastAtPtr) ++passwordStart;  // skip over the ':'
      unsigned passwordLen = lastAtPtr - passwordStart;
      password = new char[passwordLen + 1];  // allow for the trailing '\0'
      copyUsernameOrPasswordStringFromURL(password, passwordStart, passwordLen);

      from = lastAtPtr + 1;  // skip over the '@'
    }
  } while (0);

  urlNoPassword = new char[strlen(url) + 1];
  strncpy(urlNoPassword, url, prefixLength);
  strncat(urlNoPassword + prefixLength, from, strlen(from));
}

ourRTSPClient *openURL(UsageEnvironment &env, char const *progName,
                       char const *rtspURL, const bool tcp_flag,
                       const int frame_max_size, const std::string &file_name,
                       const bool save_stream, int channel) {
  char *username = NULL;
  char *password = NULL;
  char *urlWithoutPassword = NULL;

  parseRTSPURL(rtspURL, username, password, urlWithoutPassword);

  // Begin by creating a "RTSPClient" object.  Note that there is a separate
  // "RTSPClient" object for each stream that we wish to receive (even if more
  // than stream uses the same "rtsp://" URL).
  ourRTSPClient *rtspClient = ourRTSPClient::createNew(
      env, urlWithoutPassword, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
  if (rtspClient == NULL) {
    env << "Failed to create a RTSP client for URL \"" << rtspURL
        << "\": " << env.getResultMsg() << "\n";
    return nullptr;
  }
  rtspClient->SetOutputFileName(save_stream, file_name);
  rtspClient->SetChannel(channel);
  rtspClient->SetTCPFlag(tcp_flag);
  rtspClient->SetFrameMaxSize(frame_max_size);

  env << "Set output file name:" << file_name.c_str() << "\n";

  ++rtspClientCount;

  // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the
  // stream. Note that this command - like all RTSP commands - is sent
  // asynchronously; we do not block, waiting for a response. Instead, the
  // following function call returns immediately, and we handle the RTSP
  // response later, from within the event loop:
  Authenticator rtspAuthenticator(username, password);
  rtspClient->sendDescribeCommand(continueAfterDESCRIBE, &rtspAuthenticator);

  if (username) delete[] username;
  if (password) delete[] password;
  if (urlWithoutPassword) delete[] urlWithoutPassword;

  return rtspClient;
}

// Implementation of the RTSP 'response handlers':

void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode,
                           char *resultString) {
  ourRTSPClient * client = dynamic_cast<ourRTSPClient *>(rtspClient);
  HOBOT_CHECK(client);
  LOGD << "continueAfterDESCRIBE resultCode:" << resultCode
       << "  channel:" << client->channel_;
  do {
    UsageEnvironment &env = rtspClient->envir();  // alias
    StreamClientState &scs =
        (reinterpret_cast<ourRTSPClient *>(rtspClient))->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to get a SDP description: " << resultString
          << "\n";
      delete[] resultString;
      break;
    }

    char *const sdpDescription = resultString;
    env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdpDescription);
    delete[] sdpDescription;  // because we don't need it anymore
    if (scs.session == NULL) {
      env << *rtspClient
          << "Failed to create a MediaSession object from the SDP description: "
          << env.getResultMsg() << "\n";
      break;
    } else if (!scs.session->hasSubsessions()) {
      env << *rtspClient
          << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
      break;
    }

    // Then, create and set up our data source objects for the session.  We do
    // this by iterating over the session's 'subsessions', calling
    // "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command,
    // on each one. (Each 'subsession' will have its own data source.)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    auto media = solution::video_box::MediaPipeManager::GetInstance()
                     .GetPipeline()[client->GetChannel()];
    media->SetDecodeType(solution::video_box::RTSP_Payload_H264);
    LOGW << "channel:" << client->GetChannel() << " recv describe success!!!";
    // ad
    setupNextSubsession(rtspClient);
    return;
  } while (0);

  // An unrecoverable error occurred with this stream.
  shutdownStream(rtspClient);
}

static unsigned getBufferSize_(UsageEnvironment &env, int bufOptName,
                               int socket) {
  int curSize = 0;
  int optlen = sizeof(int);
  // SOCKLEN_T sizeSize = sizeof curSize;
  if (getsockopt(socket, SOL_SOCKET, bufOptName,
                 reinterpret_cast<char *>(&curSize),
                 reinterpret_cast<socklen_t *>(&optlen)) < 0) {
    LOGE << "rtsp getBufferSize error!!!";
    return 0;
  }

  return curSize;
}

unsigned getReceiveBufferSize_(UsageEnvironment &env, int socket) {
  return getBufferSize_(env, SO_RCVBUF, socket);
}

static unsigned setBufferTo_(UsageEnvironment &env, int bufOptName, int socket,
                             unsigned requestedSize) {
  SOCKLEN_T sizeSize = sizeof requestedSize;
  setsockopt(socket, SOL_SOCKET, bufOptName,
             reinterpret_cast<char *>(&requestedSize), sizeSize);

  // Get and return the actual, resulting buffer size:
  return getBufferSize_(env, bufOptName, socket);
}

unsigned setReceiveBufferTo_(UsageEnvironment &env, int socket,
                             unsigned requestedSize) {
  return setBufferTo_(env, SO_RCVBUF, socket, requestedSize);
}

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP-over-TCP,
// change the following to True:
#define REQUEST_STREAMING_OVER_TCP False

void setupNextSubsession(RTSPClient *rtspClient) {
  UsageEnvironment &env = rtspClient->envir();  // alias
  StreamClientState &scs =
      (reinterpret_cast<ourRTSPClient *>(rtspClient))->scs;  // alias
  bool tcp_flag = (reinterpret_cast<ourRTSPClient *>(rtspClient))->GetTCPFlag();
  LOGW << "channel:"
       << (reinterpret_cast<ourRTSPClient *>(rtspClient))->GetChannel()
       << " begin to setup to ipc";
  scs.subsession = scs.iter->next();
  if (scs.subsession != NULL) {
    if (!scs.subsession->initiate()) {
      env << *rtspClient << "Failed to initiate the \"" << *scs.subsession
          << "\" subsession: " << env.getResultMsg() << "\n";
      setupNextSubsession(
          rtspClient);  // give up on this subsession; go to the next one
    } else {
      env << *rtspClient << "Initiated the \"" << *scs.subsession
          << "\" subsession (";
      if (scs.subsession->rtcpIsMuxed()) {
        env << "client port " << scs.subsession->clientPortNum();
      } else {
        env << "client ports " << scs.subsession->clientPortNum() << "-"
            << scs.subsession->clientPortNum() + 1;
      }
      env << ")\n";

      // Continue setting up this subsession, by sending a RTSP "SETUP" command:
      rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False,
                                   tcp_flag);
    }
    return;
  }

  // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY"
  // command to start the streaming:
  if (scs.session->absStartTime() != NULL) {
    // Special case: The stream is indexed by 'absolute' time, so send an
    // appropriate "PLAY" command:
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY,
                                scs.session->absStartTime(),
                                scs.session->absEndTime());
  } else {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
  }
}

void continueAfterSETUP(RTSPClient *rtspClient, int resultCode,
                        char *resultString) {
  do {
    UsageEnvironment &env = rtspClient->envir();  // alias
    ourRTSPClient *our_rtsp_client =
        reinterpret_cast<ourRTSPClient *>(rtspClient);
    StreamClientState &scs = our_rtsp_client->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to set up the \"" << *scs.subsession
          << "\" subsession: " << resultString << "\n";
      break;
    }
    std::string media_name(scs.subsession->mediumName());
    if (media_name.compare("video") != 0) {
      env << *rtspClient << "Skip " << scs.subsession->mediumName()
          << " stream";
      break;
    }

    env << *rtspClient << "Set up the \"" << *scs.subsession
        << "\" subsession (";
    if (scs.subsession->rtcpIsMuxed()) {
      env << "client port " << scs.subsession->clientPortNum();
    } else {
      env << "client ports " << scs.subsession->clientPortNum() << "-"
          << scs.subsession->clientPortNum() + 1;
    }
    env << ")\n";

    // Having successfully setup the subsession, create a data sink for it, and
    // call "startPlaying()" on it. (This will prepare the data sink to receive
    // data; the actual flow of data from the client won't start happening until
    // later, after we've sent a RTSP "PLAY" command.)
    int pay_load = solution::video_box::RTSP_Payload_NONE;
    if (strcmp(scs.subsession->mediumName(), "video") == 0) {
      if (strcmp(scs.subsession->codecName(), "H264") == 0) {
        pay_load = solution::video_box::RTSP_Payload_H264;
      } else if (strcmp(scs.subsession->codecName(), "H265") == 0) {
        pay_load = solution::video_box::RTSP_Payload_H265;
      } else {
        LOGE << "rtsp recv sdp video, unknow codee name:"
             << scs.subsession->codecName();
        shutdownStream(rtspClient);
      }
    } else if (strcmp(scs.subsession->mediumName(), "audio") == 0) {
      LOGE << "channel: " << our_rtsp_client->GetChannel()
           << "recv audio media";
      break;
      if (strcmp(scs.subsession->codecName(), "PCMA") == 0) {
        pay_load = solution::video_box::RTSP_Payload_PCMA;
      } else if (strcmp(scs.subsession->codecName(), "PCMU") == 0) {
        pay_load = solution::video_box::RTSP_Payload_PCMU;
      } else {
        LOGE << "rtsp recv sdp audio, unknow codee name:"
             << scs.subsession->codecName();
        shutdownStream(rtspClient);
      }
    } else {
      LOGE << "rtsp recv sdp info, unknow codee name:"
           << scs.subsession->codecName();
      shutdownStream(rtspClient);
    }

    LOGI << "channel:" << our_rtsp_client->GetChannel()
         << " , rtsp recv decode strem:" << scs.subsession->codecName()
         << ", resolution:" << scs.subsession->videoWidth() << ", "
         << scs.subsession->videoHeight();

    int buffer_size = our_rtsp_client->GetFrameMaxSize();
    int buffer_count = 8;
    if (buffer_size > 500) {
      buffer_count = 6;
    }
    if (buffer_size > 1024) {
      LOGD << "rtsp config channel:" << our_rtsp_client->GetChannel()
           << " max frame size:" << buffer_size;
      buffer_size = 1024;
    }
    buffer_size = buffer_size * 1024;

    auto media = solution::video_box::MediaPipeManager::GetInstance()
                     .GetPipeline()[our_rtsp_client->GetChannel()];

    if (pay_load == solution::video_box::RTSP_Payload_H264) {
      H264Sink *dummy_sink_ptr = H264Sink::createNew(
          env, *scs.subsession, rtspClient->url(), buffer_size, buffer_count);
      LOGI << "rtsp client new H264Sink, channel:"
           << our_rtsp_client->GetChannel();
      dummy_sink_ptr->SetFileName(our_rtsp_client->GetOutputFileName());
      dummy_sink_ptr->SetChannel(our_rtsp_client->GetChannel());
      dummy_sink_ptr->AddPipeline(media);
      scs.subsession->sink = dummy_sink_ptr;
    } else if (pay_load == solution::video_box::RTSP_Payload_H265) {
      H265Sink *dummy_sink_ptr = H265Sink::createNew(
          env, *scs.subsession, rtspClient->url(), buffer_size, buffer_count);
      LOGI << "rtsp client new H265Sink, channel:"
           << our_rtsp_client->GetChannel();
      dummy_sink_ptr->SetFileName(our_rtsp_client->GetOutputFileName());
      dummy_sink_ptr->SetChannel(our_rtsp_client->GetChannel());
      dummy_sink_ptr->AddPipeline(media);
      scs.subsession->sink = dummy_sink_ptr;
    } else if (pay_load == solution::video_box::RTSP_Payload_PCMU ||
               pay_load == solution::video_box::RTSP_Payload_PCMA) {
      AudioG711Sink *dummy_sink_ptr =
          AudioG711Sink::createNew(env, *scs.subsession, rtspClient->url());
      LOGI << "rtsp client new G711Sink, channel:"
           << our_rtsp_client->GetChannel();
      dummy_sink_ptr->SetFileName(our_rtsp_client->GetOutputFileName());
      dummy_sink_ptr->SetChannel(our_rtsp_client->GetChannel());
      dummy_sink_ptr->AddPipeline(media);
      scs.subsession->sink = dummy_sink_ptr;
    }

    // perhaps use your own custom "MediaSink" subclass instead
    if (scs.subsession->sink == NULL) {
      env << *rtspClient << "Failed to create a data sink for the \""
          << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
      break;
    }

    media->SetDecodeType(pay_load);
    env << *rtspClient << "Created a data sink for the \"" << *scs.subsession
        << "\" subsession\n";
    scs.subsession->miscPtr =
        rtspClient;  // a hack to let subsession handler functions get the
                     // "RTSPClient" from the subsession
    scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
                                       subsessionAfterPlaying, scs.subsession);
    // Also set a handler to be called if a RTCP "BYE" arrives for this
    // subsession:
    if (scs.subsession->rtcpInstance() != NULL) {
      scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler,
                                                    scs.subsession);
    }
  } while (0);
  delete[] resultString;

  // Set up the next subsession, if any:
  setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient *rtspClient, int resultCode,
                       char *resultString) {
  Boolean success = False;

  do {
    UsageEnvironment &env = rtspClient->envir();  // alias
    StreamClientState &scs =
        (reinterpret_cast<ourRTSPClient *>(rtspClient))->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to start playing session: " << resultString
          << "\n";
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration
    // (if the stream does not already signal its end using a RTCP "BYE").  This
    // is optional.  If, instead, you want to keep the stream active - e.g., so
    // you can later 'seek' back within it and do another RTSP "PLAY" - then you
    // can omit this code. (Alternatively, if you don't want to receive the
    // entire stream, you could set this timer for some shorter value.)
    if (scs.duration > 0) {
      unsigned const delaySlop =
          2;  // number of seconds extra to delay, after the stream's expected
              // duration.  (This is optional.)
      scs.duration += delaySlop;
      unsigned uSecsToDelay = (unsigned)(scs.duration * 1000000);
      scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(
          uSecsToDelay, (TaskFunc *)streamTimerHandler, rtspClient);
    }

    env << *rtspClient << "Started playing session";
    if (scs.duration > 0) {
      env << " (for up to " << scs.duration << " seconds)";
    }
    env << "...\n";

    //{{
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession *subsession;
    while ((subsession = iter.next()) != NULL) {
      if (subsession->rtpSource() != NULL) {
        unsigned const thresh = 1000000;  // 1 second
        subsession->rtpSource()->setPacketReorderingThresholdTime(thresh);
        int socketNum = subsession->rtpSource()->RTPgs()->socketNum();
        int curBufferSize = getReceiveBufferSize_(env, socketNum);
        int newBufferSize = 200 * 1024;
        if (curBufferSize < newBufferSize) {
          newBufferSize = setReceiveBufferTo_(env, socketNum, newBufferSize);
        }
      }
    }
    //}}

    success = True;
  } while (0);
  delete[] resultString;

  if (!success) {
    // An unrecoverable error occurred with this stream.
    shutdownStream(rtspClient);
  } else {
    LOGW << "channel:"
         << (reinterpret_cast<ourRTSPClient *>(rtspClient))->GetChannel()
         << " start playing success!";
  }
}

// Implementation of the other event handlers:

void subsessionAfterPlaying(void *clientData) {
  MediaSubsession *subsession = (MediaSubsession *)clientData;
  RTSPClient *rtspClient = (RTSPClient *)(subsession->miscPtr);

  // Begin by closing this subsession's stream:
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether *all* subsessions' streams have now been closed:
  MediaSession &session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL) {
    if (subsession->sink != NULL) return;  // this subsession is still active
  }

  // All subsessions' streams have now been closed, so shutdown the client:
  shutdownStream(rtspClient);
}

void subsessionByeHandler(void *clientData) {
  MediaSubsession *subsession = (MediaSubsession *)clientData;
  RTSPClient *rtspClient = (RTSPClient *)subsession->miscPtr;
  UsageEnvironment &env = rtspClient->envir();  // alias

  env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession
      << "\" subsession\n";

  // Now act as if the subsession had closed:
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void *clientData) {
  ourRTSPClient *rtspClient = (ourRTSPClient *)clientData;
  StreamClientState &scs = rtspClient->scs;  // alias

  scs.streamTimerTask = NULL;

  // Shut down the stream:
  shutdownStream(rtspClient);
}

void shutdownStream(RTSPClient *rtspClient, int exitCode) {
  UsageEnvironment &env = rtspClient->envir();  // alias
  StreamClientState &scs =
      (reinterpret_cast<ourRTSPClient *>(rtspClient))->scs;  // alias
  int channel = (reinterpret_cast<ourRTSPClient *>(rtspClient))->channel_;
  // First, check whether any subsessions have still to be closed:
  if (scs.session != NULL) {
    printf("scs.session\n");
    Boolean someSubsessionsWereActive = False;
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession *subsession;

    while ((subsession = iter.next()) != NULL) {
      LOGI << "channel:" << channel
           << " subsession name:" << subsession->mediumName();
      if (subsession->sink != NULL) {
        LOGI << "channel:" << channel << " subsession->sink try close";
        Medium::close(subsession->sink);
        subsession->sink = NULL;
        LOGI << "channel:" << channel << " subsession->sink";
        if (subsession->rtcpInstance() != NULL) {
          subsession->rtcpInstance()->setByeHandler(
              NULL, NULL);  // in case the server sends a RTCP "BYE" while
                            // handling "TEARDOWN"
        }
        someSubsessionsWereActive = True;
      }
    }
    LOGI << "channel:" << channel << " subsession->sink22";
    if (someSubsessionsWereActive) {
      // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the
      // stream. Don't bother handling the response to the "TEARDOWN".
      rtspClient->sendTeardownCommand(*scs.session, NULL);
    }
  }

  env << *rtspClient << "Closing the stream.\n";
  Medium::close(rtspClient);
  // Note that this will also cause this stream's "StreamClientState" structure
  // to get reclaimed.

  if (--rtspClientCount == 0) {
    // The final stream has ended, so exit the application now.
    // (Of course, if you're embedding this code into your own application, you
    // might want to comment this out, and replace it with
    // "eventLoopWatchVariable = 1;", so that we leave the LIVE555 event loop,
    // and continue running "main()".) exit(exitCode);
  }
}
