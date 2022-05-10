// Copyright (c) 2019, Horizon Robotics, Inc.
#include "horizon_rtsp_server.hh"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>

#include "visual_plugin/horizon_server_api.h"

#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>

static pthread_t gs_RtspPid;
static pthread_mutex_t gs_mutex;

uint32_t g_data_buf_size_ = 0;

TaskScheduler *scheduler;
UsageEnvironment *env;

HorizonRTSPServer *HorizonRTSPServer::createNew(
    UsageEnvironment &env, Port ourPort,
    UserAuthenticationDatabase *authDatabase, unsigned reclamationTestSeconds) {
  int ourSocket = setUpOurSocket(env, ourPort);
  if (ourSocket == -1) {
    return NULL;
  }

  return new HorizonRTSPServer(env, ourSocket, ourPort, authDatabase,
                               reclamationTestSeconds);
}

HorizonRTSPServer::HorizonRTSPServer(UsageEnvironment &env, int ourSocket,
                                     Port ourPort,
                                     UserAuthenticationDatabase *authDatabase,
                                     unsigned reclamationTestSeconds)
    : RTSPServerSupportingHTTPStreaming(env, ourSocket, ourPort, authDatabase,
                                        reclamationTestSeconds) {}

HorizonRTSPServer::~HorizonRTSPServer() {}

static ServerMediaSession *createNewSMS(UsageEnvironment &env,
                                        char const *streamName, char *name,
                                        char *key, char *value);

ServerMediaSession *HorizonRTSPServer ::lookupServerMediaSession(
    char const *streamName, Boolean isFirstLookupInSession) {
  int len = strlen(streamName);
  char name[64] = {0};
  char key[64] = {0};
  char value[64] = {0};

  int namePos = 0;
  int keyPos = -1;
  int valuePos = -1;

  for (int i = 0; i < len; i++) {
    if (streamName[i] == '?') {
      name[namePos] = 0;
      namePos = -1;
      keyPos = 0;
      continue;
    }
    if (namePos >= 0) {
      name[namePos] = streamName[i];
      namePos++;
    }
    if (streamName[i] == '=') {
      key[keyPos] = 0;
      keyPos = -1;
      valuePos = 0;
      continue;
    }
    if (keyPos >= 0) {
      key[keyPos] = streamName[i];
      keyPos++;
    }
    if (valuePos >= 0) {
      value[valuePos] = streamName[i];
      valuePos++;
    }
  }
  value[valuePos] = 0;

  envir() << "streamName:" << streamName << " name:" << name << " key:" << key
          << " value:" << value << "\n";

  // Next, check whether we already have a "ServerMediaSession" for this file:
  ServerMediaSession *sms = RTSPServer::lookupServerMediaSession(streamName);

  if (sms == NULL && (strcmp(name, "horizonMeta") == 0) &&
      (strcmp(key, "MediaType") == 0)) {
    int flag = atoi(value);
    int sendflag = flag & 0xFF;
    //    envir() << "sendflag:" << sendflag << "\n";
    if (sendflag == 0) {
      return NULL;
    }
    sms = createNewSMS(envir(), streamName, name, key, value);
    addServerMediaSession(sms);
  }

  return sms;
}

static ServerMediaSession *createNewSMS(UsageEnvironment &env,
                                        char const *streamName, char *name,
                                        char *key, char *value) {
  ServerMediaSession *sms = ServerMediaSession::createNew(
      env, streamName, streamName, "session by horizonH264", False);
  MetadataServerMediaSubsession *subsession =
      MetadataServerMediaSubsession::createNew(env, False, key, value);
  sms->addSubsession(subsession);

  return sms;
}

MetadataFramedSource *MetadataFramedSource::createNew(
    UsageEnvironment &env, bool sendH264, bool sendImage, bool sendSmart,
    bool sendFeature, bool sendBackground) {
  MetadataFramedSource *source = new MetadataFramedSource(
      env, sendH264, sendImage, sendSmart, sendFeature, sendBackground);
  return source;
}

MetadataFramedSource *MetadataFramedSource::frameSources[100];

void MetadataFramedSource::getH264Frame(unsigned char *buff, int len) {
  int offset = len;
  buffSize = len;
  int totalSize = sizeof(frameBuffer) / sizeof(char) - offset;
  int totalCnt = sizeof(videoOffset) / sizeof(int) - 1;

  //缓存满需要另外处理
  if (videoTail + buffSize > totalSize || videoBuffCnt >= totalCnt) {
    envir() << "------drop frame videoCurrentCnt:" << videoCurrentCnt
            << " videoBuffCnt:" << videoBuffCnt << " videoTail:" << videoTail
            << " videoFront:" << videoFront << " object:" << this << "\n";
    videoFront = 0;
    videoTail = 0;
    videoCurrentCnt = 0;
    videoBuffCnt = 0;
    memset(videoOffset, 0, sizeof(videoOffset));

    return;
  }

  offset = videoTail;
  // perception.SerializeToArray(frameBuffer + offset, buffSize);
  memcpy(frameBuffer + offset, buff, len);
  buffSize = len;

  videoTail += buffSize;
  videoOffset[videoBuffCnt] = videoTail;

  videoBuffCnt++;

  if (horizonEventTriggerId > 0) {
    scheduler->triggerEvent(horizonEventTriggerId, this);
  }
}

void MetadataFramedSource::sendFrame(void *client) {
  if (client != NULL) {
    // 单个连接发送数据
    MetadataFramedSource *source =
        reinterpret_cast<MetadataFramedSource *>(client);
    source->doGetNextFrame();
  } else {
    // 所有连接发送数据
    int count = sizeof(frameSources) / sizeof(MetadataFramedSource *);
    for (int i = 0; i < count; i++) {
      MetadataFramedSource *source = frameSources[i];
      if (source != NULL) {
        source->doGetNextFrame();
      }
    }
  }
}

void MetadataFramedSource::onDataFrame(unsigned char *buff, int len,
                                       int msgType) {
  int count = sizeof(frameSources) / sizeof(MetadataFramedSource *);
  for (int i = 0; i < count; i++) {
    pthread_mutex_lock(&gs_mutex);

    MetadataFramedSource *source = frameSources[i];
    if (source != NULL) {
      if (msgType == MsgSmart && source->needSmart()) {
        source->sendSmartCnt++;
        //        if (source->sendSmartCnt <= 1000) {
        source->getDataFrame(buff, len);
        //        }
      }
      if (msgType == MsgFeature && source->needFeature()) {
        source->getDataFrame(buff, len);
      }
      if (msgType == MsgImage && source->needImage()) {
        source->sendImageCnt++;
        //        if (source->sendImageCnt <= 150) {
        source->getDataFrame(buff, len);
        //        }
      }
      if (msgType == MsgBackground && source->needBackground()) {
        source->sendBackgroudCnt++;
        //        if (source->sendBackgroudCnt <= 150) {
        source->getDataFrame(buff, len);
        //        }
      }
    } else {
      //    printf("source is null\n");
    }

    pthread_mutex_unlock(&gs_mutex);
  }
}

void MetadataFramedSource::getDataFrame(unsigned char *buff, int len) {
  int totalSize = sizeof(dataFrameBuffer) / sizeof(char);
  // int totalSize = g_data_buf_size_;
  int totalCnt = sizeof(dataOffset) / sizeof(int) - 1;

  //缓存满需要另外处理
  if (dataTail + len > totalSize || dataBuffCnt >= totalCnt) {
    envir() << "------drop frame currentCnt:" << currentCnt
            << " dataBuffCnt:" << dataBuffCnt << " dataTail:" << dataTail
            << " dataFront:" << dataFront << " object:" << this << "\n";
    dataFront = 0;
    dataTail = 0;
    currentCnt = 0;
    dataBuffCnt = 0;
    memset(dataOffset, 0, sizeof(dataOffset));

    return;
  }

  int offset = dataTail;
  memcpy(dataFrameBuffer + offset, buff, len);
  dataTail += len;
  dataOffset[dataBuffCnt] = dataTail;

  dataBuffCnt++;

  if (dataEventTriggerId > 0) {
    scheduler->triggerEvent(dataEventTriggerId, this);
  }
}

void MetadataFramedSource::onH264Frame(unsigned char *buff, int len) {
  int count = sizeof(frameSources) / sizeof(MetadataFramedSource *);

  for (int i = 0; i < count; i++) {
    pthread_mutex_lock(&gs_mutex);

    MetadataFramedSource *source = frameSources[i];
    if (source != NULL) {
      //   if (source->needMedia()) {
      if (source->needImage()) {
        source->getH264Frame(buff, len);
      }
    }

    pthread_mutex_unlock(&gs_mutex);
  }
}

unsigned MetadataFramedSource::referenceCount = 0;

MetadataFramedSource::MetadataFramedSource(UsageEnvironment &env, bool sendH264,
                                           bool sendImage, bool sendSmart,
                                           bool sendFeature,
                                           bool sendBackground)
    : FramedSource(env) {
  envir() << "MetadataFramedSource create . " << this << "\n";
  ++referenceCount;

  dataFrameBuffer = (unsigned char *)malloc(g_data_buf_size_);

  buffSize = 0;
  videoFront = 0;
  videoTail = 0;
  videoCurrentCnt = 0;
  videoBuffCnt = 0;
  memset(videoOffset, 0, sizeof(videoOffset));

  dataFront = 0;
  dataTail = 0;
  currentCnt = 0;
  dataBuffCnt = 0;
  memset(dataOffset, 0, sizeof(dataOffset));

  this->sendH264 = sendH264;
  this->sendImage = sendImage;
  this->sendSmart = sendSmart;
  this->sendFeature = sendFeature;
  this->sendBackground = sendBackground;

  sendH264Cnt = 0;
  sendImageCnt = 0;
  sendSmartCnt = 0;
  sendBackgroudCnt = 0;
  sendFeatureCnt = 0;

  horizonEventTriggerId = 0;
  if (horizonEventTriggerId == 0) {
    horizonEventTriggerId =
        envir().taskScheduler().createEventTrigger(sendFrame);
  }
  dataEventTriggerId = 0;
  if (dataEventTriggerId == 0) {
    dataEventTriggerId = envir().taskScheduler().createEventTrigger(sendFrame);
  }

  addSource(this);
}

MetadataFramedSource::~MetadataFramedSource() {
  envir() << "MetadataFramedSource destory . " << this << "\n";
  eraseSource(this);

  --referenceCount;
  if (referenceCount == 0) {
    // Reclaim our 'event trigger'
  }

  if (horizonEventTriggerId > 0) {
    envir().taskScheduler().deleteEventTrigger(horizonEventTriggerId);
    horizonEventTriggerId = 0;
  }
  if (dataEventTriggerId > 0) {
    envir().taskScheduler().deleteEventTrigger(dataEventTriggerId);
    dataEventTriggerId = 0;
  }

  if (dataFrameBuffer) {
    free(dataFrameBuffer);
    dataFrameBuffer = nullptr;
  }
}

int MetadataFramedSource::addSource(MetadataFramedSource *source) {
  int count = sizeof(frameSources) / sizeof(MetadataFramedSource *);
  for (int i = 0; i < count; i++) {
    if (frameSources[i] == NULL) {
      envir() << "MetadataFramedSource " << source << " addSource i = " << i
              << "\n";
      frameSources[i] = source;
      return i;
    }
  }
  envir() << "MetadataFramedSource::addSource failed\n ";
  return -1;
}

int MetadataFramedSource::eraseSource(MetadataFramedSource *source) {
  pthread_mutex_lock(&gs_mutex);
  int count = sizeof(frameSources) / sizeof(MetadataFramedSource *);
  for (int i = 0; i < count; i++) {
    if (frameSources[i] == source) {
      envir() << "MetadataFramedSource " << source << " eraseSource i = " << i
              << "\n";
      frameSources[i] = NULL;
      pthread_mutex_unlock(&gs_mutex);
      return i;
    }
  }
  envir() << "MetadataFramedSource::eraseSource failed!\n";
  pthread_mutex_unlock(&gs_mutex);
  return -1;
}

void MetadataFramedSource::doGetNextFrame() {
  //上次读的数据还没有处理完
  if (isCurrentlyAwaitingData() == False) {
    //    envir() << "isCurrentlyAwaitingData()" << isCurrentlyAwaitingData() <<
    //    "\n";
    return;
  }

  if (videoTail > videoFront && horizonEventTriggerId > 0) {
    pthread_mutex_lock(&gs_mutex);

    int offset = videoOffset[videoCurrentCnt];
    int size = offset - videoFront;
    memcpy(fTo, frameBuffer + videoFront, size);
    fFrameSize = size;
    videoFront = offset;
    videoCurrentCnt++;

    if (videoFront >= videoTail) {
      videoFront = 0;
      videoTail = 0;
      videoCurrentCnt = 0;
      videoBuffCnt = 0;
      memset(videoOffset, 0, sizeof(videoOffset));
    }

    // gettimeofday(&fPresentationTime, NULL);
    fPresentationTime.tv_usec =
        std::chrono::system_clock::now().time_since_epoch().count();
    afterGetting(this);

    pthread_mutex_unlock(&gs_mutex);
    return;
  }
  if (dataTail > dataFront && dataEventTriggerId > 0) {
    pthread_mutex_lock(&gs_mutex);

    unsigned int offset = dataOffset[currentCnt];
    unsigned int size = offset - dataFront;
    if (size >= fMaxSize) {
      envir() << "drop buffer! buffsize : " << size
              << "larger than fMaxSize : " << fMaxSize << " \n";

      dataFront = offset;
      currentCnt++;
      if (dataFront >= dataTail) {
        dataFront = 0;
        dataTail = 0;
        currentCnt = 0;
        dataBuffCnt = 0;
        memset(dataOffset, 0, sizeof(dataOffset));
      }

      pthread_mutex_unlock(&gs_mutex);
      return;
    }

    memcpy(fTo, dataFrameBuffer + dataFront, size);
    fFrameSize = size;
    dataFront = offset;
    currentCnt++;

    if (dataFront >= dataTail) {
      dataFront = 0;
      dataTail = 0;
      currentCnt = 0;
      dataBuffCnt = 0;
      memset(dataOffset, 0, sizeof(dataOffset));
    }

    // gettimeofday(&fPresentationTime, NULL);
    fPresentationTime.tv_usec =
        std::chrono::system_clock::now().time_since_epoch().count();
    afterGetting(this);

    pthread_mutex_unlock(&gs_mutex);
    return;
  }
}

void MetadataFramedSource::doStopGettingFrames() {
  envir() << "MetadataFramedSource::doStopGettingFrames" << this << "\n";
  eraseSource(this);

  pthread_mutex_lock(&gs_mutex);

  if (horizonEventTriggerId > 0) {
    envir().taskScheduler().deleteEventTrigger(horizonEventTriggerId);
    horizonEventTriggerId = 0;
  }
  if (dataEventTriggerId > 0) {
    envir().taskScheduler().deleteEventTrigger(dataEventTriggerId);
    dataEventTriggerId = 0;
  }

  videoFront = 0;
  videoTail = 0;
  videoCurrentCnt = 0;
  videoBuffCnt = 0;
  memset(videoOffset, 0, sizeof(videoOffset));

  dataFront = 0;
  dataTail = 0;
  currentCnt = 0;
  dataBuffCnt = 0;
  memset(dataOffset, 0, sizeof(dataOffset));

  pthread_mutex_unlock(&gs_mutex);

  FramedSource::doStopGettingFrames();
}

MetadataServerMediaSubsession *MetadataServerMediaSubsession::createNew(
    UsageEnvironment &env, Boolean reuseFirstSource, char *key, char *value) {
  MetadataServerMediaSubsession *subsession =
      new MetadataServerMediaSubsession(env, reuseFirstSource, key, value);
  return subsession;
}

void MetadataServerMediaSubsession::onSendError(void *clientData) {
  static int counter = 0;
  counter++;
  *env << "onSendErrorFunc():counter:" << counter << "\n";
}

MetadataServerMediaSubsession::MetadataServerMediaSubsession(
    UsageEnvironment &env, Boolean reuseFirstSource, char *key, char *value)
    : OnDemandServerMediaSubsession(env, reuseFirstSource) {
  sendH264 = true;
  sendSmart = true;
  sendFeature = true;
  sendImage = true;
  sendBackground = true;

  if (key != NULL) {
    snprintf(m_key, sizeof(m_key), "%s", key);
  }
  if (value != NULL) {
    snprintf(m_value, sizeof(m_value), "%s", value);
    int flag = atoi(value);
    if (flag & 0x01) {
      sendH264 = true;
    } else {
      sendH264 = false;
    }
    if (flag & 0x02) {
      sendSmart = true;
    } else {
      sendSmart = false;
    }
    if (flag & 0x04) {
      sendFeature = true;
    } else {
      sendFeature = false;
    }
    if (flag & 0x08) {
      sendImage = true;
    } else {
      sendImage = false;
    }
    if (flag & 0x10) {
      sendBackground = true;
    } else {
      sendBackground = false;
    }
  }
}

MetadataServerMediaSubsession::~MetadataServerMediaSubsession() {}

FramedSource *MetadataServerMediaSubsession::createNewStreamSource(
    unsigned clientSessionId, unsigned &estBitrate) {
  MetadataFramedSource *source = MetadataFramedSource::createNew(
      envir(), sendH264, sendImage, sendSmart, sendFeature, sendBackground);

  envir() << "key:" << m_key << " value:" << m_value << " sendH264:" << sendH264
          << " sendImage:" << sendImage << " sendSmart:" << sendSmart
          << " sendFeature:" << sendFeature
          << " sendBackground:" << sendBackground << "\n";

  estBitrate = 4000;

  return source;
}

RTPSink *MetadataServerMediaSubsession::createNewRTPSink(
    Groupsock *rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic,
    FramedSource *inputSource) {
  SimpleRTPSink *sink = SimpleRTPSink::createNew(
      envir(), rtpGroupsock, 112, 90000, "video", "HORIZON", 1, False, True);
  sink->setOnSendErrorFunc(onSendError, this);
  return sink;
}

char const *MetadataServerMediaSubsession::sdpLines() {
  if (fSDPLines == NULL) {
    // We need to construct a set of SDP lines that describe this
    // subsession (as a unicast stream).  To do so, we first create
    // dummy (unused) source and "RTPSink" objects,
    // whose parameters we use for the SDP lines:
    unsigned estBitrate;
    FramedSource *inputSource = createNewStreamSource(0, estBitrate);
    if (inputSource == NULL) {
      return NULL;
    }  // file not found

    struct in_addr dummyAddr;
    dummyAddr.s_addr = 0;
    Groupsock *dummyGroupsock = createGroupsock(dummyAddr, 0);
    unsigned char rtpPayloadType = 96 + trackNumber() - 1;  // if dynamic
    RTPSink *dummyRTPSink =
        createNewRTPSink(dummyGroupsock, rtpPayloadType, inputSource);
    if (dummyRTPSink != NULL && dummyRTPSink->estimatedBitrate() > 0) {
      estBitrate = dummyRTPSink->estimatedBitrate();
    }

    setSDPLinesFromRTPSink(dummyRTPSink, inputSource, estBitrate);
    Medium::close(dummyRTPSink);
    delete dummyGroupsock;
    closeStreamSource(inputSource);
  }

  envir() << "fSDP = " << fSDPLines << "\n";

  return fSDPLines;
}

void MetadataServerMediaSubsession::setSDPLinesFromRTPSink(
    RTPSink *rtpSink, FramedSource *inputSource, unsigned estBitrate) {
  if (rtpSink == NULL) {
    return;
  }

  char const *mediaType = rtpSink->sdpMediaType();
  unsigned char rtpPayloadType = rtpSink->rtpPayloadType();
  AddressString ipAddressStr(fServerAddressForSDP);
  char *rtpmapLine = rtpSink->rtpmapLine();
  char const *rangeLine = rangeSDPLine();

  char const *const sdpFmt =
      "m=%s %u RTP/AVP %d\r\n"
      "c=IN IP4 %s\r\n"
      "b=AS:%u\r\n"
      "%s"
      "%s"
      "a=control:%s\r\n";
  unsigned sdpFmtSize = strlen(sdpFmt) + strlen(mediaType) +
                        5 /* max short len */ + 3         /* max char len */
                        + strlen(ipAddressStr.val()) + 20 /* max int len */
                        + strlen(rtpmapLine) + strlen(rangeLine) +
                        strlen(trackId());
  char *sdpLines = new char[sdpFmtSize];
  snprintf(sdpLines, sdpFmtSize, sdpFmt,
           mediaType,           // m= <media>
           fPortNumForSDP,      // m= <port>
           rtpPayloadType,      // m= <fmt list>
           ipAddressStr.val(),  // c= address
           estBitrate,          // b=AS:<bandwidth>
           rtpmapLine,          // a=rtpmap:... (if present)
           rangeLine,           // a=range:... (if present)
           trackId());          // a=control:<track-id>
  delete[] reinterpret_cast<const char *>(rangeLine);
  delete[] rtpmapLine;

  fSDPLines = strDup(sdpLines);
  delete[] sdpLines;
}

void *RtspServerRun(void *pVoid) {
  SERVER_PARAM_S *param_s = reinterpret_cast<SERVER_PARAM_S *>(pVoid);
  // Begin by setting up our usage environment:
  scheduler = BasicTaskScheduler::createNew();
  env = BasicUsageEnvironment::createNew(*scheduler);

  UserAuthenticationDatabase *authDB = NULL;
  if (param_s->hasConnAuth) {
    // To implement client access control to the RTSP server, do the following:
    authDB = new UserAuthenticationDatabase;
    authDB->addUserRecord(param_s->username, param_s->password);
  }
  *env << "RTSP server use auth: " << param_s->hasConnAuth
       << "  username:" << param_s->username << " password" << param_s->password
       << "\n";

  g_data_buf_size_ = param_s->data_buf_size;
  char *input_filename = param_s->input_filename;
  // OutPacketBuffer::maxSize = 2000000; // bytes
  // OutPacketBuffer::maxSize = 1920 * 1080 * 1.5; // 分包应该大于1帧
  //  OutPacketBuffer::maxSize = param_s->packet_size;
  OutPacketBuffer::maxSize = 2000000;
  // Create the RTSP server.  Try first with the default port number (555),
  // and then with the alternative port number (8555):
  RTSPServer *rtspServer;
  portNumBits rtspServerPortNum = 555;
  //  rtspServer = RTSPServer::createNew(*env, rtspServerPortNum, authDB);
  rtspServer = HorizonRTSPServer::createNew(*env, rtspServerPortNum, authDB);
  if (rtspServer == NULL) {
    *env << "Failed to create RTSP server: " << env->getResultMsg() << "\n";
    exit(1);
  }
  ServerMediaSession *sms = ServerMediaSession::createNew(
      *env, "horizonStream", "h264_fifo", "session by x3H264");
  // sms->addSubsession(OnDemandServerMediaSubsession::createNew(*env, False));
  sms->addSubsession(H264VideoFileServerMediaSubsession ::createNew(
      *env, "/tmp/h264_fifo", True));
  rtspServer->addServerMediaSession(sms);

  ServerMediaSession *sms1 = ServerMediaSession::createNew(
      *env, "horizonStream1", "h264_fifo", "session by x3H264");
  sms1->addSubsession(H264VideoFileServerMediaSubsession ::createNew(
      *env, "/tmp/h264_fifo1", True));
  rtspServer->addServerMediaSession(sms1);

  ServerMediaSession *local_sms = ServerMediaSession::createNew(
      *env, input_filename, "virtual_ipc", "session by x3H264");
  local_sms->addSubsession(H264VideoFileServerMediaSubsession ::createNew(
      *env, input_filename, True));
  rtspServer->addServerMediaSession(local_sms);

  ServerMediaSession *metaSession = ServerMediaSession::createNew(
      *env, "horizonMeta", "horizon metadata stream", "session by horizon",
      False);
  metaSession->addSubsession(
      MetadataServerMediaSubsession::createNew(*env, False, NULL, NULL));
  rtspServer->addServerMediaSession(metaSession);

  *env << "Horizon Media Server\n";
  *env << "\tversion " << HORIZON_SERVER_VERSION_STRING
       << " (Horizon Streaming Media library version "
       << HORIZON_LIBRARY_VERSION_STRING << ")."
#if WIN32
       << " pthread:" << pthread_self().x << "\n";
#else
       << " pthread:" << (uint32_t)pthread_self() << "\n";
#endif
  char *url = rtspServer->rtspURL(sms);
  char *metaUrl = rtspServer->rtspURL(metaSession);
  char *localUrl = rtspServer->rtspURL(local_sms);
  *env << "Play this stream using the URL \n\t" << url << "\n\t" << metaUrl
       << " \n\t" << localUrl << "\n";
  delete[] url;
  delete[] metaUrl;
  delete[] localUrl;

  env->taskScheduler().doEventLoop();  // does not return
  return nullptr;
}

void RtspStop() {
  pthread_cancel(gs_RtspPid);
  pthread_join(gs_RtspPid, 0);
}

int server_send(unsigned char *buffer, int len, int msgType) {
  if (msgType == MsgH264) {
    MetadataFramedSource::onH264Frame(buffer, len);
  } else {
    MetadataFramedSource::onDataFrame(buffer, len, msgType);
  }
  return 0;
}

int server_run(SERVER_PARAM_S *param_s) {
  pthread_mutex_init(&gs_mutex, NULL);
  // RTSP SERVER需要跑在独立线程上，保证与主逻辑线程相互独立
  pthread_create(&gs_RtspPid, 0, RtspServerRun, param_s);

  return 0;
}

int server_stop() {
  RtspStop();
  return 0;
}
