#ifndef RTSP_SERVER_H_
#define RTSP_SERVER_H_

#include <memory>
#include <thread>
#include <fstream>
#include <string>
#include <time.h>

class RTSPServer;
class TaskScheduler;
class UsageEnvironment;
class ServerMediaSession;

namespace rtspcomponent {
class RtspServerConfig;
class RtspServer {
 public:
  explicit RtspServer(std::string config_file);
  ~RtspServer();
  /**
   * @brief load rtsp config
   * @return 0: ok, -1: fail
   */
  int Init();

  /**
   * @brief create RTSPServerRun thread
   * @return 0: ok, -1: fail
   */
  int Start();

  /**
   * @brief stop thread
   * @return 0: ok, -1: fail
   */  
  int Stop();

   /**
   * @brief distory resource
   * @return 0: ok, -1: fail
   */ 
  int DeInit();

  /**
   * @brief send data to rtsp frame source
   * @param in buf: frame
   * @param in buf_len: frame size
   * @param in media_type: H264/H265
   * @param in chn_id: channel id
   * @return 0: ok, -1: fail
   */
  int SendData(unsigned char* buf, int buf_len,
        int media_type, int chn_id);

 private:
   /**
   * @brief create rtsp server and add mediasession
   * @return 0: ok, -1: fail
   */ 
  int RTSPServerRun();   // server 运行在独立线程

 private:
  RtspServerConfig* config_;
  RTSPServer *rtsp_server_;   // live555 rtsp server
  TaskScheduler *scheduler_;
  UsageEnvironment *env_;
  ServerMediaSession *server_media_session_;
  std::shared_ptr<std::thread> rtsp_server_thread_;
  volatile char watch_variable_;
};
}  // namespace RTSPComponent
#endif
