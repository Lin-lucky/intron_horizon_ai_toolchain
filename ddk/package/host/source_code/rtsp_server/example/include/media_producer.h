#ifndef MEDIA_PRODUCER_H_
#define MEDIA_PRODUCER_H_

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "rtsp_server.h"
#include "rtsp_server_config.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "libswresample/swresample.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/avutil.h"
#include "libavutil/imgutils.h"
#include "libavutil/hwcontext.h"
#ifdef __cplusplus
}
#endif
#include "hobotlog/hobotlog.hpp"

namespace rtspcomponent {
class MediaProducer {
 public:
  explicit MediaProducer(std::string rtsp_config);
  ~MediaProducer();
  int Init();
  int DeInit();
  int Start();
  int Stop();

 private:
  int GetMediaThread(int chn);

 private:
  std::shared_ptr<RtspServer> rtsp_server_;
  std::shared_ptr<RtspServerConfig> config_;
  std::vector<AVFormatContext *> input_ctx_;

  std::vector<bool> thread_exit_flag_;
  std::vector<std::string> media_source_list_;
  std::vector<std::shared_ptr<std::thread>> get_frame_thread_;
  int video_type_;
};
}  // namespace RTSPComponent
#endif
