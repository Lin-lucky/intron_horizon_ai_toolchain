/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#ifndef INCLUDE_RTSPPLUGIN_SMARTPLUGIN_H_
#define INCLUDE_RTSPPLUGIN_SMARTPLUGIN_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"

#include "xstream/xstream_sdk.h"
#include "media_pipe_manager/media_pipeline.h"
#include "rtsp_client/rtsp_client.h"
#include "transport_message/rtsp_message.h"
#include "json/json.h"
#include "video_processor.h"

#ifdef __cplusplus
extern "C" {
#include "hb_comm_vdec.h"
#include "hb_comm_venc.h"
#include "hb_comm_video.h"
#include "hb_common.h"
#include "hb_sys.h"
#include "hb_type.h"
#include "hb_vdec.h"
#include "hb_venc.h"
#include "hb_vio_interface.h"
#include "hb_vp_api.h"
#include "hb_vps_api.h"
}
#endif

namespace solution {
namespace video_box {

using xproto::XPluginAsync;
using xproto::XProtoMessage;
using xproto::XProtoMessagePtr;

using xproto::message::RtspMessage;
using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

class RtspPlugin : public XPluginAsync {
 public:
  RtspPlugin() = default;
  explicit RtspPlugin(const std::string &config_file) {
    config_file_ = config_file;
  }

  void SetConfig(const std::string &config_file) { config_file_ = config_file; }

  ~RtspPlugin() = default;
  int Init() override;
  int Start() override;
  int Stop() override;
  void GetDecodeFrame0();
  void GetDeocdeFrame(std::shared_ptr<MediaPipeline> pipeline, int channel);
  void WaitToStart();
  void GetConfigFromFile(const std::string &path);

 private:
  int Feed(XProtoMessagePtr msg);
  void OnCallback(xstream::OutputDataPtr out);
  void ParseConfig();
  int DecodeInit();
  void Process();
  void CheckRtspState();
  void ProcessData(const int channel_id, pym_buffer_t *out_pym_buf);
  void ComputeFpsThread();

  std::vector<std::thread> threads_;
  std::vector<ourRTSPClient *> rtsp_clients_;

  struct Rtspinfo {
    std::string url;
    bool tcp_flag;
    int frame_max_size;
    bool save_stream;
  };
  std::vector<Rtspinfo> rtsp_url_;

  std::vector<bool> rtsp_clients_stat_;
  int image_width_;
  int image_height_;
  int channel_number_;
  PAYLOAD_TYPE_E codec_type_;
  std::string config_file_;
  Json::Value config_;
  std::shared_ptr<std::thread> process_thread_;
  bool running_;
  static int frame_count_;
  static std::mutex framecnt_mtx_;

  std::shared_ptr<std::thread> check_thread_;
  TaskScheduler *scheduler_;
  UsageEnvironment *env_;

  bool drop_frame_ = false;
  int drop_frame_interval_ = 0;

  bool not_run_smart_ = false;
  std::shared_ptr<VideoProcessor> video_processor_;
  int display_mode_ = 0;
};

}  // namespace video_box
}  // namespace solution
#endif  // INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
