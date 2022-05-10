/*
 * @Description: implement of visualplugin
 * @Author: GYW
 * @Date: 2020-03-30
 * @Copyright 2020 Horizon Robotics, Inc.
 */
#ifndef INCLUDE_RTSP_SERVER_PLUGIN_H_
#define INCLUDE_RTSP_SERVER_PLUGIN_H_

#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <thread>
#include <queue>

#include "rtsp_plugin_config.h"
#include "xproto/message/flowmsg.h"
#include "xproto/plugin/xpluginasync.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
#include "smart_message/smart_message.h"
#include "xproto/msg_type/vio_message.h"
#include "thread_pool/thread_pool.h"
#include "rtsp_server/rtsp_server.h"
#include "media_codec/media_codec_manager.h"

namespace xproto {
using horizon::vision::CThreadPool;
using xproto::XProtoMessagePtr;
using xproto::message::SmartMessagePtr;
using rtspcomponent::RtspServer;
using xproto::message::VioMessage;
using xproto::message::SmartMessage;

struct rtsp_compare_frame {
  bool operator()(const x3::FrameMessage &f1, const x3::FrameMessage &f2) {
    return (f1.timestamp_() > f2.timestamp_());
  }
};

struct rtsp_compare_msg {
  bool operator()(const SmartMessagePtr m1, const SmartMessagePtr m2) {
    return (m1->time_stamp_ > m2->time_stamp_);
  }
};

#define min(X, Y) ((X) < (Y) ? (X) : (Y))

#define UUID_SIZE 16

static unsigned char uuid[] = { 0x54, 0x80, 0x83, 0x97, 0xf0,
     0x23, 0x47, 0x4b, 0xb7, 0xf7, 0x4f, 0x32, 0xb5, 0x4e, 0x06, 0xac };

static unsigned char start_code[] = {0x00, 0x00, 0x00, 0x01};

class RTSPServerPlugin : public xproto::XPluginAsync {
 public:
  RTSPServerPlugin() = delete;
  explicit RTSPServerPlugin(const std::string &config_file);
  ~RTSPServerPlugin() override;
  int Init() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "rtsp_server_plugin"; }

 private:
  enum InputType {
    VT_VIDEO,  // VioMessage
    VT_SMART   // SmartMessage
  };

  int Reset();
  void PushFrame(InputType type, XProtoMessagePtr frame_msg);
  // encode thread
  void EncodeStream(XProtoMessagePtr msg);

  int FeedVideo(XProtoMessagePtr msg);
  int FeedSmart(XProtoMessagePtr msg);
  int FeedCommand(XProtoMessagePtr msg);          // 处理总线command
  int SendSmartMessage(SmartMessagePtr msg, x3::FrameMessage &fm);
  void MapSmartProc();

  virtual std::string GetSmartMessageType() {
    // 当前解决方案默认使用TYPE_SMART_MESSAGE
    return TYPE_SMART_MESSAGE;
  }

 private:
  uint32_t ReverseBytes(uint32_t value);
  uint32_t GetSeiNaluSize(uint32_t content);
  uint32_t GetSeiPacketSize(uint32_t size);
  int FillSeiPacket(unsigned char *packet, bool isAnnexb,
            const char *content, uint32_t size);

 private:
  static int GetYUV(VideoEncodeSourceBuffer *frame_buf, VioMessage *vio_msg,
          int level, int use_vb);
  static int PackSmartMsg(std::string &data, SmartMessage *smart_msg,
                            int ori_w, int ori_h, int dst_w, int dst_h);

 private:
  std::string config_file_;
  std::shared_ptr<RTSPPluginConfig> config_;

  const uint8_t cache_size_ = 25;  // max input cache size
  bool stop_flag_;
  std::mutex map_mutex_;
  std::mutex map_smart_mutex_;
  bool map_stop_ = false;
  std::shared_ptr<std::thread> worker_ = nullptr;
  std::priority_queue<x3::FrameMessage, std::vector<x3::FrameMessage>,
                      rtsp_compare_frame>
    x3_frames_;
  std::priority_queue<SmartMessagePtr, std::vector<SmartMessagePtr>,
                      rtsp_compare_msg>
    x3_smart_msg_;
  int chn_;
  CThreadPool video_encode_thread_;
  CThreadPool data_send_thread_;
  bool smart_stop_flag_;
  bool video_stop_flag_;
  std::condition_variable map_smart_condition_;
  std::mutex video_mutex_;
  std::mutex smart_mutex_;

  int origin_image_width_ = 1920;  // update by FeedVideo
  int origin_image_height_ = 1080;
  int dst_image_width_ = 1920;  // update by FeedVideo
  int dst_image_height_ = 1080;
  std::shared_ptr<RtspServer> rtsp_server_;
  int sei_len_ = 0;
  unsigned char sei_buf_[30000];
  bool is_annexb_ = true;
};

}  // namespace xproto
#endif
