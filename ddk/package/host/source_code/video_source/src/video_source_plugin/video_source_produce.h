/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef INCLUDE_VIOPRODUCE_VIOPRODUCE_H_
#define INCLUDE_VIOPRODUCE_VIOPRODUCE_H_
#include <atomic>
#include <cstdint>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <memory>
#include <vector>
#include <unordered_map>

#include "video_source_plugin/video_source_message.h"
#include "video_source/video_source.h"
#include "video_source/video_source_type.h"

namespace xproto {

using videosource::VideoSource;
using videosource::VideoSourceLogLevel;
using videosource::VideoSourceType;
using videosource::VideoSourceErrorCode;
using videosource::ImageFrame;
using videosource::PyramidFrame;

enum HorizonVideoSourceType {
  kHorizonVideoSourceTypeNone = 0,
  kHorizonVideoSourceTypeCam,
  kHorizonVideoSourceTypeFb
};

struct ProduceConfig {
  ProduceConfig() {}
  ~ProduceConfig() {}
  std::string board_name;
  std::string produce_name;
  std::string data_type;
  bool is_msg_package;
  bool is_msg_order;
  int max_vio_buffer = 2;
  int max_ts_cmp = 0;  // uint is ms
  int channel_id;
  std::string cfg_file;
  std::string log_level;
};

class VideoSourceProduce : public std::enable_shared_from_this<
                           VideoSourceProduce> {
 public:
  static std::shared_ptr<VideoSourceProduce> CreateVideoSourceProduce(
      const std::string &produce_name,
      const int &channel_id,
      const std::string &cfg_file);

  virtual ~VideoSourceProduce() {}

  int Init();
  int DeInit();
  int Start();
  int Stop();
  virtual int Run() = 0;
  using Listener = std::function<int(const std::shared_ptr<VioMessage> &input)>;
  // set callback function
  int SetListener(const Listener &callback);
  int SetConfig(const ProduceConfig &cfg);
  void SetConfigNum(int num) { pipe_num_ = num; }
  HorizonVideoSourceType GetSourceType() { return source_type_; }

 protected:
  virtual int Finish();
  virtual void FreeBuffer();
  virtual bool AllocBuffer();
  virtual void WaitUntilAllDone();
  virtual int PushPyramidFrameMsg();
  virtual int PushSourceFrameMsg();

 protected:
  std::shared_ptr<ProduceConfig> produce_cfg_;
  std::vector<ProduceConfig> produce_cfg_list_;
  std::shared_ptr<VideoSource> video_source_;
  HorizonVideoSourceType source_type_ = kHorizonVideoSourceTypeNone;
  std::function<int(const std::shared_ptr<VioMessage> &input)> push_data_cb_ =
      nullptr;
  std::atomic_bool is_inited_{false};
  std::atomic_bool is_running_{false};
  int channel_id_ = 0;
  int consumed_vio_buffers_ = 0;
  int pipe_num_ = 2;
  int max_vio_buffer_ = 0;
  std::mutex vio_buffer_mutex_;
  std::future<bool> task_future_;
  uint32_t sample_freq_ = 1;
  std::shared_ptr<std::thread> produce_pym_thread_ = nullptr;
  uint64_t last_frame_id_ = -1;
};

class MipiCamera : public VideoSourceProduce {
 public:
  explicit MipiCamera(const int &channel_id, const std::string &cfg_file);
  virtual ~MipiCamera();
  int Run() override;
};

class UsbCamera : public VideoSourceProduce {
 public:
  explicit UsbCamera(const int &channel_id, const std::string &cfg_file);
  virtual ~UsbCamera();
  int Run() override;
};

class Feedback : public VideoSourceProduce {
 public:
  explicit Feedback(const int &channel_id, const std::string &cfg_file);
  virtual ~Feedback();
  int Run() override;
};

class RtspClient : public VideoSourceProduce {
 public:
  explicit RtspClient(const int &channel_id, const std::string &cfg_file);
  virtual ~RtspClient();
  int Run() override;
};

class PanelCamera : public MipiCamera {
 public:
  explicit PanelCamera(const int &channel_id, const std::string &cfg_file)
    : MipiCamera(channel_id, cfg_file) {}
  virtual ~PanelCamera() {}
};

}  // namespace xproto
#endif  // XPROTO_INCLUDE_XPROTO_PLUGIN_XPLUGIN_H_
