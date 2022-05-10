/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_PLUGIN_H_
#define VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_PLUGIN_H_
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <map>
#include "xproto/msg_type/vio_message.h"
#include "xproto/plugin/xpluginasync.h"

namespace videosource {
template <class T> class BlockingQueue;
}
namespace xproto {

using videosource::BlockingQueue;
using xproto::message::VioMessage;
class VideoSourceConfig;
class ProduceConfig;
class VideoSourceProduce;
using Listener = std::function<int(const std::shared_ptr<VioMessage> &input)>;

class VideoSourcePlugin : public xproto::XPluginAsync {
 public:
  VideoSourcePlugin() = delete;
  explicit VideoSourcePlugin(const std::string &path);
  ~VideoSourcePlugin() override;
  int Init() override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  int CamStart();
  int FbStart();
  int CamStop();
  int FbStop();
  std::string desc() const { return "VideoSourcePlugin"; }
  void SetLoggingLevel(const std::string &log_level);

 private:
  std::shared_ptr<VideoSourceConfig> GetConfigFromFile(const std::string &path);
  void GetSubConfigs();
  void GetJ3devConfigs(ProduceConfig &produce_cfg,
      const std::shared_ptr<VideoSourceConfig> &config_json);
  Listener SetListenerFunc();
  void ClearAllQueue();
  void SyncPymImage();
  void SyncPymImages();
  struct comp {
    bool operator() (
        const std::pair<uint64_t, std::shared_ptr<VioMessage>> &a,
        const std::pair<uint64_t, std::shared_ptr<VioMessage>> &b) {
      return a.first > b.first; }  // descend order
  } cmp_time_stamp;

 private:
  std::shared_ptr<VideoSourceConfig> config_;
  std::vector<ProduceConfig> produce_cfg_list_;
  std::vector<std::shared_ptr<VideoSourceProduce>> produce_handles_;
  std::vector<std::string> cfg_files_;
  std::string log_level_;
  bool is_inited_ = false;
  /**
   * message sync mode as follows:
   * 1) async mode: is_sync_mode = false,
   *   a) is_order_mode = true
   *      [0, 1, 2, 3, 4...], msg is async(VioMessage)
   *      message is send in a fixed order
   *   b) is_order_mode = false
   *      [0, 1, 4, 2, 3...], msg is async(VioMessage),
   *      message is send maybe not in a fixed order
   * 2) sync_mode: msg_sync_mode = 1
   *    [0, 1, 2, 3, 4...],  msg is sync(MultiVioMessage)
   *    message is send in a fixed order
   */
  bool is_sync_mode_ = false;
  bool is_order_mode_ = true;
  std::atomic_bool is_running_{false};
  int data_source_num_ = 1;
  uint64_t max_ts_cmp_ = 0;
  std::vector<std::shared_ptr<BlockingQueue<std::shared_ptr<VioMessage>>>>
    img_msg_queue_list_;
  std::map<int, int> channel_status_;
  std::mutex pym_img_mutex_;
  std::map<int, int> chn_id_2_index_;
};

}  // namespace xproto
#endif  // VIDEO_SOURCE_PLUGIN_VIDEO_SOURCE_PLUGIN_H_
