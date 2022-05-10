/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: Songshan Gong
 * @Mail: songshan.gong@horizon.ai
 * @Date: 2019-08-04 02:41:22
 * @Version: v0.0.1
 * @Brief: smartplugin declaration
 * @Last Modified by: Songshan Gong
 * @Last Modified time: 2019-09-30 00:45:01
 */

#ifndef INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
#define INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "smart_plugin/message/custom_smart_message.h"
#include "smart_plugin/runtime_monitor.h"
#include "thread_pool/thread_pool.h"
#include "xproto/msg_type/smart_legible_message.h"
#include "xproto/plugin/xpluginasync.h"
#include "xstream/xstream_world.h"

struct VideoEncodeSourceBuffer;
namespace xproto {
using horizon::vision::CThreadPool;
using xproto::message::CustomSmartMessage;
using xproto::message::SmartLegibleMessage;
using xproto::message::SmartLegibleMessagePtr;
using xproto::message::SmartMessage;
using xstream::InputDataPtr;
using xstream::OutputDataPtr;
using xstream::XStreamSDK;

using RawDataImageFramePtr = std::shared_ptr<xstream::RawDataImageFrame>;
class JsonConfigWrapper;

struct CodecParam {
  CodecParam() {
    is_valid_ = false;
    use_vb_ = 0;
    is_cbr_ = 0;
    bitrate_ = 0;
    frame_buf_depth_ = 0;
    jpg_encode_time_ = 0;
    jpeg_quality_ = 0;
  }
  bool is_valid_;
  int use_vb_;
  int is_cbr_;
  int bitrate_;
  int frame_buf_depth_;
  int jpg_encode_time_;
  uint8_t jpeg_quality_;
};

struct compare_image {
  bool operator()(const RawDataImageFramePtr f1,
                  const RawDataImageFramePtr f2) {
    return (f1->time_stamp_ > f2->time_stamp_);
  }
};
struct compare_smart_msg {
  bool operator()(const SmartLegibleMessagePtr m1,
                  const SmartLegibleMessagePtr m2) {
    return (m1->time_stamp_ > m2->time_stamp_);
  }
};

class SmartPlugin : public XPluginAsync {
 public:
  SmartPlugin() = default;
  explicit SmartPlugin(const std::string& config_file);

  void SetConfig(const std::string& config_file) { config_file_ = config_file; }

  ~SmartPlugin() = default;
  int Init() override;
  int DeInit() override;
  int Start() override;
  int Stop() override;
  std::string desc() const { return "SmartPlugin"; }

 private:
  // 获取单路图像，workflow配置的图像输入节点名字
  // SmartPlugin派生类可以根据需要修改输入节点的名字
  // 但是必须保证该接口返回的图像输入节点名字和xstream json配置中一致
  virtual std::string GetWorkflowInputImageName() {
    return "image";  // 当前沉淀的solution均使用image这个
  }

  // 创建xproto框架下感知结果的消息对象
  // 感知结果消息对象必须是CustomSmartMessage或者集成自CustomSmartMessage
  // 输入参数xstream_out为xstream workflow执行完成，xstream回调返回的数据对象
  virtual std::shared_ptr<CustomSmartMessage>
  CreateSmartMessage(xstream::OutputDataPtr xstream_out) {
    // 当前沉淀的解决方案，默认为CustomSmartMessage对象
    return std::make_shared<CustomSmartMessage>(xstream_out);
  }
  // 放在单独线程中实现
  void CreateSmartLegibleMessage(xstream::OutputDataPtr xstream_out);
  void GetWorkflowTargetsConfig();

  int Feed(XProtoMessagePtr msg);
  void OnCallback(xstream::OutputDataPtr out);
  void ParseConfig();
  int CodecInit(int image_width, int image_height);
  void CodecDeInit();
  void EncodeJpg(XProtoMessagePtr msg);
  void SmartImageMapProc();  // jpg和智能消息匹配
  int GetYUV(VideoEncodeSourceBuffer *frame_buf, VioMessage *vio_msg, int level,
             int use_vb);

  std::shared_ptr<XStreamSDK> sdk_;
  int sdk_monitor_interval_;
  std::string config_file_;
  std::shared_ptr<RuntimeMonitor> monitor_;
  std::shared_ptr<JsonConfigWrapper> config_;
  std::string xstream_workflow_cfg_file_;
  bool enable_profile_{false};
  std::string profile_log_file_;
  bool result_to_json_{false};
  bool dump_result_{false};
  Json::Value root_;
  bool run_flag_ = false;
  bool hand_id_merge_ = true;
  bool convert_keypoint_format_ = false;
  CThreadPool jpg_encode_thread_;
  CThreadPool smart_parse_thread_;
  CThreadPool map_thread_;
  CodecParam codec_param_;
  int chn_;
  bool codec_inited_;
  std::mutex codec_mutex_;
  const uint8_t cache_size_ = 20;  // max input cache size
  std::priority_queue<RawDataImageFramePtr, std::vector<RawDataImageFramePtr>,
                      compare_image>
      jpg_datas_;
  std::priority_queue<SmartLegibleMessagePtr,
                      std::vector<SmartLegibleMessagePtr>, compare_smart_msg>
      smart_msgs_;
  std::mutex map_smart_mutex_;
  std::condition_variable map_smart_condition_;
  /**
   * @description: workflow targets
   * @param std::string output filed name
   * @param std::string target name
   */
  std::map<std::string, std::string> workflow_targets_;
  /**
   * @description: output filed type
   * @param std::string output filed name
   * @param std::string output filed type
   */
  std::map<std::string, std::string> workflow_output_types_;
#ifdef USE_MC
  int OnApInfoMessage(const XProtoMessagePtr &msg);
#endif
};

}  // namespace xproto
#endif  // INCLUDE_SMARTPLUGIN_SMARTPLUGIN_H_
