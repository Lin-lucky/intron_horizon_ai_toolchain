#ifndef RTSP_SERVER_CONFIG_H_
#define RTSP_SERVER_CONFIG_H_
#include <fstream>
#include <string>
#include <vector>

namespace Json{
  class Value;
}

namespace rtspcomponent {
enum Video_Type { H264 = 0, H265 };
enum Audio_Type { G711 = 2, G726 };

class RtspServerConfig {
  friend class RtspServer;
 public:
  explicit RtspServerConfig(const std::string &path);
  ~RtspServerConfig();
  bool LoadConfig();
  std::string GetStringValue(const std::string &key);  // 获取参数值
  int GetIntValue(const std::string &key);
  std::string GetChnStringValue(int chn, const std::string &key);
  int GetChnIntValue(int chn, const std::string &key);
  Video_Type GetChnVideoType(int chn);
  Audio_Type GetChnAudioType(int chn);
  std::string GetChnStreamName(int chn);

 private:
  bool CheckConfig();

 private:
  uint8_t auth_mode_;  // 0 for none
  std::string user_, password_;
  int port_;

  int chn_num_;
  std::vector<std::string> stream_name_;
  std::vector<Video_Type> video_type_;
  std::vector<Audio_Type> audio_type_;

  std::string path_;
  Json::Value *json_ = nullptr;
};
}  // namespace rtspcomponent
#endif
