#include <cstdint>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>
#include <gtest/gtest.h>
#include "./rtsp_server_gtest.h"
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

const char *video_path[] = {
  "./data/chn0.264",
  "./data/dh5c.hevc"
};

using rtspcomponent::RtspServerConfig;
using rtspcomponent::RtspServer;

static const char* rtsp_config_file = "./configs/rtsp_server.json";


int RunMediaThread(int chn_id, std::shared_ptr<RtspServer> rtsp_server) {
  // get AVPacket
  AVPacket packet;
  bool restart_flag = true;
  int run_time = 60000;
  std::string media_source = video_path[chn_id];
  AVFormatContext* input_ctx = NULL;
  auto rtsp_config = std::make_shared<RtspServerConfig>(rtsp_config_file);
  rtsp_config->LoadConfig();
  auto video_type = rtsp_config->GetChnVideoType(chn_id);
  auto start_time = std::chrono::system_clock::now();
  while (1) {
    if (restart_flag == true) {
      // ffmpeg init
      avformat_open_input(&input_ctx, media_source.c_str(), NULL, NULL);
      av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    }
    restart_flag = false;
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGW << "run time end, cost_time: " << cost_time.count();
      break;
    }
    int ret = av_read_frame(input_ctx, &packet);
    if (ret < 0) {
      av_packet_unref(&packet);
      avformat_close_input(&input_ctx);
      restart_flag = true;
      continue;
    } else {
      // through channel chn_id forwarding H264orH265 file data
       rtsp_server->SendData(packet.data, packet.size, video_type, chn_id);
    }
    av_packet_unref(&packet);
    usleep(40000);
  }
  avformat_close_input(&input_ctx);
  return 0;
  }

// create rtsp server
TEST(RtspServerTest, CreateRtspServer) {
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  EXPECT_NE(rtsp_server, nullptr);
}

// rtsp server init successful
TEST(RtspServerTest, Init) {
  int ret = -1;
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  ASSERT_NE(rtsp_server, nullptr);
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
}

// rtsp server stop
TEST(RtspServerTest, Stop) {
  int ret = -1;
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  ASSERT_NE(rtsp_server, nullptr);
  ret = rtsp_server->Stop();
  EXPECT_EQ(ret, 0) << "rtsp server stop failed, ret: " << ret;
}

// rtsp server deinit
TEST(RtspServerTest, DeInit) {
  int ret = -1;
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  ASSERT_NE(rtsp_server, nullptr);
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
}


// rtsp server Init->DeInit
TEST(RtspServerTest, Init_DeInit_func) {
  int ret = -1;
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  ASSERT_NE(rtsp_server, nullptr);
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
}

// rtsp server DeInit->Init
TEST(RtspServerTest, DeInit_Init_func) {
  int ret = -1;
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  ASSERT_NE(rtsp_server, nullptr);
  // rtsp server has not init, deinit will not fail
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
}

// rtsp server start
TEST(RtspServerTest, StartProcess) {
  int ret = -1;
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  ASSERT_NE(rtsp_server, nullptr);
  // rtsp server init first and then start
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
  ret = rtsp_server->Start();
  EXPECT_EQ(ret, 0) << "rtsp server start failed, ret: " << ret;
  ret = rtsp_server->Stop();
  EXPECT_EQ(ret, 0) << "rtsp server stop failed, ret: " << ret;
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
}

// Send Chn0H264FileData To RTSPServer Successful
TEST(RTSPServerTest, H264Chn0FileSendToRTSPServer) {
  int ret = -1;
  int chn_id = 0;
  int run_time = 30000;  // 30s
  std::string media_source = video_path[chn_id];

  // create rtsp_server and rtsp_config
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  auto rtsp_config = std::make_shared<RtspServerConfig>(rtsp_config_file);
  ret = rtsp_config->LoadConfig();  // return ture if success
  EXPECT_EQ(ret, 1) << "rtsp server init failed, ret: " << ret;
  auto video_type = rtsp_config->GetChnVideoType(chn_id);

  // rtsp server init and start
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
  ret = rtsp_server->Start();
  EXPECT_EQ(ret, 0) << "rtsp server start failed, ret: " << ret;

  // get AVPacket
  AVPacket packet;
  bool restart_flag = true;
  AVFormatContext* input_ctx = NULL;
  auto start_time = std::chrono::system_clock::now();
  while (1) {
    if (restart_flag == true) {
      // ffmpeg init
      ret = avformat_open_input(&input_ctx, media_source.c_str(), NULL, NULL);
      EXPECT_EQ(ret, 0) << "open media file failed, ret: " << ret;
      ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
      EXPECT_EQ(ret, 0) << "av_find_best_stream failed, ret: " << ret;
    }
    restart_flag = false;
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGW << "run time end, cost_time: " << cost_time.count();
      break;
    }
    ret = av_read_frame(input_ctx, &packet);
    if (ret < 0) {
      av_packet_unref(&packet);
      avformat_close_input(&input_ctx);
      restart_flag = true;
      continue;
    } else {
      // through channel chn_id 0 forwarding H264 file data
      rtsp_server->SendData(packet.data, packet.size, video_type, chn_id);
    }
    av_packet_unref(&packet);
    usleep(40000);
  }
  avformat_close_input(&input_ctx);

  ret = rtsp_server->Stop();
  EXPECT_EQ(ret, 0) << "rtsp server stop failed, ret: " << ret;
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
}

// Send Chn1H265FileData To RTSPServer Successful
TEST(RTSPServerTest, H265Chn1FileSendToRTSPServer) {
  int ret = -1;
  int chn_id = 1;
  int run_time = 30000;  // 30s
  std::string media_source = video_path[chn_id];

  // create rtsp_server and rtsp_config
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  auto rtsp_config = std::make_shared<RtspServerConfig>(rtsp_config_file);
  ret = rtsp_config->LoadConfig();  // return ture if success
  EXPECT_EQ(ret, 1) << "rtsp server init failed, ret: " << ret;
  auto video_type = rtsp_config->GetChnVideoType(chn_id);

  // rtsp server init and start
  ret = rtsp_server->Init();
  EXPECT_EQ(ret, 0) << "rtsp server init failed, ret: " << ret;
  ret = rtsp_server->Start();
  EXPECT_EQ(ret, 0) << "rtsp server start failed, ret: " << ret;

  // get AVPacket
  AVPacket packet;
  bool restart_flag = true;
  AVFormatContext* input_ctx = NULL;
  auto start_time = std::chrono::system_clock::now();
  while (1) {
    if (restart_flag == true) {
      // ffmpeg init
      ret = avformat_open_input(&input_ctx, media_source.c_str(), NULL, NULL);
      EXPECT_EQ(ret, 0) << "open media file failed, ret: " << ret;
      ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
      EXPECT_EQ(ret, 0) << "av_find_best_stream failed, ret: " << ret;
    }
    restart_flag = false;
    auto curr_time = std::chrono::system_clock::now();
    auto cost_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          curr_time - start_time);
    if (cost_time.count() >= run_time) {
      LOGW << "run time end, cost_time: " << cost_time.count();
      break;
    }
    ret = av_read_frame(input_ctx, &packet);
    if (ret < 0) {
      av_packet_unref(&packet);
      avformat_close_input(&input_ctx);
      restart_flag = true;
      continue;
    } else {
      // through channel chn_id 1 forwarding H265 file data
      rtsp_server->SendData(packet.data, packet.size, video_type, chn_id);
    }
    av_packet_unref(&packet);
    usleep(40000);
  }
  avformat_close_input(&input_ctx);
  ret = rtsp_server->Stop();
  EXPECT_EQ(ret, 0) << "rtsp server stop failed, ret: " << ret;
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
}

TEST(RTSPServerTest, H264orH265FileSendToRTSPServer) {
  int ret = -1;
  auto rtsp_server = std::make_shared<RtspServer>(rtsp_config_file);
  rtsp_server->Init();
  rtsp_server->Start();
  std::shared_ptr<std::thread> get_frame_thread0 =
      std::make_shared<std::thread>(&RunMediaThread, 0, rtsp_server);
  std::shared_ptr<std::thread> get_frame_thread1 =
      std::make_shared<std::thread>(&RunMediaThread, 1, rtsp_server);
  if (get_frame_thread0->joinable()) {
    get_frame_thread0->join();
}
  if (get_frame_thread1->joinable()) {
    get_frame_thread1->join();
}
  ret = rtsp_server->Stop();
  EXPECT_EQ(ret, 0) << "rtsp server stop failed, ret: " << ret;
  ret = rtsp_server->DeInit();
  EXPECT_EQ(ret, 0) << "rtsp server deinit failed, ret: " << ret;
}



