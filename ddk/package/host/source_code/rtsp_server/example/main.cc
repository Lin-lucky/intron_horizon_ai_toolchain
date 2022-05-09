#include <iostream>
#include <signal.h>
#include "media_producer.h"
static bool exit_ = false;
using rtspcomponent::MediaProducer;

// usage:
// ./rtsp_server_example ./rtsp.json
static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    exit_ = true;
  }
}

int main(int argc, char **argv) {
  if (argv[1] == nullptr) {
    std::cout << "rtsp config file is null" << std::endl;
    return -1;
  }

  std::string rtsp_config_file = std::string(argv[1]);
  std::string log_level;
  if (argv[2] == nullptr) {
    std::cout << "set default log level: [-i] " << std::endl;
    log_level = "-i";
  } else {
    log_level = argv[2];
  }
  if (log_level == "-i") {
    SetLogLevel(HOBOT_LOG_INFO);
  } else if (log_level == "-d") {
    SetLogLevel(HOBOT_LOG_DEBUG);
  } else if (log_level == "-w") {
    SetLogLevel(HOBOT_LOG_WARN);
  } else if (log_level == "-e") {
    SetLogLevel(HOBOT_LOG_ERROR);
  } else if (log_level == "-f") {
    SetLogLevel(HOBOT_LOG_FATAL);
  } else {
    std::cout << "set default log level: [-i] " << std::endl;
    SetLogLevel(HOBOT_LOG_INFO);
  }
  std::string run_mode = "normal";
  if (argc > 3) {
    run_mode.assign(argv[3]);
    LOGD << "run_mode:" << run_mode;
    if (run_mode != "ut" && run_mode != "normal" && run_mode != "loop") {
      LOGE << "not support mode: " << run_mode;
      return 0;
    }
  }

  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);
  signal(SIGSEGV, signal_handle);

  std::shared_ptr<MediaProducer> media_producer =
    std::make_shared<MediaProducer>(rtsp_config_file);

  auto ret = media_producer->Init();
  if (ret != 0) {
    LOGE << "rtsp server init failed";
    return -1;
  }

  ret = media_producer->Start();
  if (ret != 0) {
    LOGE << "rtsp server start failed";
    return -1;
  }
  if (run_mode == "ut") {
    std::this_thread::sleep_for(std::chrono::seconds(10));  // ut test
  } else {
    while (!exit_) {
      if (run_mode == "normal") {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
      } else if (run_mode == "loop") {
        std::this_thread::sleep_for(std::chrono::seconds(60));
        media_producer->Stop();
        media_producer->DeInit();
        std::this_thread::sleep_for(std::chrono::seconds(5));

        auto ret = media_producer->Init();
        if (ret != 0) {
          LOGE << "rtsp server init failed";
          return -1;
        }
        ret = media_producer->Start();
        if (ret != 0) {
          LOGE << "rtsp server start failed";
          return -1;
        }
      }
    }
  }
  LOGW << "wait to quit";
  media_producer->Stop();
  media_producer->DeInit();

  return 0;
}
