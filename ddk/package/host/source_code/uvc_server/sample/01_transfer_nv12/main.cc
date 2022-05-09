#include <fstream>
#include <iostream>

#include "transfer_nv12.h"

static bool exit_ = false;
using uvc_sample::TestTransfer;

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cout
        << "[Usage]: uvc_sample nv12_path nv12_height nv12_width [ut normal]"
        << std::endl;
    return -1;
  }
  std::string run_mode = "ut";
  if (argc > 4) {
    run_mode.assign(argv[4]);
    std::cout << "run_mode:" << run_mode << std::endl;
    if (run_mode != "ut" && run_mode != "normal") {
      std::cout << "[ERROR] not support mode: " << run_mode << std::endl;
      return 0;
    }
  }
  // 1. read nv12 image data
  std::ifstream ifs(argv[1], std::ios::in | std::ios::binary);
  if (!ifs) {
    std::cout << "[ERROR] Open nv12 file: " << argv[1] << " failed"
              << std::endl;
    return -1;
  }
  ifs.seekg(0, std::ios::end);
  int img_length = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char *img_nv12 = new char[sizeof(char) * img_length];
  ifs.read(img_nv12, img_length);
  ifs.close();

  int height = atoi(argv[2]);
  int width = atoi(argv[3]);
  // 2. Create TestTransfer Instance
  std::shared_ptr<TestTransfer> uvc_sender = std::make_shared<TestTransfer>();
  // 3. Init TestTransfer
  auto ret = uvc_sender->Init();
  if (ret != 0) {
    std::cout << "[ERROR] uvc server init failed" << std::endl;
    return -1;
  }
  // 4. Start TestTransfer
  ret = uvc_sender->Start();
  if (ret != 0) {
    std::cout << "[ERROR] uvc server start failed" << std::endl;
    return -1;
  }
  // 5. Wait some time
  if (run_mode == "ut") {
    int count = 10000 / 40;
    while (!exit_ && count-- > 0) {
      uvc_sender->SendNv12Data(img_nv12, img_length, width, height);
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
  } else {
    while (!exit_) {
      uvc_sender->SendNv12Data(img_nv12, img_length, width, height);
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
  }
  // 6. Stop&DeInit
  uvc_sender->Stop();
  uvc_sender->DeInit();

  return 0;
}
