/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <assert.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fasterrcnn_method/dump.h"
#include "fasterrcnn_method/fasterrcnn_method.h"
#include "fasterrcnn_method/yuv_utils.h"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"
#include "opencv2/opencv.hpp"
#include "xstream/vision_type.h"
#include "xstream/xstream_sdk.h"
// #include "./vio_wrapper_global.h"
#include "./block_queue.h"


using xstream::BaseData;
using xstream::BaseDataPtr;
using xstream::BaseDataVector;
using xstream::InputData;
using xstream::InputDataPtr;

using xstream::FasterRCNNMethod;
using xstream::ImageFrame;
using xstream::PymImageFrame;

static void Usage() {
  std::cout << "./FasterRCNNMethod_example two_faster_rcnn "
               "model_config_file vio_config_file image_list\n";
}

std::atomic<long long> frame_cnt(0);    // NOLINT
#ifdef X2
BlockQueue<img_info_t*> g_pyramid_img_queue;
HbVioFbWrapper *g_fb_vio = nullptr;
#endif
#ifdef X3
BlockQueue<std::shared_ptr<PymImageFrame>> g_pyramid_img_queue;
HbVioFbWrapperGlobal *g_fb_vio = nullptr;
#endif

void ThreadFun(FasterRCNNMethod *faster_rcnn, std::string model_config_file) {
  faster_rcnn->Init(model_config_file);
  while (1) {
    auto py_image_frame_ptr = g_pyramid_img_queue.Take();
    // auto xstream_pyramid = std::make_shared << ImageFrame > ();
    // xstream_pyramid->value = py_image_frame_ptr;

    std::vector<BaseDataPtr> input;
    xstream::InputParamPtr param;
    input.push_back(py_image_frame_ptr);

    auto xstream_output = faster_rcnn->DoProcess(input, param);

    auto faster_rcnn_out = xstream_output;
    g_fb_vio->FreeImgInfo(py_image_frame_ptr);
    std::cout << "predict success: " << frame_cnt++ << std::endl;
  }
}

int TestTwoFasterRCNN(int argc, char **argv) {
  if (argc < 4) {
    Usage();
    return -1;
  }
  FasterRCNNMethod faster_rcnn_method1;
  FasterRCNNMethod faster_rcnn_method2;
  std::string model_config_file = argv[1];
  std::string vio_config_file = argv[2];
  std::string image_list = argv[3];
  g_fb_vio = new HbVioFbWrapperGlobal(vio_config_file);
  int ret = g_fb_vio->Init();
  HOBOT_CHECK(ret == 0) << "fb vio init failed!!!";

  std::thread t1(ThreadFun, &faster_rcnn_method1, model_config_file);
  std::thread t2(ThreadFun, &faster_rcnn_method2, model_config_file);

  std::cout << "fasterrcnn init success." << std::endl;

  std::ifstream ifs(image_list);
  if (!ifs.is_open()) {
    std::cout << "open image list file failed." << std::endl;
    return -1;
  }

  std::string input_image;
  while (getline(ifs, input_image)) {
    /* 灌图片到内存 */
    cv::Mat bgr_img = cv::imread(input_image);
    int width = bgr_img.cols;
    int height = bgr_img.rows;
    cv::Mat img_nv12;
    bgr_to_nv12(bgr_img.data, height, width, img_nv12);

    /* 得到灌图片 */
    auto pym = g_fb_vio->GetImgInfo(img_nv12.data, width, height);
    HOBOT_CHECK(pym != nullptr) << "fb vio get image failed!!!";
    g_pyramid_img_queue.Push(pym);
  }
  t1.join();
  t2.join();
  hb_vio_stop();
  hb_vio_deinit();
  faster_rcnn_method1.Finalize();
  faster_rcnn_method2.Finalize();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
