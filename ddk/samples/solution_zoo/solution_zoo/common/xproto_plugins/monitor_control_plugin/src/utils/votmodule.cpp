/*
 * @Description:
 * @Author: xx@horizon.ai
 * @Date: 2020-06-22 16:17:25
 * @Copyright 2017~2019 Horizon Robotics, Inc.
 */

#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <string.h>
#include <fstream>
#include <vector>
#include "hobotlog/hobotlog.hpp"
#include "opencv2/opencv.hpp"
#include "utils/votmodule.h"
#include "utils/smart_vision_type.h"
#include "xproto/msg_type/protobuf/x3.pb.h"
namespace xproto {

using horizon::vision::MediaCodecManager;
static int fbfd;
char *fb = NULL;
int hb_vot_init(void)
{
//  int ret = 0;
//  VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
//  VOT_CHN_ATTR_S stChnAttr;
  struct fb_var_screeninfo vinfo;
  struct fb_fix_screeninfo finfo;

  fbfd = open("/dev/fb0", O_RDWR);
  if (!fbfd) {
    printf("Error: cannot open framebuffer device1.\n");
    HOBOT_CHECK(0);
  }
  if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo)) {
    printf("Error reading fixed information.\n");
    HOBOT_CHECK(0);
  }

  /* Get variable screen information */
  if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo)) {
    printf("Error reading variable information.\n");
    HOBOT_CHECK(0);
  }
  printf("vinfo.xres=%d\n", vinfo.xres);
  printf("vinfo.yres=%d\n", vinfo.yres);
  printf("vinfo.bits_per_bits=%d\n", vinfo.bits_per_pixel);
  printf("vinfo.xoffset=%d\n", vinfo.xoffset);
  printf("vinfo.yoffset=%d\n", vinfo.yoffset);
  printf("finfo.line_length=%d\n", finfo.line_length);
  printf("finfo.left_margin = %d\n", vinfo.left_margin);
  printf("finfo.right_margin = %d\n", vinfo.right_margin);
  printf("finfo.upper_margin = %d\n", vinfo.upper_margin);
  printf("finfo.lower_margin = %d\n", vinfo.lower_margin);
  printf("finfo.hsync_len = %d\n", vinfo.hsync_len);
  printf("finfo.vsync_len = %d\n", vinfo.vsync_len);
  fb = (char *)mmap(0, 1920 * 1080 * 4, PROT_READ | PROT_WRITE,  // NOLINT
                    MAP_SHARED, fbfd, 0);
  if (fb == (void*)(-1)) {  // NOLINT
    printf("Error: failed to map framebuffer device to memory.\n");
    HOBOT_CHECK(0);
  }
  memset(fb, 0x0, 1920 * 1080 * 4);
  return 0;
}

VotModule::VotModule(const std::string& config):group_id_(-1), stop_(true) {
  // parse config
  if (!config.empty()) {
    LOGI << "vot config file: " << config;
    Json::Value cfg_jv;
    std::ifstream infile(config);
    if (infile) {
      infile >> cfg_jv;
      config_.reset(new JsonConfigWrapper(cfg_jv));
    } else {
      LOGE << "open vot config fail";
    }
  }

//  hb_vot_init();
}

VotModule::~VotModule() {
  std::unique_lock<std::mutex> lk(send_vot_mtx_);
  send_vot_cache_.clear();
  lk.unlock();
  free(buffer_);
}

int VotModule::Start() {
  if (!stop_) return 0;
  stop_ = false;

  // plot task
  auto plot_func = [this](){
      while (!stop_) {
        std::shared_ptr<VotData_t> vot_data;
        auto is_getitem = in_queue_.try_pop(&vot_data,
                                         std::chrono::milliseconds(1000));
        if (!is_getitem || !vot_data) {
          continue;
        }

        if (vot_data->is_drop_frame_ || !vot_data->sp_smart_pb_) {
          if (!plot_drop_img_) {
            continue;
          }
        }

        std::shared_ptr<char> sp_buf = PlotImage(vot_data);
        if (!sp_buf) {
          continue;
        }

        std::unique_lock<std::mutex> lk(send_vot_mtx_);
        send_vot_cache_[vot_data->sp_img_->frame_id_] = sp_buf;
        send_vot_cv_.notify_one();
      }
    };
  for (uint32_t idx = 0; idx < plot_task_num_; idx++) {
    plot_tasks_.emplace_back(std::make_shared<std::thread>(plot_func));
  }

  // send vot display task
  auto send_vot_func = [this](){
      auto last_send_vot_tp = std::chrono::system_clock::now();
      while (!stop_) {
        std::unique_lock<std::mutex> lk(send_vot_mtx_);
        send_vot_cv_.wait_for(lk, std::chrono::milliseconds(100),
                          [this] () {
            return stop_ || send_vot_cache_.size() >= in_queue_len_max_ ||
                   (!send_vot_cache_.empty() &&
                    send_vot_cache_.begin()->first == need_send_vot_id_);
        });

        if (stop_) {
          send_vot_cache_.clear();
          lk.unlock();
          break;
        }

        std::shared_ptr<char> sp_buf = nullptr;
        if (send_vot_cache_.size() >= in_queue_len_max_ ||
                (!send_vot_cache_.empty() &&
                        send_vot_cache_.begin()->first == need_send_vot_id_)) {
          sp_buf = send_vot_cache_.begin()->second;
          need_send_vot_id_ = send_vot_cache_.begin()->first + 1;
          send_vot_cache_.erase(send_vot_cache_.begin());
          lk.unlock();
        } else {
          lk.unlock();
          continue;
        }

        if (!buffer_ || !sp_buf) continue;

        auto dura = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - last_send_vot_tp).count();
        if (frame_fps_ > 0 && dura + 5 < 1000 / frame_fps_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1000 /
                                      frame_fps_ - dura));
        }
        LOGD << "plot frame time diff ms:"
             << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::system_clock::now() -
                             last_send_vot_tp).count();
        last_send_vot_tp = std::chrono::system_clock::now();

        memcpy(buffer_, sp_buf.get(), image_data_size_);
        static VOT_FRAME_INFO_S stFrame;
        stFrame.addr = buffer_;
        stFrame.size = image_data_size_;
        HB_VOT_SendFrame(0, channel_, &stFrame, -1);

        if (en_encoder_ && 0 == encoder_input_) {
          for (int i = 0; i < venc_param_.height; ++i) {
            memcpy(venc_param_.mmz_vaddr + i * venc_param_.width,
                   buffer_ + i * venc_param_.width, venc_param_.width);
          }
          for (int i = 0; i < venc_param_.height / 2; ++i) {
            memcpy(venc_param_.mmz_vaddr +
                           (i + venc_param_.height) * venc_param_.width,
                   buffer_ + venc_param_.width * venc_param_.height +
                           i * venc_param_.width,
               venc_param_.width);
          }
          int ret = HB_VENC_SendFrame(venc_param_.veChn, &pstFrame, 20);
          if (ret != 0) {
            LOGE << "HB_VENC_SendStream Failed. ret = " << ret;
          }
        }
      }
  };
  send_vot_task_ = std::make_shared<std::thread>(send_vot_func);

//  display_task_ = std::make_shared<std::thread>([this]{
//    while (!stop_) {
//      static VOT_FRAME_INFO_S stFrame {buffer_, image_data_size_};
//      HB_VOT_SendFrame(0, channel_, &stFrame, -1);
//      std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    }
//  });


  if (InitCodecManager() != 0) {
    LOGE << "init codec fail";
    return -1;
  }

  if (en_encoder_ && !recv_stream_task_) {
    auto exec = [this](){
        VIDEO_STREAM_S vstream;
        while (!stop_) {
          memset(&vstream, 0, sizeof(VIDEO_STREAM_S));
          auto ret = HB_VENC_GetStream(venc_param_.veChn, &vstream, 2000);
          if (ret < 0) {
            LOGE << "HB_VENC_GetStream timeout: " << ret;
            continue;
          } else {
            HOBOT_CHECK(vstream.pstPack.size > 5)
            << "encode bitstream too small";
            if (ofs_encoder_output_.good()) {
              ofs_encoder_output_.write(vstream.pstPack.vir_ptr,
                                        vstream.pstPack.size);
            }
            HB_VENC_ReleaseStream(0, &vstream);
          }
        }
    };
    recv_stream_task_ = std::make_shared<std::thread>(exec);
  }

  return 0;
}

int VotModule::Init() {
  int ret = 0;

  if (config_) {
    if (config_->HasKey("en_encoder")) {
      en_encoder_ = config_->GetBoolValue("en_encoder");
    }
    if (config_->HasKey("encoder_input")) {
      encoder_input_ = config_->GetBoolValue("encoder_input");
    }
    if (config_->HasKey("encoder_output_save_file")) {
      encoder_output_save_file_ =
              config_->GetSTDStringValue("encoder_output_save_file");
    }
    if (config_->HasKey("en_bgr_convert")) {
      plot_color_ = config_->GetBoolValue("en_bgr_convert");
    }
    if (config_->HasKey("en_fps_draw")) {
      vot_draw_ctrl_.draw_fps = config_->GetBoolValue("en_fps_draw");
    }
    if (config_->HasKey("en_gesture_val_draw")) {
      vot_draw_ctrl_.draw_gesture_val =
              config_->GetBoolValue("en_gesture_val_draw");
    }
    if (config_->HasKey("en_handid_draw")) {
      vot_draw_ctrl_.draw_hand_id = config_->GetBoolValue("en_handid_draw");
    }
  }

  image_height_ = 1080;
  image_width_ = 1920;
  image_data_size_ = image_width_ * image_height_ * 3 / 2;

  VOT_VIDEO_LAYER_ATTR_S stLayerAttr;
  VOT_CHN_ATTR_S stChnAttr;
  VOT_CROP_INFO_S cropAttrs;
  VOT_PUB_ATTR_S devAttr;

  devAttr.enIntfSync = VOT_OUTPUT_1920x1080;
  devAttr.u32BgColor = 0x108080;
  devAttr.enOutputMode = HB_VOT_OUTPUT_BT1120;
  ret = HB_VOT_SetPubAttr(0, &devAttr);
  if (ret) {
      printf("HB_VOT_SetPubAttr failed\n");
      return -1;
  }
  ret = HB_VOT_Enable(0);
  if (ret) printf("HB_VOT_Enable failed.\n");

  ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
  if (ret) {
      printf("HB_VOT_GetVideoLayerAttr failed.\n");
  }
  stLayerAttr.stImageSize.u32Width  = 1920;
  stLayerAttr.stImageSize.u32Height = 1080;

  stLayerAttr.panel_type = 0;
  stLayerAttr.rotate = 0;
  stLayerAttr.dithering_flag = 0;
  stLayerAttr.dithering_en = 0;
  stLayerAttr.gamma_en = 0;
  stLayerAttr.hue_en = 0;
  stLayerAttr.sat_en = 0;
  stLayerAttr.con_en = 0;
  stLayerAttr.bright_en = 0;
  stLayerAttr.theta_sign = 0;
  stLayerAttr.contrast = 0;
  stLayerAttr.theta_abs = 0;
  stLayerAttr.saturation = 0;
  stLayerAttr.off_contrast = 0;
  stLayerAttr.off_bright = 0;
  stLayerAttr.user_control_disp = 0;
  stLayerAttr.big_endian = 0;
  stLayerAttr.display_addr_type = 2;
  stLayerAttr.display_addr_type_layer1 = 2;
  ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
  if (ret) printf("HB_VOT_SetVideoLayerAttr failed.\n");

  ret = HB_VOT_EnableVideoLayer(0);
  if (ret) printf("HB_VOT_EnableVideoLayer failed.\n");

  stChnAttr.u32Priority = 2;
  stChnAttr.s32X = 0;
  stChnAttr.s32Y = 0;
  stChnAttr.u32SrcWidth = 1920;
  stChnAttr.u32SrcHeight = 1080;
  stChnAttr.u32DstWidth = 1920;
  stChnAttr.u32DstHeight = 1080;
  ret = HB_VOT_SetChnAttr(0, channel_, &stChnAttr);
  printf("HB_VOT_SetChnAttr 0: %d\n", ret);

  cropAttrs.u32Width = stChnAttr.u32DstWidth;
  cropAttrs.u32Height = stChnAttr.u32DstHeight;
  ret = HB_VOT_SetChnCrop(0, channel_, &cropAttrs);
  printf("HB_VOT_EnableChn: %d\n", ret);

  ret = HB_VOT_EnableChn(0, channel_);
  printf("HB_VOT_EnableChn: %d\n", ret);

  buffer_ = (char *)malloc(image_data_size_);// NOLINT

  if (en_encoder_) {
    ofs_encoder_output_.open(encoder_output_save_file_);
    if (ofs_encoder_output_.good()) {
      LOGD << "save encoder output to file " << encoder_output_save_file_;
    } else {
      LOGE << "error! open file " << encoder_output_save_file_;
      return -1;
    }

    // alloc ion buffer for video encoder
    // 1. init
    VP_CONFIG_S vp_config;
    memset(&vp_config, 0x00, sizeof(VP_CONFIG_S));
    vp_config.u32MaxPoolCnt = venc_param_.vp_pool_cnt;
    ret = HB_VP_SetConfig(&vp_config);
    if (ret) {
      if (ret == HB_ERR_VP_BUSY) {
        LOGW << "hb vp have already config, ret: " << ret;
      } else {
        LOGE << "hb vp config failed, ret: " << ret;
        return ret;
      }
    }
    // feedback mode has already init in vio plugin
    if (ret != HB_ERR_VP_BUSY) {
      ret = HB_VP_Init();
      if (ret != 0) {
        LOGE << "hb vp init failed, ret: " << ret;
        return ret;
      }
      venc_param_.vp_init_local = true;
    }

    // 2. alloc
    venc_param_.mmz_size = venc_param_.width * venc_param_.height * 3 / 2;
    ret = HB_SYS_Alloc(&venc_param_.mmz_paddr,
                       reinterpret_cast<void **>(&venc_param_.mmz_vaddr),
                       venc_param_.mmz_size);
    if (ret != 0) {
      LOGE << "HB_SYS_Alloc failed, ret: " << ret;
      return ret;
    }
    // 3. set venc input frame
    memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
    pstFrame.stVFrame.width = venc_param_.width;
    pstFrame.stVFrame.height = venc_param_.height;
    pstFrame.stVFrame.size = venc_param_.mmz_size;
    pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
    pstFrame.stVFrame.phy_ptr[0] = venc_param_.mmz_paddr;
    pstFrame.stVFrame.phy_ptr[1] =
            venc_param_.mmz_paddr + venc_param_.width * venc_param_.height;
    pstFrame.stVFrame.vir_ptr[0] = venc_param_.mmz_vaddr;
    pstFrame.stVFrame.vir_ptr[1] =
            venc_param_.mmz_vaddr + venc_param_.width * venc_param_.height;
    pstFrame.stVFrame.pts = 0;
  }

  return ret;
}

void VotModule::bgr_to_nv12(uint8_t *bgr, char *buf) {
  cv::Mat bgr_mat(image_height_, image_width_, CV_8UC3, bgr);
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);

  int uv_height = image_height_ / 2;
  int uv_width = image_width_ / 2;
  // copy y data
  int y_size = uv_height * uv_width * 4;

  uint8_t *yuv = yuv_mat.ptr<uint8_t>();
  memcpy(buf, yuv, y_size);

  // copy uv data
  int uv_stride = uv_width * uv_height;
  char *uv_data = buf + y_size;
  for (int i = 0; i < uv_stride; ++i) {
    *(uv_data++) = *(yuv + y_size + i);
    *(uv_data++) = *(yuv + y_size + +uv_stride + i);
  }

//  {
//    auto start = std::chrono::high_resolution_clock::now();
//    auto end = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double, std::milli> cost = end - start;
//    LOGD << "bgr_to_nv12 cost ms " << cost.count();
//  }
}

std::shared_ptr<char>
VotModule::PlotImage(const std::shared_ptr<VotData_t>& vot_data) {
  HOBOT_CHECK(vot_data->sp_img_);

  if (!vot_data->sp_img_) {
    LOGE << "invalid image in smart frame";
    return nullptr;
  }

  const auto& image = vot_data->sp_img_;
  const auto& height = image->height_;
  const auto& width = image->width_;
  if (height <= 0 || width <= 0 ||
      image->DataSize() != height * width * 3 / 2) {
    LOGE << "pyrmid: " << width << "x" << height;
    HOBOT_CHECK(0);
    return nullptr;
  }
  auto *img_addr = image->data_;
  auto start = std::chrono::high_resolution_clock::now();

//  {
//    static int count = 0;
//    std::ofstream ofs("pym_vot_" + std::to_string(count++) + ".yuv");
//    ofs.write(reinterpret_cast<char*>(img_addr), image.data_size);
//  }

  cv::Mat bgr;
  if (!plot_color_) {
    bgr = cv::Mat(1080, 1920, CV_8UC1, img_addr);
  } else {
  cv::cvtColor(cv::Mat(height * 3 / 2, width, CV_8UC1, img_addr),
               bgr, CV_YUV2BGR_NV12);
  }

  {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> cost = end - start;
    LOGD << "cvt bgr cost ms " << cost.count();
    start = std::chrono::high_resolution_clock::now();
  }

  if (image_width_ != image->width_ ||
          image_height_ != image->height_) {
    HOBOT_CHECK(image_width_ / image->width_ ==
                image_height_ / image->height_);
    float scale_factor = static_cast<float>(image_width_) /
                         static_cast<float>(image->width_);
    cv::resize(bgr, bgr, cv::Size(), scale_factor, scale_factor);
  }

  const static std::map<std::string, decltype(CV_RGB(255, 0, 0))> d_color = // NOLINT
          {{"id", CV_RGB(255, 0, 0)},
           {"face", CV_RGB(248, 250, 143)},
           {"head", CV_RGB(0, 255, 0)},
           {"body", CV_RGB(129, 211, 248)},
           {"hand", CV_RGB(128, 255, 255)},
           {"lmk", CV_RGB(128, 255, 255)},
           {"kps", CV_RGB(163, 0, 20)},
           {"fps", CV_RGB(255, 0, 255)}
          };

  LOGD << "plot frame_id " << vot_data->sp_img_->frame_id_;
  std::string plot_frame_id = "frame id "
                              + std::to_string(vot_data->sp_img_->frame_id_);
  cv::putText(bgr, plot_frame_id,
              cv::Point(10, 60),
              cv::HersheyFonts::FONT_HERSHEY_PLAIN,
              1.5, d_color.at("fps"), 2);

//  {
//    static int count = 0;
//    std::string pic_name{"plot_" + std::to_string(count++) + ".jpg"};
//    LOGD << "pic_name:" << pic_name;
//    cv::imwrite(pic_name, bgr);
//  }

  // plot smart data from pb
  x3::MessagePack pack;
  x3::FrameMessage frame;
//  HOBOT_CHECK(vot_data->sp_smart_pb_);
//  HOBOT_CHECK(pack.ParseFromString(*vot_data->sp_smart_pb_));
//  HOBOT_CHECK(frame.ParseFromString(pack.content_()));
//  HOBOT_CHECK(frame.has_smart_msg_());

  if (vot_data->sp_smart_pb_ &&
          pack.ParseFromString(*vot_data->sp_smart_pb_) &&
          frame.ParseFromString(pack.content_()) &&
          frame.has_smart_msg_()) {
    LOGD << "plot frame_id " << vot_data->sp_img_->frame_id_
         << "  targets__size:" << frame.smart_msg_().targets__size();

    if (frame.has_statistics_msg_()) {
      for (int idx = 0; idx < frame.statistics_msg_().attributes__size();
           idx++) {
        if (frame.statistics_msg_().attributes_(idx).type_() == "fps" &&
                frame.statistics_msg_().attributes_(idx).value_() > 0) {
          frame_fps_ = frame.statistics_msg_().attributes_(idx).value_();
        }
      }
    }
    if (vot_draw_ctrl_.draw_fps && frame_fps_ > 0) {
      cv::putText(bgr, "fps " + std::to_string(frame_fps_),
                  cv::Point(10, 20),
                  cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                  1.5, d_color.at("fps"), 2);
    }

    for (int idx_tar = 0; idx_tar < frame.smart_msg_().targets__size();
         idx_tar++) {
      const auto& target = frame.smart_msg_().targets_(idx_tar);
      HorizonVisionSmartData s_data;
      s_data.face = NULL;
      s_data.body = NULL;
      s_data.track_id = target.track_id_();
      HorizonVisionFaceSmartData face;
      HorizonVisionBodySmartData body;
//      HorizonVisionLandmarks landmarks;
      std::stringstream ss_attr;

      HorizonVisionBBox face_rect;
      HorizonVisionBBox head_rect;
      HorizonVisionBBox hand_rect;
      hand_rect.y1 = 0;
      hand_rect.x1 = 0;
      hand_rect.x2 = 0;
      hand_rect.y2 = 0;
      hand_rect.id = 0;
      std::vector<cv::Point> contours;
      bool has_face = false;
      bool has_head = false;
      bool has_hand = false;
      bool has_gesture_move = false;

      LOGD << "track_id " << s_data.track_id;

      if (target.boxes__size() > 0) {
        // box
        for (int idx = 0; idx < target.boxes__size(); idx++) {
          const auto &box = target.boxes_(idx);
          LOGD << "box:" << box.type_()
               << " x1:" << box.top_left_().x_()
               << " y1:" << box.top_left_().y_()
               << " x2:" << box.bottom_right_().x_()
               << " y2:" << box.bottom_right_().y_();

          HorizonVisionBBox rect;
          rect.x1 = box.top_left_().x_();
          rect.y1 = box.top_left_().y_();
          rect.x2 = box.bottom_right_().x_();
          rect.y2 = box.bottom_right_().y_();
          rect.score = box.score();
//          rect.id = box.score();  // for debug
          if ("face" == box.type_()) {
            has_face = true;
            face_rect = rect;
          } else if ("head" == box.type_()) {
            has_head = true;
            head_rect = rect;
          } else if ("body" == box.type_()) {
            body.body_rect = rect;
            s_data.body = &body;
          } else if ("hand" == box.type_()) {
            has_hand = true;
            hand_rect = rect;
            hand_rect.id = target.track_id_();
          }
        }
        if (has_face || has_head) {
          s_data.face = &face;
          if (has_face) {
            face.face_rect = face_rect;
          }
          if (has_head) {
            face.head_rect = head_rect;
          }
        }
      }

      // points
      for (int idx = 0; idx < target.points__size(); idx++) {
        const auto point = target.points_(idx);
        LOGD << "point type:" << point.type_();
        if (point.type_() == "lmk_106pts") {
          if (vot_draw_ctrl_.draw_face_lmk) {
            for (int lmk_idx = 0; lmk_idx < point.points__size();
                 lmk_idx++) {
              cv::circle(bgr,
                         cv::Point(point.points_(lmk_idx).x_(),
                                   point.points_(lmk_idx).y_()),
                         1, d_color.at("lmk"), 1);
            }
          }
        }

        if (point.type_() == "gesture_move") {
          has_gesture_move = true;
          for (int lmk_idx = 0; lmk_idx < point.points__size();
               lmk_idx++) {
//              cv::circle(bgr,
//                         cv::Point(point.points_(lmk_idx).x_(),
//                                   point.points_(lmk_idx).y_()),
//                         1, CV_RGB(0, 0, 255), 1);
            contours.push_back(
                    cv::Point(point.points_(lmk_idx).x_(),
                              point.points_(lmk_idx).y_()));
          }
        }

        if ("body_landmarks" == point.type_() && vot_draw_ctrl_.draw_body_kps) {
      #if 0
          for (int lmk_idx = 0; lmk_idx < 5; lmk_idx++) {
            cv::circle(bgr,
                       cv::Point(point.points_(lmk_idx).x_(),
                                 point.points_(lmk_idx).y_()),
                       2, CV_RGB(163, 0, 20), 2);
          }
      #endif
          for (int lmk_idx = 5; lmk_idx < point.points__size(); lmk_idx++) {
            cv::circle(bgr,
                       cv::Point(point.points_(lmk_idx).x_(),
                                 point.points_(lmk_idx).y_()),
                       8, CV_RGB(163, 0, 20), -1);
          }
        }

        if (point.type_() == "hand_landmarks" && vot_draw_ctrl_.draw_hand_lmk) {
          for (int lmk_idx = 0; lmk_idx < point.points__size(); lmk_idx++) {
            cv::circle(bgr,
                       cv::Point(point.points_(lmk_idx).x_(),
                                 point.points_(lmk_idx).y_()),
                       2, CV_RGB(255, 0, 0), 2);
            cv::putText(bgr, std::to_string(lmk_idx),
                        cv::Point(point.points_(lmk_idx).x_(),
                                  point.points_(lmk_idx).y_()),
                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                        1, CV_RGB(255, 255, 255), 1);
          }
        }
      }

      // attr
      bool has_rotate_res = false;
      float fit_err = -1.0;
      float fit_ratio = -1.0;
      int rotate_num = 0;
      int rotate_angel = 0;
      bool has_fit_rotated = false;
      cv::RotatedRect fit_rotated_rect;

      int gesture = 0;
      int gesture_orig = -1;
      int gesture_raw = -1;
      float gesture_score = 0;
      int gesture_direction = -1;
      int age = -1;
      int gender = 0;
      int dist = -1;

      for (int idx_mtx = 0; idx_mtx < target.float_matrixs__size();
           idx_mtx++) {
        const auto& matrix = target.float_matrixs_(idx_mtx);
        LOGI << "matrix type:" << matrix.type_();
        if (matrix.type_() == "rotate_res") {
          for (int idx_array = 0; idx_array < matrix.arrays__size();
               idx_array++) {
            const auto& mtx_array = matrix.arrays_(idx_array);
            LOGI << "array type:" << mtx_array.type_();
            if (mtx_array.type_() == "statistics_res" &&
                    mtx_array.value__size() >= 4) {
              has_rotate_res = true;
              fit_err = mtx_array.value_(0);
              fit_ratio = mtx_array.value_(1);
              rotate_num = mtx_array.value_(2);
              rotate_angel = mtx_array.value_(3);
            } else if (mtx_array.type_() == "fit_res" &&
                       mtx_array.value__size() >= 5) {
              has_fit_rotated = true;
              fit_rotated_rect.angle = mtx_array.value_(0);
              fit_rotated_rect.center.x = mtx_array.value_(1);
              fit_rotated_rect.center.y = mtx_array.value_(2);
              fit_rotated_rect.size.width = mtx_array.value_(3);
              fit_rotated_rect.size.height = mtx_array.value_(4);
            }
          }
        }
      }

      for (int idx = 0; idx < target.attributes__size(); idx++) {
        const auto& attr = target.attributes_(idx);
        LOGD << "frame:" << vot_data->sp_img_->frame_id_
             << " hand id:" << hand_rect.id
             << " attr type:" << attr.type_()
             << " val:" << attr.value_()
             << " score:" << target.attributes_(idx).score_();
//        ss_attr << " " << attr.type_() << ":" << attr.value_();

        if (attr.type_() == "gesture") {
          gesture = attr.value_();
        }
        if (attr.type_() == "gesture_orig") {
          gesture_orig = attr.value_();
        }
        if (attr.type_() == "gesture_raw") {
          gesture_raw = attr.value_();
          gesture_score = attr.score_();
        }
        if (attr.type_() == "gesture_direction") {
          gesture_direction = attr.value_();
        }
        if (attr.type_() == "age") {
          age = attr.value_();
        }
        if (attr.type_() == "gender") {
          gender = attr.value_();
        }
        if (attr.type_() == "dist") {
          dist = attr.value_();
        }
      }

      if (vot_draw_ctrl_.draw_age_gender_dist) {
        cv::Point draw_attr_point;
        draw_attr_point.x = -1;
        draw_attr_point.y = -1;
        if (has_face) {
          draw_attr_point.x = face_rect.x1;
          draw_attr_point.y = face_rect.y1;
        } else if (has_head) {
          draw_attr_point.x = head_rect.x1;
          draw_attr_point.y = head_rect.y1;
        }
        int y_offset = 20;
        if (draw_attr_point.x > 0 && draw_attr_point.y > 0) {
          if (age >= 0 && draw_attr_point.y > y_offset) {
            draw_attr_point.y -= y_offset;
            cv::putText(bgr, "age:" + std::to_string(age),
                        draw_attr_point,
                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                        1.5, CV_RGB(255, 255, 255), 1);
          }

          if (gender != 0 && draw_attr_point.y > y_offset) {
            draw_attr_point.y -= y_offset;
            //    "1": "男",  "-1": "女"
            std::string str_gender = gender == 1 ? "male" : "female";
            cv::putText(bgr, "gender:" + str_gender,
                        draw_attr_point,
                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                        1.5, CV_RGB(255, 255, 255), 1);
          }

          if (dist >= 0 && draw_attr_point.y > y_offset) {
            draw_attr_point.y -= y_offset;
            cv::putText(bgr, "dist:" + std::to_string(dist) + "cm",
                        draw_attr_point,
                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                        1.5, CV_RGB(255, 255, 255), 1);
          }
        }
      }

      std::string gesture_type{""};
      if (gesture < static_cast<int>(gesture_type_str.size()) && gesture >= 0) {
        gesture_type = gesture_type_str[gesture];
      }
      if (has_hand) {
        LOGD << "plot frame:" << vot_data->sp_img_->frame_id_
             << "  hand id:" << hand_rect.id
             << "  gesture_type:" << gesture_type;
        cv::rectangle(bgr,
                      cv::Point(hand_rect.x1, hand_rect.y1),
                      cv::Point(hand_rect.x2, hand_rect.y2),
                      CV_RGB(128, 255, 255), 2);

        // plot original gesture type
        if (vot_draw_ctrl_.draw_gesture_val) {
          auto draw_gesture_pt = cv::Point2f(hand_rect.x1, hand_rect.y2);
          draw_gesture_pt.y += 20;
          cv::putText(bgr, "raw:" + std::to_string(gesture_raw) + " " +
                              std::to_string(gesture_score).substr(0, 4),
                      draw_gesture_pt,
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                      2, CV_RGB(255, 255, 255), 1);
          draw_gesture_pt.y += 20;
          cv::putText(bgr, "vote:" + std::to_string(gesture_orig),
                      draw_gesture_pt,
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                      2, CV_RGB(255, 255, 255), 1);
          draw_gesture_pt.y += 20;
          cv::putText(bgr, "gest:" + std::to_string(gesture),
                      draw_gesture_pt,
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                      2, CV_RGB(255, 255, 255), 1);
        }

        // hand id
        if (vot_draw_ctrl_.draw_hand_id) {
          int offset_factor = 1;
          if (hand_rect.id < 10) {
            offset_factor = 1;
          } else if (hand_rect.id < 100) {
            offset_factor = 2;
          } else {
            offset_factor = 3;
          }
          cv::putText(bgr, std::to_string(hand_rect.id),
                      cv::Point2f(hand_rect.x1 - 20 * offset_factor,
                                  hand_rect.y1 + 20),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                      2, CV_RGB(255, 255, 255), 2);
        }

        if (gesture > 0 && !gesture_type.empty()) {
          // plot gesture type
          std::string gesture_direction_str =
                  map_gesture_direction_.find(gesture_direction) ==
                          map_gesture_direction_.end() ?
                  "" : " " + map_gesture_direction_.at(gesture_direction);
          cv::putText(bgr, gesture_type + gesture_direction_str,
                      cv::Point2f(hand_rect.x1, hand_rect.y1 - 15),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                      3, CV_RGB(255, 255, 255), 3);
        }
      }

      if (has_gesture_move && contours.size() >= 2) {
        if (static_cast<int>(output_gesture_type::PalmMove) == gesture ||
                static_cast<int>(output_gesture_type::Pinch) == gesture ||
                static_cast<int>(output_gesture_type::PinchMove) == gesture) {
          // palm move or pinch move
          cv::putText(bgr, gesture_type,
                      cv::Point2f(hand_rect.x1, hand_rect.y1 - 15),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                      3, CV_RGB(255, 255, 255), 3);
          cv::arrowedLine(bgr,
                          contours[0],
                          contours[contours.size() - 1],
                          CV_RGB(255, 255, 255),
                          2, 8, 0, 0.3);
        } else if (static_cast<int>(
             output_gesture_type::IndexFingerRotateAntiClockwise) == gesture ||
             static_cast<int>(output_gesture_type::IndexFingerRotateClockwise)
             == gesture ||
             static_cast<int>(output_gesture_type::PinchRotateAntiClockwise)
             == gesture ||
             static_cast<int>(output_gesture_type::PinchRotateClockwise)
             == gesture) {
          // rotate
          if (has_rotate_res) {
            // plot move points
            for (size_t lmk_idx = 0; lmk_idx < contours.size();
                 lmk_idx++) {
              cv::circle(bgr, contours[lmk_idx],
                         1, CV_RGB(0, 0, 255), 1);
            }

            // plot start and end pts
            if (contours.size() >= 2) {
              cv::circle(bgr,
                         contours[0],
                         4, CV_RGB(0, 255, 0), 4);
              cv::circle(bgr, contours[contours.size() - 1],
                         4, CV_RGB(255, 0, 0), 4);
            }

            // plot gesture type
            cv::putText(bgr, gesture_type + "/" +
                             std::to_string(rotate_num),
                        cv::Point2f(hand_rect.x1, hand_rect.y1 - 15),
                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                        3, CV_RGB(255, 255, 255), 3);

            // plot angel
            cv::Point2f plot_pt(contours[contours.size() - 1].x,
                                contours[contours.size() - 1].y);
            cv::putText(bgr,
                        std::to_string(rotate_angel),
                        plot_pt,
                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                        4, CV_RGB(0, 255, 255), 3);

            if (has_fit_rotated) {
              // 画出拟合的椭圆
              ellipse(bgr, fit_rotated_rect, cv::Scalar(255, 255, 255),
                      1, CV_AA);
              cv::Point2f fit_center(fit_rotated_rect.center.x,
                                     fit_rotated_rect.center.y);
              cv::circle(bgr, fit_center,
                         4, CV_RGB(255, 255, 255), 4);
            }
          } else {
            contours.clear();
            LOGD << "fit_ratio:" << fit_ratio << "  fit_err:" << fit_err;
//            cv::putText(bgr,
//                        "invalid, ratio:" + std::to_string(fit_ratio) +
//                        " err:" + std::to_string(fit_err),
//                        cv::Point2f(hand_rect.x1, hand_rect.y1 - 15),
//                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
//                        2, CV_RGB(255, 255, 255), 2);
          }
        }
      }

      if (has_face && vot_draw_ctrl_.draw_face_box) {
        cv::rectangle(bgr, cv::Point(face_rect.x1, face_rect.y1),
                      cv::Point(face_rect.x2, face_rect.y2),
                      d_color.at("face"), 2);
      }

      if (s_data.face) {
        {
          HorizonVisionBBox rect;
          if (has_face && vot_draw_ctrl_.draw_face_box) {
            rect = s_data.face->face_rect;
//            cv::rectangle(bgr, cv::Point(rect.x1, rect.y1),
//                          cv::Point(rect.x2, rect.y2),
//                          d_color.at("face"), 2);
          } else {
            rect = s_data.face->head_rect;
          }
          // track id
          std::stringstream id_info;
          id_info << s_data.track_id;
          if (!ss_attr.str().empty()) {
            id_info << "_" << ss_attr.str();
          }
          {
            std::lock_guard<std::mutex> lk(cache_mtx_);
            auto itr = recog_res_cache_.find(s_data.track_id);
            if (itr != recog_res_cache_.end()) {
              std::string new_url{itr->second->img_uri_list};
              if (itr->second->img_uri_list.rfind("/") > 0) {
                new_url = itr->second->img_uri_list.substr(
                        itr->second->img_uri_list.rfind("/") + 1);
              }
              id_info  //  << "_" << itr->second->record_id
                      << "_" << std::setprecision(3) << itr->second->similar
                      << "_" << new_url;
            } else {
              LOGI << "track id " << s_data.track_id
                   << " has no recog res, cache size: "
                   << recog_res_cache_.size();
            }
          }
          if (vot_draw_ctrl_.draw_id) {
            cv::putText(bgr, id_info.str(),
                        cv::Point(rect.x1, rect.y1 - 5),
                        cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                        1.5, d_color.at("id"), 2);
          }
        }

        if (vot_draw_ctrl_.draw_head_box) {
          const auto& rect = s_data.face->head_rect;
          cv::rectangle(bgr,
                        cv::Point(rect.x1, rect.y1),
                        cv::Point(rect.x2, rect.y2),
                        d_color.at("head"), 2);
        }

        // face lmk
        if (vot_draw_ctrl_.draw_head_box) {
          if (s_data.face->landmarks) {
            for (size_t lmk_idx = 0; lmk_idx < s_data.face->landmarks->num;
                 lmk_idx++) {
              cv::circle(bgr,
                         cv::Point(s_data.face->landmarks->points[lmk_idx].x,
                                   s_data.face->landmarks->points[lmk_idx].y),
                         1, d_color.at("lmk"), 2);
            }
          }
        }
      }

      if (s_data.body && vot_draw_ctrl_.draw_body_box) {
        const auto& rect = s_data.body->body_rect;
        cv::rectangle(bgr, cv::Point(rect.x1, rect.y1),
                      cv::Point(rect.x2, rect.y2),
                      d_color.at("body"), 2);
      }
    }
  } else {
    LOGE << "parse fail";
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> cost = end - start;
  LOGD << "draw jpeg cost ms " << cost.count();
  start = std::chrono::high_resolution_clock::now();

  char *buf_ = new char[image_data_size_];
  if (!plot_color_) {
    memcpy(buf_, bgr.ptr<uint8_t>(), image_data_size_);
  } else {
    bgr_to_nv12(bgr.ptr<uint8_t>(), buf_);
  }
  {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> cost = end - start;
    LOGD << "cvt to nv12 cost ms " << cost.count();
    start = std::chrono::high_resolution_clock::now();
  }

  std::shared_ptr<char> sps(buf_,
                           [](char* buf) {
                               delete []buf;
                           });
  return sps;
}  // NOLINT

int VotModule::Input(const std::shared_ptr<VotData_t>& vot_data) {
//  HOBOT_CHECK(fb);
//  memset(fb, 0x0, 1920 * 1080 * 4);
//  return 0;

  if (stop_) return 0;

  HOBOT_CHECK(vot_data);
  if (vot_data->sp_recog_) {
    // cache recog res to display
    if (vot_data->sp_recog_->is_recognize) {
      std::lock_guard<std::mutex> lk(cache_mtx_);
      recog_res_cache_[vot_data->sp_recog_->track_id] = vot_data->sp_recog_;
    }
  }
  if (!vot_data->sp_img_) {
    return 0;
  }

  if (en_encoder_ && 1 == encoder_input_ && vot_data && vot_data->sp_img_) {
    memcpy(venc_param_.mmz_vaddr,
           vot_data->sp_img_->data_,
           vot_data->sp_img_->DataSize());
    int ret = HB_VENC_SendFrame(venc_param_.veChn, &pstFrame, 20);
    if (ret != 0) {
      LOGE << "HB_VENC_SendStream Failed. ret = " << ret;
    }
  }

  if (in_queue_.size() < in_queue_len_max_) {
    in_queue_.push(std::move(vot_data));
  } else {
    LOGE << "vot queue is full, drop frame_id " << vot_data->sp_img_->frame_id_;
  }

  return 0;
}

int VotModule::Stop() {
  if (stop_)
    return 0;
  stop_ = true;
  if (send_vot_task_ != nullptr) {
    send_vot_task_->join();
    send_vot_task_ = nullptr;
  }
  if (display_task_ != nullptr) {
    display_task_->join();
    display_task_ = nullptr;
  }
  for (const auto& task : plot_tasks_) {
    task->join();
  }
  recog_res_cache_.clear();
  std::unique_lock<std::mutex> lk(send_vot_mtx_);
  send_vot_cache_.clear();
  lk.unlock();
  in_queue_.clear();
  plot_tasks_.clear();

  if (recv_stream_task_) {
    recv_stream_task_->join();
    recv_stream_task_ = nullptr;
  }

  if (DeinitCodecManager() != 0) {
    LOGE << "deinit codec fail";
    return -1;
  }

  return 0;
}

int VotModule::DeInit() {
  int ret = 0;
  ret = HB_VOT_DisableChn(0, channel_);
  if (ret) printf("HB_VOT_DisableChn failed.\n");

  ret = HB_VOT_DisableVideoLayer(0);
  if (ret) printf("HB_VOT_DisableVideoLayer failed.\n");

  ret = HB_VOT_Disable(0);
  if (ret) printf("HB_VOT_Disable failed.\n");

  if (en_encoder_) {
    if (ofs_encoder_output_.good()) {
      ofs_encoder_output_.close();
    }

    if (venc_param_.mmz_vaddr) {
      ret = HB_SYS_Free(venc_param_.mmz_paddr,
                        reinterpret_cast<void *>(venc_param_.mmz_vaddr));
      if (ret != 0) {
        LOGE << "HB_SYS_Free error! already exit in vio plugin. ret: " << ret;
      }
      venc_param_.mmz_vaddr = nullptr;
    }
    if (venc_param_.vp_init_local) {
      ret = HB_VP_Exit();
      if (ret == 0) {
        LOGI << "vp exit ok!";
      } else {
        LOGE << "vp exit error! ret: " << ret;
      }
    }
  }

  return ret;
}

int VotModule::InitCodecManager() {
  if (!en_encoder_) {
    return 0;
  }
  MediaCodecManager &manager = MediaCodecManager::Get();
  auto rv = manager.ModuleInit();  // ModuleInit()内部保证可以重复初始化
  HOBOT_CHECK(rv == 0);

  PAYLOAD_TYPE_E format = venc_param_.format;
  int chn_ = venc_param_.veChn;
  int pic_width = venc_param_.width;
  int pic_height = venc_param_.height;
  int frame_buf_depth = venc_param_.frame_buf_depth;
  int is_cbr = venc_param_.is_cbr;
  int bitrate = venc_param_.bitrate;

  LOGI << "pic_width: " << pic_width << " pic_height: " << pic_height
       << " bitrate:" << bitrate << " format:" << format;

  rv = manager.EncodeChnInit(chn_, format, pic_width, pic_height,
                             frame_buf_depth, HB_PIXEL_FORMAT_NV12,
                             is_cbr, bitrate);
  HOBOT_CHECK(rv == 0);

  rv = manager.EncodeChnStart(chn_);
  HOBOT_CHECK(rv == 0);
  return 0;
}

int VotModule::DeinitCodecManager() {
  if (!en_encoder_) {
    return 0;
  }
  LOGI << "DeinitCodecManager";
  MediaCodecManager &manager = MediaCodecManager::Get();
  manager.EncodeChnStop(venc_param_.veChn);
  manager.EncodeChnDeInit(venc_param_.veChn);
  // manager.ModuleDeInit();
  return 0;
}
}   //  namespace xproto
