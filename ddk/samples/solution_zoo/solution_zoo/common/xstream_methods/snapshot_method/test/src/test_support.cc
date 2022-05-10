/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     dump helper
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2019.05.23
 */
#include "test_support.h"

#include <set>

#include "xstream/vision_type.h"

int WriteLog(const xstream::XStreamSnapshotInfoPtr &snapshot_info) {
  FILE *plog = nullptr;
  plog = fopen("snap_log.txt", "a+");
  fprintf(plog, "track_id: %d, frame_id: %li\n",
          snapshot_info->value->track_id_,
          snapshot_info->value->origin_image_frame_->frame_id_);
  fclose(plog);
  return 0;
}

static int SaveImg(const xstream::ImageFramePtr &img_ptr,
                   const std::string &path) {
  if (!img_ptr) {
    return 0;
  }
  auto width = img_ptr->Width();
  auto height = img_ptr->Height();
  auto snap_data = img_ptr->Data();
  cv::Mat bgrImg;
  switch (img_ptr->pixel_format_) {
    case xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatNone: {
      return -1;
    }
    case xstream::HorizonVisionPixelFormat::
        kHorizonVisionPixelFormatImageContainer: {
      return -1;
    }
    case xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawRGB: {
      auto rgbImg = cv::Mat(height, width, CV_8UC3, snap_data);
      cv::cvtColor(rgbImg, bgrImg, CV_RGB2BGR);
      break;
    }
    case xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR: {
      bgrImg = cv::Mat(height, width, CV_8UC3, snap_data);
      break;
    }
    case xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawNV12: {
      uint8_t *data = static_cast<uint8_t *>(std::calloc(
          img_ptr->DataSize() + img_ptr->DataUVSize(), sizeof(uint8_t)));
      memcpy(data, reinterpret_cast<uint8_t *>(img_ptr->Data()),
             img_ptr->DataSize());
      memcpy(data + img_ptr->DataSize(),
             reinterpret_cast<uint8_t *>(img_ptr->DataUV()),
             img_ptr->DataUVSize());
      auto nv12Img = cv::Mat(height * 3 / 2, width, CV_8UC1, data);
      cv::cvtColor(nv12Img, bgrImg, CV_YUV2BGR_NV12);
      free(data);
      break;
    }
    default:
      break;
  }
  cv::imwrite(path + ".png", bgrImg);
  return 0;
}

int DumpSnap(const xstream::XStreamSnapshotInfoPtr &snapshot_info,
             std::string dir) {
  char filename[50];
  snprintf(filename, sizeof(filename), "%s/FaceSnap%li_%d_%li.yuv", dir.c_str(),
           snapshot_info->value->origin_image_frame_->time_stamp_,
           snapshot_info->value->track_id_,
           snapshot_info->value->origin_image_frame_->frame_id_);
#ifdef DUMP_YUV
  FILE *pfile = nullptr;
  auto data_size = snapshot_info->value->snap->DataSize();
  void *snap_data = reinterpret_cast<void *> snapshot_info->value->snap->Data();
  if (snap_data && data_size) {
    pfile = fopen(filename, "w");
    if (!pfile) {
      std::cerr << "open file " << filename << " failed" << std::endl;
      return -1;
    }
    if (!fwrite(snap_data, data_size, 1, pfile)) {
      std::cout << "fwrite data to " << filename << " failed" << std::endl;
      fclose(pfile);
    }
    fclose(pfile);
  }
#else
  snprintf(filename, sizeof(filename), "%s/FaceSnap%li_%d_%li", dir.c_str(),
           snapshot_info->value->origin_image_frame_->time_stamp_,
           snapshot_info->value->track_id_,
           snapshot_info->value->origin_image_frame_->frame_id_);
  std::string s_filename(filename);
  SaveImg(snapshot_info->value->snap_, s_filename);
#endif
  return 0;
}

int ConstructInput(const std::string &smart_frame,
                   const std::string &video_path, xstream::InputDataPtr &input,
                   const std::string &img_format, bool filter) {
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::DataState;
  using xstream::InputData;
  using xstream::InputDataPtr;
  using xstream::RawDataImageFrame;

  unsigned id = 0, md_id = 0;
  float x0 = 0, x1 = 0, y0 = 0, y1 = 0;
  float pose = 0, pitch = 0, yaw = 0, roll = 0, quality = 0, over_quality = 0;
  int lmk0 = 0, lmk1 = 0, lmk2 = 0, lmk3 = 0, lmk4 = 0;
  uint64_t timestamp = 0, frame_id = 0;

  std::istringstream ss(smart_frame);
  ss >> frame_id >> timestamp;

  xstream::CVImageFramePtr cv_img_frame(new xstream::CVImageFrame());
  cv_img_frame->time_stamp_ = timestamp;
  cv_img_frame->frame_id_ = frame_id;

  if (!video_path.empty()) {
    if (img_format == "gray") {
#if 0
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawGRAY;
      unsigned frame_width = 1920, frame_height = 1080;
      auto data_size = frame_width * frame_height;
      auto *img_data = new uint8_t[data_size];
      if (!video_path.empty()) {
        FILE *pfile = nullptr;
        pfile = fopen(video_path.data(), "r");
        if (!pfile) {
          std::cout << "Open video failed" << std::endl;
          return -1;
        }
        if (!fread(img_data, data_size, 1, pfile)) {
          std::cout << "fread video.yuv error" << std::endl;
          fclose(pfile);
          return -1;
        }
        fclose(pfile);
      }
      auto &cv_img = cv_img_frame->img;
      cv_img = cv::Mat(frame_height, frame_width, CV_8UC1);
      memcpy(cv_img.data, img_data, data_size);
      delete[] img_data;
#endif
    } else if (img_format == "raw_bgr") {
      cv_img_frame->img_ = cv::imread(video_path);
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;
    } else if (img_format == "raw_rgb") {
      cv::Mat bgrImage = cv::imread(video_path);
      cv::cvtColor(bgrImage, cv_img_frame->img_, CV_BGR2RGB);
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawRGB;
    } else if (img_format == "yuv_i420") {
#if 0
      cv::Mat bgrImage = cv::imread(video_path);
      cv::cvtColor(bgrImage, cv_img_frame->img_, CV_BGR2YUV_I420);
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawI420;
#endif
    }
  }

  xstream::XStreamImageFramePtr img_frame(new xstream::XStreamImageFrame());
  img_frame->type_ = "ImageFrame";
  img_frame->name_ = "img_frame";
  img_frame->value = cv_img_frame;
  input->datas_.push_back(BaseDataPtr(img_frame));

  std::shared_ptr<BaseDataVector> box_list(new BaseDataVector());
  box_list->name_ = "box_list";
  input->datas_.push_back(BaseDataPtr(box_list));

  std::shared_ptr<BaseDataVector> select_score_list(new BaseDataVector());
  select_score_list->name_ = "select_score_list";
  input->datas_.push_back(BaseDataPtr(select_score_list));

  std::shared_ptr<BaseDataVector> disappeared_track_id_list(
      new BaseDataVector());
  disappeared_track_id_list->name_ = "disappeared_track_id_list";
  input->datas_.push_back(BaseDataPtr(disappeared_track_id_list));

  std::shared_ptr<BaseDataVector> userdata_list1(new BaseDataVector());
  userdata_list1->name_ = "userdata_list1";
  input->datas_.push_back(BaseDataPtr(userdata_list1));

  std::shared_ptr<BaseDataVector> userdata_list2(new BaseDataVector());
  userdata_list2->name_ = "userdata_list2";
  input->datas_.push_back(BaseDataPtr(userdata_list2));

  std::set<uint64_t> id_set;
  static std::map<unsigned, unsigned> track_map;

  while (ss >> id) {
    ss >> md_id >> x0 >> y0 >> x1 >> y1;
    ss >> pose;
    ss >> pitch >> yaw >> roll;
    ss >> quality >> over_quality;
    ss >> lmk0 >> lmk1 >> lmk2 >> lmk3 >> lmk4;

    std::shared_ptr<xstream::BBox> bbox(new xstream::BBox());
    std::shared_ptr<xstream::XStreamFloat> score(new xstream::XStreamFloat());
    std::shared_ptr<BaseData> data1(new xstream::BaseData);
    std::shared_ptr<BaseData> data2(new xstream::BaseData);

    if (md_id == 0) {  // is fase
      id_set.insert(id);
      track_map[id] = frame_id;

      data1->name_ =
          std::to_string(frame_id) + "_" + std::to_string(id) + "_UserData0";
      data2->name_ =
          std::to_string(frame_id) + "_" + std::to_string(id) + "_UserData1";

      float resp_width = x1 - x0;
      float resp_height = y1 - y0;
      float size = std::min(resp_width, resp_height);

      bbox->x1_ = x0;
      bbox->y1_ = y0;
      bbox->x2_ = x1;
      bbox->y2_ = y1;
      bbox->id_ = id;

      if (pose > 1000 && size > 40 && x0 > 10 && x1 < 1910 && y0 > 10 &&
          y1 < 1070) {
        bbox->state_ = DataState::VALID;
      } else if (filter) {
        bbox->state_ = DataState::FILTERED;
      }
      if (bbox->state_ == DataState::VALID) {
        //  grading for snaps
        unsigned size_min = 40, size_max = 200, size_inflexion = 80;
        unsigned pose_thr = 1000, pose_max = 2000;

        float frontalPos_weight_ = 0.3;
        float size_weight_ = 0.2;
        float blur_weight_ = 0.2;
        float lmk_weight_ = 0.3;

        float normalized_frontal_pos =
            (pose - pose_thr) / (pose_max - pose_thr);
        float normalized_size = 0;
        if (size < size_min) {
          normalized_size = 0;
        } else if (size > size_max) {
          normalized_size = 1;
        } else {
          if (size > size_inflexion)
            normalized_size =
                (size - size_inflexion) / (size_max - size_inflexion) * 0.5f +
                0.5f;
          else
            normalized_size =
                (size - size_min) / (size_inflexion - size_min) * 0.5f;
        }
        float normalized_blur = 0;
        if (quality < -200)
          normalized_blur = 1;
        else if (quality > 200)
          normalized_blur = 0;
        else
          normalized_blur = -(quality - 200) / 400;

        float normalized_lmk = 0;
        normalized_lmk += lmk0;
        normalized_lmk += lmk1;
        normalized_lmk += lmk2;
        normalized_lmk += lmk3;
        normalized_lmk += lmk4;
        normalized_lmk /= 75;
        if (normalized_lmk < 0) normalized_lmk = 0;
        if (normalized_lmk > 1) normalized_lmk = 1;

        float res = frontalPos_weight_ * normalized_frontal_pos +
                    size_weight_ * normalized_size +
                    blur_weight_ * normalized_blur +
                    lmk_weight_ * normalized_lmk;

        score->value = static_cast<int>(2000 * res);
      }
      box_list->datas_.push_back(bbox);
      select_score_list->datas_.push_back(score);
      userdata_list1->datas_.push_back(data1);
      userdata_list2->datas_.push_back(data2);
    } else {  // head
      if (track_map.find(id) != track_map.end()) {
        if (id_set.find(id) == id_set.end()) {
          track_map[id] = frame_id;
        }
      }
    }
  }

  auto iter = track_map.begin();
  while (iter != track_map.end()) {
    std::shared_ptr<xstream::XStreamUint32> track_id(
        new xstream::XStreamUint32());
    track_id->value = iter->first;
    if (frame_id - iter->second == 50) {
      track_id->state_ = DataState::DISAPPEARED;
      disappeared_track_id_list->datas_.push_back(track_id);
      iter = track_map.erase(iter);
    } else {
      iter++;
    }
  }
  return 0;
}

int ConstructInputInvalidUserdata(const std::string &smart_frame,
                                  const std::string &video_path,
                                  xstream::InputDataPtr &input,
                                  const std::string &img_format, bool filter) {
  using xstream::BaseData;
  using xstream::BaseDataPtr;
  using xstream::BaseDataVector;
  using xstream::DataState;
  using xstream::InputData;
  using xstream::InputDataPtr;
  using xstream::RawDataImageFrame;

  unsigned id = 0, md_id = 0;
  float x0 = 0, x1 = 0, y0 = 0, y1 = 0;
  float pose = 0, pitch = 0, yaw = 0, roll = 0, quality = 0, over_quality = 0;
  int lmk0 = 0, lmk1 = 0, lmk2 = 0, lmk3 = 0, lmk4 = 0;
  uint64_t timestamp = 0, frame_id = 0;

  std::istringstream ss(smart_frame);
  ss >> frame_id >> timestamp;

  xstream::CVImageFramePtr cv_img_frame(new xstream::CVImageFrame());
  cv_img_frame->time_stamp_ = timestamp;
  cv_img_frame->frame_id_ = frame_id;

  if (!video_path.empty()) {
    if (img_format == "gray") {
#if 0
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawGRAY;
      unsigned frame_width = 1920, frame_height = 1080;
      auto data_size = frame_width * frame_height;
      auto *img_data = new uint8_t[data_size];
      if (!video_path.empty()) {
        FILE *pfile = nullptr;
        pfile = fopen(video_path.data(), "r");
        if (!pfile) {
          std::cout << "Open video failed" << std::endl;
          return -1;
        }
        if (!fread(img_data, data_size, 1, pfile)) {
          std::cout << "fread video.yuv error" << std::endl;
          fclose(pfile);
          return -1;
        }
        fclose(pfile);
      }
      auto &cv_img = cv_img_frame->img;
      cv_img = cv::Mat(frame_height, frame_width, CV_8UC1);
      memcpy(cv_img.data, img_data, data_size);
      delete[] img_data;
#endif
    } else if (img_format == "raw_bgr") {
      cv_img_frame->img_ = cv::imread(video_path);
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawBGR;
    } else if (img_format == "raw_rgb") {
      cv::Mat bgrImage = cv::imread(video_path);
      cv::cvtColor(bgrImage, cv_img_frame->img_, CV_BGR2RGB);
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawRGB;
    } else if (img_format == "yuv_i420") {
#if 0
      cv::Mat bgrImage = cv::imread(video_path);
      cv::cvtColor(bgrImage, cv_img_frame->img_, CV_BGR2YUV_I420);
      cv_img_frame->pixel_format_ =
          xstream::HorizonVisionPixelFormat::kHorizonVisionPixelFormatRawI420;
#endif
    }
  }

  xstream::XStreamImageFramePtr img_frame(new xstream::XStreamImageFrame());
  img_frame->type_ = "ImageFrame";
  img_frame->name_ = "img_frame";
  img_frame->value = cv_img_frame;
  input->datas_.push_back(BaseDataPtr(img_frame));

  std::shared_ptr<BaseDataVector> box_list(new BaseDataVector());
  box_list->name_ = "box_list";
  input->datas_.push_back(BaseDataPtr(box_list));

  std::shared_ptr<BaseDataVector> select_score_list(new BaseDataVector());
  select_score_list->name_ = "select_score_list";
  input->datas_.push_back(BaseDataPtr(select_score_list));

  std::shared_ptr<BaseDataVector> disappeared_track_id_list(
      new BaseDataVector());
  disappeared_track_id_list->name_ = "disappeared_track_id_list";
  input->datas_.push_back(BaseDataPtr(disappeared_track_id_list));

  BaseDataPtr userdata_list1(new BaseData());
  userdata_list1->name_ = "userdata_list1";
  userdata_list1->state_ = xstream::DataState::INVALID;
  input->datas_.push_back(BaseDataPtr(userdata_list1));

  BaseDataPtr userdata_list2(new BaseData());
  userdata_list2->state_ = xstream::DataState::INVALID;
  userdata_list2->name_ = "userdata_list2";
  input->datas_.push_back(BaseDataPtr(userdata_list2));

  std::set<uint64_t> id_set;
  static std::map<unsigned, unsigned> track_map;

  while (ss >> id) {
    ss >> md_id >> x0 >> y0 >> x1 >> y1;
    ss >> pose;
    ss >> pitch >> yaw >> roll;
    ss >> quality >> over_quality;
    ss >> lmk0 >> lmk1 >> lmk2 >> lmk3 >> lmk4;

    std::shared_ptr<xstream::BBox> bbox(new xstream::BBox());
    std::shared_ptr<xstream::XStreamFloat> score(new xstream::XStreamFloat());

    if (md_id == 0) {  // is fase
      id_set.insert(id);
      track_map[id] = frame_id;

      float resp_width = x1 - x0;
      float resp_height = y1 - y0;
      float size = std::min(resp_width, resp_height);

      bbox->x1_ = x0;
      bbox->y1_ = y0;
      bbox->x2_ = x1;
      bbox->y2_ = y1;
      bbox->id_ = id;

      if (pose > 1000 && size > 40 && x0 > 10 && x1 < 1910 && y0 > 10 &&
          y1 < 1070) {
        bbox->state_ = DataState::VALID;
      } else if (filter) {
        bbox->state_ = DataState::FILTERED;
      }
      if (bbox->state_ == DataState::VALID) {
        //  grading for snaps
        unsigned size_min = 40, size_max = 200, size_inflexion = 80;
        unsigned pose_thr = 1000, pose_max = 2000;

        float frontalPos_weight_ = 0.3;
        float size_weight_ = 0.2;
        float blur_weight_ = 0.2;
        float lmk_weight_ = 0.3;

        float normalized_frontal_pos =
            (pose - pose_thr) / (pose_max - pose_thr);
        float normalized_size = 0;
        if (size < size_min) {
          normalized_size = 0;
        } else if (size > size_max) {
          normalized_size = 1;
        } else {
          if (size > size_inflexion)
            normalized_size =
                (size - size_inflexion) / (size_max - size_inflexion) * 0.5f +
                0.5f;
          else
            normalized_size =
                (size - size_min) / (size_inflexion - size_min) * 0.5f;
        }
        float normalized_blur = 0;
        if (quality < -200)
          normalized_blur = 1;
        else if (quality > 200)
          normalized_blur = 0;
        else
          normalized_blur = -(quality - 200) / 400;

        float normalized_lmk = 0;
        normalized_lmk += lmk0;
        normalized_lmk += lmk1;
        normalized_lmk += lmk2;
        normalized_lmk += lmk3;
        normalized_lmk += lmk4;
        normalized_lmk /= 75;
        if (normalized_lmk < 0) normalized_lmk = 0;
        if (normalized_lmk > 1) normalized_lmk = 1;

        float res = frontalPos_weight_ * normalized_frontal_pos +
                    size_weight_ * normalized_size +
                    blur_weight_ * normalized_blur +
                    lmk_weight_ * normalized_lmk;

        score->value = static_cast<int>(2000 * res);
      }
      box_list->datas_.push_back(bbox);
      select_score_list->datas_.push_back(score);
    } else {  // head
      if (track_map.find(id) != track_map.end()) {
        if (id_set.find(id) == id_set.end()) {
          track_map[id] = frame_id;
        }
      }
    }
  }

  auto iter = track_map.begin();
  while (iter != track_map.end()) {
    std::shared_ptr<xstream::XStreamUint32> track_id(
        new xstream::XStreamUint32());
    track_id->value = iter->first;
    if (frame_id - iter->second == 50) {
      track_id->state_ = DataState::DISAPPEARED;
      disappeared_track_id_list->datas_.push_back(track_id);
      iter = track_map.erase(iter);
    } else {
      iter++;
    }
  }
  return 0;
}
