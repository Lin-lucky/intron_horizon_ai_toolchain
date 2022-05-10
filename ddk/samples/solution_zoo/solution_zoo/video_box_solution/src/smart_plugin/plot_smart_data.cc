/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author:
 * @Mail: @horizon.ai
 */

#include "plot_smart_data.h"

#include "hobotlog/hobotlog.hpp"
#include "libyuv/convert.h"
#include "opencv2/opencv.hpp"

namespace solution {
namespace video_box {
PlotSmartData::PlotSmartData() {}

PlotSmartData::~PlotSmartData() {}

int PlotSmartData::PlotData(const smart_vo_cfg_t vo_confg,
                            std::shared_ptr<VideoData> video_data,
                            const bool face, const bool head, const bool body,
                            const bool kps, const bool veh) {
  uint32_t width_tmp = video_data->width;
  uint32_t height_tmp = video_data->height;
  auto image_data_size_ = width_tmp * height_tmp * 3;
  uint8_t *bgr_buf = new uint8_t[image_data_size_ / 2 + image_data_size_ * 2];
  cv::Mat bgr(height_tmp, width_tmp, CV_8UC3, bgr_buf);
  cv::cvtColor(
      cv::Mat(height_tmp * 3 / 2, width_tmp, CV_8UC1, video_data->buffer), bgr,
      CV_YUV2BGR_NV12);

  int x_offset = 0;
  int y_offset = 0;
  float x_scale = 0.5;
  float y_scale = 0.5;
  x_scale = static_cast<float>(video_data->width / 1920.0);
  y_scale = static_cast<float>(video_data->height / 1080.0);

  static const std::map<std::string, decltype(CV_RGB(255, 0, 0))>
      d_color =  // NOLINT
      {{"id", CV_RGB(255, 0, 0)},        {"face", CV_RGB(255, 128, 0)},
       {"head", CV_RGB(255, 128, 0)},    {"body", CV_RGB(255, 128, 0)},
       {"lmk", CV_RGB(0, 245, 255)},     {"kps", CV_RGB(0, 245, 255)},
       {"vehicle", CV_RGB(255, 128, 0)}, {"plate", CV_RGB(0, 255, 0)},
       {"fps", CV_RGB(0, 255, 0)}};

  static const std::map<std::string, int> d_thickness =  // NOLINT
      {{"id", 2},  {"face", 2},     {"head", 2},    {"body", 2},  {"lmk", 2},
       {"kps", 2}, {"kps_line", 2}, {"vehicle", 2}, {"plate", 1}, {"fps", 2}};

  auto plot_box = [](cv::Mat &bgr_, const HorizonVisionBBox &box, float score,
                     decltype(CV_RGB(255, 0, 0)) color, double x_scale0,
                     double y_scale0, int x0, int y0, int thickness = 1) {
    if (box.score < score) return;
    cv::rectangle(bgr_,
                  cv::Point(box.x1 * x_scale0 + x0, box.y1 * y_scale0 + y0),
                  cv::Point(box.x2 * x_scale0 + x0, box.y2 * y_scale0 + y0),
                  color, thickness);
  };

  for (uint32_t i = 0; i < video_data->smart_frame->smart_data_list_num; ++i) {
    const auto &s_data = video_data->smart_frame->smart_data_list[i];
    if (s_data.body && body) {
#if 0
      if (s_data.body->skeleton) {
        auto points = s_data.body->skeleton->points;
        for (size_t kps_idx = 0; kps_idx < s_data.body->skeleton->num;
             kps_idx++) {
          cv::circle(bgr,
                     cv::Point(points[kps_idx].x * x_scale + x_offset,
                               points[kps_idx].y * y_scale + y_offset),
                     3, d_color.at("kps"), 3);
        }
        cv::line(bgr,
                 cv::Point(points[15].x * x_scale + x_offset,
                           points[15].y * y_scale + y_offset),
                 cv::Point(points[13].x * x_scale + x_offset,
                           points[13].y * y_scale + y_offset),
                 CV_RGB(255, 0, 0), 2);
        cv::line(bgr,
                 cv::Point(points[13].x * x_scale + x_offset,
                           points[13].y * y_scale + y_offset),
                 cv::Point(points[11].x * x_scale + x_offset,
                           points[11].y * y_scale + y_offset),
                 CV_RGB(255, 85, 0), 2);
        cv::line(bgr,
                 cv::Point(points[16].x * x_scale + x_offset,
                           points[16].y * y_scale + y_offset),
                 cv::Point(points[14].x * x_scale + x_offset,
                           points[14].y * y_scale + y_offset),
                 CV_RGB(255, 170, 0), 2);
        cv::line(bgr,
                 cv::Point(points[14].x * x_scale + x_offset,
                           points[14].y * y_scale + y_offset),
                 cv::Point(points[12].x * x_scale + x_offset,
                           points[12].y * y_scale + y_offset),
                 CV_RGB(255, 170, 0), 2);

        cv::line(bgr,
                 cv::Point(points[11].x * x_scale + x_offset,
                           points[11].y * y_scale + y_offset),
                 cv::Point(points[12].x * x_scale + x_offset,
                           points[12].y * y_scale + y_offset),
                 CV_RGB(170, 255, 0), 2);
        cv::line(bgr,
                 cv::Point(points[5].x * x_scale + x_offset,
                           points[5].y * y_scale + y_offset),
                 cv::Point(points[11].x * x_scale + x_offset,
                           points[11].y * y_scale + y_offset),
                 CV_RGB(85, 255, 0), 2);
        cv::line(bgr,
                 cv::Point(points[6].x * x_scale + x_offset,
                           points[6].y * y_scale + y_offset),
                 cv::Point(points[12].x * x_scale + x_offset,
                           points[12].y * y_scale + y_offset),
                 CV_RGB(0, 255, 0), 2);
        cv::line(bgr,
                 cv::Point(points[5].x * x_scale + x_offset,
                           points[5].y * y_scale + y_offset),
                 cv::Point(points[6].x * x_scale + x_offset,
                           points[6].y * y_scale + y_offset),
                 CV_RGB(0, 255, 85), 2);
        cv::line(bgr,
                 cv::Point(points[5].x * x_scale + x_offset,
                           points[5].y * y_scale + y_offset),
                 cv::Point(points[7].x * x_scale + x_offset,
                           points[7].y * y_scale + y_offset),
                 CV_RGB(0, 255, 170), 2);
        cv::line(bgr,
                 cv::Point(points[6].x * x_scale + x_offset,
                           points[6].y * y_scale + y_offset),
                 cv::Point(points[8].x * x_scale + x_offset,
                           points[8].y * y_scale + y_offset),
                 CV_RGB(0, 255, 255), 2);
        cv::line(bgr,
                 cv::Point(points[7].x * x_scale + x_offset,
                           points[7].y * y_scale + y_offset),
                 cv::Point(points[9].x * x_scale + x_offset,
                           points[9].y * y_scale + y_offset),
                 CV_RGB(0, 170, 255), 2);
        cv::line(bgr,
                 cv::Point(points[8].x * x_scale + x_offset,
                           points[8].y * y_scale + y_offset),
                 cv::Point(points[10].x * x_scale + x_offset,
                           points[10].y * y_scale + y_offset),
                 CV_RGB(0, 85, 255), 2);
        cv::line(bgr,
                 cv::Point(points[1].x * x_scale + x_offset,
                           points[1].y * y_scale + y_offset),
                 cv::Point(points[2].x * x_scale + x_offset,
                           points[2].y * y_scale + y_offset),
                 CV_RGB(0, 0, 255), 2);
        cv::line(bgr,
                 cv::Point(points[0].x * x_scale + x_offset,
                           points[0].y * y_scale + y_offset),
                 cv::Point(points[1].x * x_scale + x_offset,
                           points[1].y * y_scale + y_offset),
                 CV_RGB(85, 0, 255), 2);
        cv::line(bgr,
                 cv::Point(points[0].x * x_scale + x_offset,
                           points[0].y * y_scale + y_offset),
                 cv::Point(points[2].x * x_scale + x_offset,
                           points[2].y * y_scale + y_offset),
                 CV_RGB(170, 0, 255), 2);
        cv::line(bgr,
                 cv::Point(points[1].x * x_scale + x_offset,
                           points[1].y * y_scale + y_offset),
                 cv::Point(points[3].x * x_scale + x_offset,
                           points[3].y * y_scale + y_offset),
                 CV_RGB(255, 0, 255), 2);
        cv::line(bgr,
                 cv::Point(points[2].x * x_scale + x_offset,
                           points[2].y * y_scale + y_offset),
                 cv::Point(points[4].x * x_scale + x_offset,
                           points[4].y * y_scale + y_offset),
                 CV_RGB(255, 0, 170), 2);
        cv::line(bgr,
                 cv::Point(points[3].x * x_scale + x_offset,
                           points[3].y * y_scale + y_offset),
                 cv::Point(points[5].x * x_scale + x_offset,
                           points[5].y * y_scale + y_offset),
                 CV_RGB(255, 0, 85), 2);
        cv::line(bgr,
                 cv::Point(points[4].x * x_scale + x_offset,
                           points[4].y * y_scale + y_offset),
                 cv::Point(points[6].x * x_scale + x_offset,
                           points[6].y * y_scale + y_offset),
                 CV_RGB(0, 0, 255), 2);
      }
#endif
    }
    if (s_data.face && face) {
      {
        const auto &rect = s_data.face->face_rect;
        //        if (128 * (i + 2) < 1080) {
        //          auto &&face_rect = bgr(cv::Rect(cv::Point(rect.x1, rect.y1),
        //          cv::Point(rect.x2, rect.y2)));
        //          cv::resize(face_rect, face_rect, cv::Size(128, 128));
        //          face_rect.copyTo(bgr(cv::Rect(0, 128 * (i + 1), 128, 128)));
        //        }
        plot_box(bgr, rect, vo_confg.box_face_thr, d_color.at("face"), x_scale,
                 y_scale, x_offset, y_offset, d_thickness.at("face"));
#if 0
        // track id
        cv::putText(bgr, std::to_string(s_data.track_id),
                    cv::Point(rect.x1 * x_scale + x_offset,
                              (rect.y1 - 5) * y_scale + y_offset),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 2, d_color.at("id"),
                    3);
#else
        // recog id
        std::unordered_map<uint64_t, std::shared_ptr<RecogResult> >::iterator
            iter = video_data->recog_cache.find(s_data.track_id);
        if (iter != video_data->recog_cache.end()) {
          auto cach = iter->second;
          LOGI << "recog success,ch:" << video_data->channel
               << ",track_id:" << s_data.track_id
               << ",record_id:" << cach->record_id
               << ",similar:" << cach->similar
               << ",image_url:" << cach->img_uri_list;
#if 1
          cv::putText(bgr, cach->record_id,
                      cv::Point(rect.x1 * x_scale + x_offset,
                                (rect.y1 - 5) * y_scale + y_offset),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1, d_color.at("id"),
                      1);
#endif
        } else {
          LOGI << "recog fail,ch:" << video_data->channel
               << ",track_id:" << s_data.track_id;
        }
#endif
      }

      {
        const auto &rect = s_data.face->head_rect;
        plot_box(bgr, rect, vo_confg.box_head_thr, d_color.at("head"), x_scale,
                 y_scale, x_offset, y_offset, d_thickness.at("head"));
      }

      // face lmk
      if (s_data.face->landmarks) {
        for (size_t lmk_idx = 0; lmk_idx < s_data.face->landmarks->num;
             lmk_idx++) {
          if (s_data.face->landmarks->points[lmk_idx].score >
              vo_confg.lmk_thr) {
            cv::circle(
                bgr,
                cv::Point(s_data.face->landmarks->points[lmk_idx].x * x_scale +
                              x_offset,
                          s_data.face->landmarks->points[lmk_idx].y * y_scale +
                              y_offset),
                1, d_color.at("lmk"), d_thickness.at("lmk"));
          }
        }
      }
    }
  }

  bgr_to_nv12(bgr.ptr<uint8_t>(), (uint8_t *)(video_data->buffer),  // NOLINT
              width_tmp,                                            // NOLINT
              height_tmp);                                          // NOLINT
  delete[] bgr_buf;
  video_data->nv12 = true;
  return 0;
}

void PlotSmartData::bgr_to_nv12(uint8_t *bgr, uint8_t *buf,
                                const uint32_t width, const uint32_t height) {
  int uv_height = height / 2;
  int uv_width = width / 2;
  int uv_size = uv_height * uv_width;
  uint8_t *uv_data = buf + (uv_size << 2);
  uint8_t *uv_data_store = bgr + (uv_size << 2) * 3;
  libyuv::RGB24ToI420(bgr, uv_width * 6, buf, uv_width * 2, uv_data_store,
                      uv_width, uv_data_store + uv_size, uv_width, uv_width * 2,
                      uv_height * 2);
  // copy uv data
  for (int i = 0; i < uv_size; ++i) {
    *(uv_data++) = *(uv_data_store + i);
    *(uv_data++) = *(uv_data_store + uv_size + i);
  }
}

}  // namespace video_box
}  // namespace solution
