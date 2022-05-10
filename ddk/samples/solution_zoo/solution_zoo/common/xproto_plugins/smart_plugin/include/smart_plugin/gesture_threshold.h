/**
 * Copyright (c) 2020, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: xue.liang
 * @Mail: xue.liang@horizon.ai
 * @Date: 2020-12-15 20:38:52
 * @Version: v0.0.1
 * @Brief:
 * @Last Modified by: xue.liang
 * @Last Modified time: 2020-12-16 22:41:30
 */

#ifndef INCLUDE_GESTURE_THRESHOLD_H_
#define INCLUDE_GESTURE_THRESHOLD_H_

#include <string>
#include <iostream>

namespace xproto {

struct GestureThreshold {
  int residual_err_thr_ = 50;  // base on 1080p
  size_t fit_points_size_thr_ = 25;
  int fit_ratio_thr_ = 3;
  int rotate_start_angel_thr_ = 180;
  int rotate_loop_angel_dist_thr_ = 200;
  // if hand continuous has no gesture frame count exceeds
  // gesture_vanish_thr, clear cache
  uint64_t dynamic_gesture_vanish_thr = 50;
  uint64_t static_gesture_vanish_thr = 25;
  uint64_t static_dynamic_dist_thr = 5;
  uint64_t valid_palmmove_in_cache_thr = 50;
  int conflict_gesture_thr_ = 25;
  uint64_t valid_palm_for_palmpat_thr_ = 150;
  int gesture_mute_outside = 20;  // base on 1080p
  // if palm_move_dist_thr is less than 0, output palm gesture only
  int palm_move_dist_thr = 5;  // base on 1080p
  // start calculate gesture direction only when
  // the Euclidean distance between start and end point exceeds thr
  float gesture_direction_activate_thr = 0.005;
  // calculate gesture direction using last N points
  size_t gesture_direction_cal_N_pts = 15;
  // direction is valid when angle with horizontal/vertical is less than thr
  int gesture_direction_angle_thr = 30;
  // fit is valid when err is less than thr
  float gesture_direction_fit_err_thr = 0.05;
};

}  // namespace xproto

#endif  // INCLUDE_GESTURE_THRESHOLD_H_
