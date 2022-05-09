/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     bbox base data type only for example
 * @author    zhe.sun
 * @version   0.0.0.1
 * @date      2020.10.30
 */

#ifndef XSTREAM_FRAMEWORK_TUTORIALS_STAGE1_METHOD_BBOX_H_
#define XSTREAM_FRAMEWORK_TUTORIALS_STAGE1_METHOD_BBOX_H_

#include <string>
#include <memory>
#include "xstream/xstream_world.h"

namespace xstream {

struct BBox : public BaseData {
  inline BBox() {}
  inline BBox(float x1_, float y1_, float x2_, float y2_,
              float score_ = 0) {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    score = score_;
  }
  inline float Width() const { return (x2 - x1); }
  inline float Height() const { return (y2 - y1); }
  inline float CenterX() const { return (x1 + (x2 - x1) / 2); }
  inline float CenterY() const { return (y1 + (y2 - y1) / 2); }

  inline friend std::ostream &operator<<(std::ostream &out, BBox &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }

  float x1 = 0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  float score = 0;
};

typedef std::shared_ptr<BBox> BBoxPtr;

}  // namespace xstream

#endif  // XSTREAM_FRAMEWORK_TUTORIALS_STAGE1_METHOD_BBOX_H_
