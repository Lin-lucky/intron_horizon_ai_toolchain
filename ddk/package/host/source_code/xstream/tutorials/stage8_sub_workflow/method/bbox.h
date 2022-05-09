/**
 * Copyright (c) 2020 Horizon Robotics. All rights reserved.
 * @brief     bbox base data type only for example
 * @author    wenhao.zou
 * @email     wenhao.zou@horizon.ai
 * @version   0.0.0.1
 * @date      2020.02.08
 */

#ifndef XSTREAM_TUTORIALS_STAGE8_METHOD_BBOX_H_
#define XSTREAM_TUTORIALS_STAGE8_METHOD_BBOX_H_

#include <memory>
#include <string>

#include "xstream/xstream_world.h"

namespace xstream {

struct BBox : public BaseData {
  inline BBox() {}
  inline BBox(float x1_, float y1_, float x2_, float y2_, float score_ = 0) {
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

  inline friend std::ostream &operator<<(std::ostream &out, const BBox &bbox) {
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

struct BBoxClassifyResult : public BaseData {
  BBoxClassifyResult() {}
  enum type { SHAPE_UNKNOW = -1, SHAPE_RECT = 0, SHAPE_SQUARE };
  type result = SHAPE_UNKNOW;  // 0: rectangle, 1: square

  inline friend std::ostream &operator<<(std::ostream &out,
                                         BBoxClassifyResult &result) {
    if (result.result == SHAPE_RECT) {
      out << "rectangle";
    } else if (result.result == SHAPE_SQUARE) {
      out << "square";
    } else {
      out << "unknow";
    }
    return out;
  }

  inline friend std::ostream &operator<<(std::ostream &out,
                                         const BBoxClassifyResult &result) {
    if (result.result == SHAPE_RECT) {
      out << "rectangle";
    } else if (result.result == SHAPE_SQUARE) {
      out << "square";
    } else {
      out << "unknow";
    }
    return out;
  }
};
typedef std::shared_ptr<BBoxClassifyResult> BBoxClassifyResultPtr;
}  // namespace xstream

#endif  // XSTREAM_TUTORIALS_STAGE4_METHOD_BBOX_H_
