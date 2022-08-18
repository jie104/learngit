#ifndef POINT_TYPE_HPP_
#define POINT_TYPE_HPP_

#include <pcl/point_types.h>
#include <iostream>

namespace obstacle_detection {
//栅格化点
struct RasterizedPoint {
  float x, y, z;
  bool validity;    //有效性
  float intensity;  //强度
  int front_index;
  int back_index;
};
}  // namespace obstacle_detection
#endif