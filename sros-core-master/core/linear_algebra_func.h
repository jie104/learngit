/**
 * @file linear_algebra_func.h
 *
 * @author pengjiali
 * @date 19-7-2.
 *
 * @describe 将线性代数的一些方法提取到此处，供各个地方调用
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_LINEAR_ALGEBRA_FUNC_H_
#define CORE_LINEAR_ALGEBRA_FUNC_H_

#include "navigation_path.h"
#include "pose.h"

namespace sros {
namespace core {

/**
 * @brief 获取直线的单位向量
 * @param sx 起点 (m)
 * @param sy
 * @param ex 终点
 * @param ey
 * @return 直线对于的单位向量
 */
Location calculateLineDirectionVector(double sx, double sy, double ex, double ey);

/**
 * @brief
 * @param d1
 * @param d2
 * @return 返回以d2基准，d1旋转到d2，需要旋转的角度(逆时针为正) 单位：弧度[-PI, PI]
 */
double getTurnAngle(const Location &d1, const Location &d2);

/**
 * @brief 获取直线朝向
 * @tparam T T只能是浮现类型
 * @param sx
 * @param sy
 * @param ex
 * @param ey
 * @param is_back_edge
 * @return
 */
template <typename T>
double getLineFacing(T sx, T sy, T ex, T ey, bool is_back_edge) {
  T d_x = ex - sx;
  T d_y = ey - sy;
  T d = sqrt(pow(d_x, 2) + pow(d_y, 2));
  double theta = acos(d_x / d);
  if (d_y < 0) {
      theta = 2 * M_PI - theta;
  }
  if (is_back_edge) {
      theta = M_PI + theta;
      if (theta > 2 * M_PI) {
          theta = theta - 2 * M_PI;
      }
  }
  return theta;
};

/**
 * 获取点到线段的距离
 * @param pose
 * @param s
 * @param e
 * @return
 */
double getDistanceToSegment(Pose pose, Pose s, Pose e);

/**
 * 获取点到贝塞尔的最短距离
 * @param x 目标点
 * @param y
 * @param sx 贝塞尔曲线
 * @param sy
 * @param cx1
 * @param cy1
 * @param cx2
 * @param cy2
 * @param ex
 * @param ey
 * @return <t, d> 参数t，和距离. t∈[0.0, 1.0]
 */
std::pair<double, double> getDistanceToBezier(double x, double y, double p0x, double p0y, double p1x, double p1y, double p2x, double p2y,
                           double p3x, double p3y);

/**
 * 获取两点之间的距离
 * @param sx 起点 (m)
 * @param sy
 * @param ex
 * @param ey
 * @return
 */
double getTwoPointDistance(double sx, double sy, double ex, double ey);

/**
 * 根据两点坐标计算p1->p2向量与x轴正方向的夹角大小
 * 计算两条直线的夹角
 * @param p1
 * @param p2
 * @return
 */
double calculateInnerAngleBetweenTwoLines(const NavigationPath<double> &p1, const NavigationPath<double> &p2);

double getRotatePathAngle(Location p1, Location p2);

}  // namespace core
}  // namespace sros

#endif  // CORE_LINEAR_ALGEBRA_FUNC_H_
