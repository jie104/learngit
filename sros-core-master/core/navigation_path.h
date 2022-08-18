//
// Created by lhx on 16-1-20.
//

#ifndef SROS_NAVIGATION_NAVIGATION_PATH_H
#define SROS_NAVIGATION_NAVIGATION_PATH_H

#include <cmath>
#include <vector>
#include "core/map/net/edge.hpp"
#include "map_manager.h"
#include "state.h"

namespace sros {
namespace core {

// 需要与scr_protocol.h中数值定义相同
enum PathType {
    PATH_LINE = 0x01,
    PATH_ARC = 0x02,
    PATH_BEZIER = 0x03,
    PATH_ROTATE = 0x04,
};

enum PathDirection {
    PATH_FORWARD = 0x01,
    PATH_BACKWARD = 0x02,
    PATH_LEFT= 0x03,
    PATH_RIGHT = 0x04,
};

/**
 * @brief
 * @note 所有的成员变量都是只读的，外界不允许修改，为了防止赋值参数漏掉，所有的构造必须用静态函数来构造
 * @tparam T
 */
template <typename T>
class NavigationPath {
 public:
    T sx() { return sx_; }
    T sy() { return sy_; }
    T cx() { return cx_; }
    T cy() { return cy_; }
    T dx() { return dx_; }
    T dy() { return dy_; }
    T ex() { return ex_; }
    T ey() { return ey_; }

    int direction() { return direction_; }

 public:
    NavigationPath() = default;

    explicit NavigationPath(T sx, T sy, double rotate_angle)
        : NavigationPath(sx, sy, rotate_angle, PATH_FORWARD, 0.0) {}

    explicit NavigationPath(T sx, T sy, double rotate_angle, PathDirection d, double limit_w)
        : type_(PATH_ROTATE),
          avoid_policy_(OBSTACLE_AVOID_WAIT),
          sx_(sx),
          sy_(sy),
          cx_(0),
          cy_(0),
          dx_(0),
          dy_(0),
          ex_(0),
          ey_(0),
          rotate_angle_(rotate_angle),
          radius_(0),
          limit_w_(limit_w),
          direction_(d){

          };

    explicit NavigationPath(double rotate_angle) : NavigationPath(rotate_angle, PATH_FORWARD) {}

    NavigationPath(double rotate_angle, PathDirection d)
        : type_(PATH_ROTATE),
          avoid_policy_(OBSTACLE_AVOID_WAIT),
          sx_(0),
          sy_(0),
          cx_(0),
          cy_(0),
          dx_(0),
          dy_(0),
          ex_(0),
          ey_(0),
          rotate_angle_(rotate_angle),
          radius_(0),
          limit_w_(0),
          direction_(d){

          };

    NavigationPath(T sx, T sy, T ex, T ey) : NavigationPath(sx, sy, ex, ey, PATH_FORWARD) {}

    NavigationPath(T sx, T sy, T ex, T ey, PathDirection d) : NavigationPath(sx, sy, ex, ey, d, 0.0, 0.0, 0.0, 0.0){};

    NavigationPath(T sx, T sy, T ex, T ey, double s_facing, double e_facing, double limit_v, double limit_w)
        : NavigationPath(sx, sy, ex, ey, PATH_FORWARD, s_facing, e_facing, limit_v, limit_w) {}

    NavigationPath(T sx, T sy, T ex, T ey, PathDirection d, double s_facing, double e_facing, double limit_v,
                   double limit_w)
        : type_(PATH_LINE),
          avoid_policy_(OBSTACLE_AVOID_WAIT),
          sx_(sx),
          sy_(sy),
          cx_(0),
          cy_(0),
          dx_(0),
          dy_(0),
          ex_(ex),
          ey_(ey),
          rotate_angle_(0),
          radius_(100.0),
          s_facing_(s_facing),
          e_facing_(e_facing),
          limit_v_(limit_v),
          limit_w_(limit_w),
          direction_(d) {}

    // cw:clockwise是否为顺时针,若为顺圆则半径为负，否则为正
    NavigationPath(T sx, T sy, T ex, T ey, T cx, T cy, bool cw)
        : NavigationPath(sx, sy, ex, ey, cx, cy, cw, PATH_FORWARD) {}

    NavigationPath(T sx, T sy, T ex, T ey, T cx, T cy, bool cw, PathDirection d)
        : type_(PATH_ARC),
          avoid_policy_(OBSTACLE_AVOID_WAIT),
          sx_(sx),
          sy_(sy),
          ex_(ex),
          ey_(ey),
          cx_(cx),
          cy_(cy),
          dx_(0),
          dy_(0),
          rotate_angle_(0),
          direction_(d) {
        if (cw)
            radius_ = -1 * (std::sqrt(std::pow(ex - cx, 2) + std::pow(ey - cy, 2)));
        else
            radius_ = std::sqrt(std::pow(ex - cx, 2) + std::pow(ey - cy, 2));
    };

    NavigationPath(T sx, T sy, T ex, T ey, T cx, T cy, double radius, double s_facing, double e_facing, double limit_v,
                   double limit_w)
        : NavigationPath(sx, sy, ex, ey, cx, cy, radius, PATH_FORWARD, s_facing, e_facing, limit_v, limit_w) {}

    // TODO: 需要区分顺圆与逆圆(根据半径的正负确定)
    NavigationPath(T sx, T sy, T ex, T ey, T cx, T cy, double radius, PathDirection d, double s_facing, double e_facing,
                   double limit_v, double limit_w)
        : type_(PATH_ARC),
          avoid_policy_(OBSTACLE_AVOID_WAIT),
          sx_(sx),
          sy_(sy),
          ex_(ex),
          ey_(ey),
          cx_(cx),
          cy_(cy),
          dx_(0),
          dy_(0),
          rotate_angle_(0),
          radius_(radius),
          direction_(d),
          s_facing_(s_facing),
          e_facing_(e_facing),
          limit_v_(limit_v),
          limit_w_(limit_w){

          };

    // Bezier
    NavigationPath(T sx, T sy, T cx, T cy, T dx, T dy, T ex, T ey, double radius)
        : NavigationPath(sx, sy, cx, cy, dx, dy, ex, ey, radius, PATH_FORWARD, 0, 0, 0, 0) {}

    NavigationPath(T sx, T sy, T cx, T cy, T dx, T dy, T ex, T ey, double radius, PathDirection d)
        : NavigationPath(sx, sy, cx, cy, dx, dy, ex, ey, radius, d, 0, 0, 0, 0) {}

    NavigationPath(T sx, T sy, T cx, T cy, T dx, T dy, T ex, T ey, double radius, PathDirection d, double s_facing,
                   double e_facing, double limit_v, double limit_w)
        : type_(PATH_BEZIER),
          avoid_policy_(OBSTACLE_AVOID_WAIT),
          sx_(sx),
          sy_(sy),
          cx_(cx),
          cy_(cy),
          dx_(dx),
          dy_(dy),
          ex_(ex),
          ey_(ey),
          s_facing_(s_facing),
          e_facing_(e_facing),
          limit_v_(limit_v),
          limit_w_(limit_w),
          rotate_angle_(0),
          radius_(radius),
          direction_(d){

          };

    friend std::ostream &operator<<(std::ostream &out, const NavigationPath<T> &path) {
        std::string path_direction;
        switch (path.direction_) {
            case PATH_FORWARD:
                path_direction = "FORWARD";
                break;
            case PATH_BACKWARD:
                path_direction = "BACKWARD";
                break;
            case PATH_LEFT:
                path_direction = "LEFT";
                break;
            case PATH_RIGHT:
                path_direction = "RIGHT";
                break;
            default:
                path_direction = std::to_string(path.direction_);
                break;
        }
        std::string path_type;
        switch (path.type_) {
            case PATH_LINE: {
                path_type = "PATH_LINE";
                out << path_type << " s(" << path.sx_ << "," << path.sy_ << ") e(" << path.ex_ << "," << path.ey_
                    << ") " << path_direction << " s_f:" << path.s_facing_ << " e_f:" << path.e_facing_;
                break;
            }
            case PATH_ARC:
                path_type = "PATH_ARC";
                out << path_type << " s(" << path.sx_ << "," << path.sy_ << ") e(" << path.ex_ << "," << path.ey_
                    << ") radius:" << path.radius_ << " " << path_direction << " s_f:" << path.s_facing_
                    << " e_f:" << path.e_facing_;
                break;
            case PATH_BEZIER:
                path_type = "PATH_BEZIER";
                out << path_type << " s(" << path.sx_ << "," << path.sy_ << ") c(" << path.cx_ << "," << path.cy_
                    << ") d(" << path.dx_ << "," << path.dy_ << ") e(" << path.ex_ << "," << path.ey_ << ") "
                    << path_direction << " s_f:" << path.s_facing_ << " e_f:" << path.e_facing_;
                break;
            case PATH_ROTATE:
                path_type = "PATH_ROTATE";
                out << path_type << " angle:" << path.rotate_angle_;
                break;
            default:
                break;
        }
        return out;
    }

    void updateFacing() {
        auto getLineFacingFunc = [](T s_x, T s_y, T e_x, T e_y, int direction) {
          T d_x = e_x - s_x;
          T d_y = e_y - s_y;
          T d = sqrt(pow(d_x, 2) + pow(d_y, 2));
          double theta = acos(d_x / d);
          if (d_y < 0) {
              theta = 2 * M_PI - theta;
          }
          switch (direction) {
              case PATH_FORWARD: {
                  break;
              }
              case PATH_BACKWARD: {
                  theta += M_PI;
                  break;
              }
              case PATH_LEFT: {
                  theta += M_PI_2;
                  break;
              }
              case PATH_RIGHT: {
                  theta -= M_PI_2;
                  break;
              }
              default: {
                  break;
              }
          }
          theta = std::abs(fmod(theta + 2.0 * M_PI, 2.0 * M_PI));
          return theta;
        };

        switch (type_) {
            case PATH_LINE: {
                s_facing_ = getLineFacingFunc(sx_, sy_, ex_, ey_, direction_);
                e_facing_ = s_facing_;
                break;
            }
            case PATH_BEZIER: {
                s_facing_ = getLineFacingFunc(sx_, sy_, cx_, cy_, direction_);
                e_facing_ = getLineFacingFunc(dx_, dy_, ex_, ey_, direction_);
                break;
            }
            case PATH_ARC: {
                s_facing_ = getLineFacingFunc(sx_, sy_, cx_, cy_, direction_) + (radius_ > 0 ? -M_PI_2 : M_PI_2);
                e_facing_ = s_facing_ + M_PI;
            }
            default: {
                break;
            }
        }
    }

    PathType type_;

    T sx_, sy_;  // 起点

    T ex_, ey_;  // 终点

    T cx_, cy_;  // 圆心/控制点1

    T dx_, dy_;  // 控制点2

    double radius_;  // 圆弧路径中的半径大小/曲率半径大小

    int direction_;  // 运动方向

    double rotate_angle_;

    // 插入旋转时直接用facing很方便
    double s_facing_;  // 起点朝向
    double e_facing_;  // 终点朝向

    double limit_v_;  // 限制线速度
    double limit_w_;  // 限制角速度

    ObstacleAvoidPolicy avoid_policy_ = OBSTACLE_AVOID_WAIT;  // 避障策略

    void setPathDirection(PathDirection d) { direction_ = d; };
};

typedef NavigationPath<double> LinePath;
typedef NavigationPath<double> RotatePath;
typedef NavigationPath<double> CirclePath;
typedef NavigationPath<double> BezierPath;

typedef NavigationPath<int> LinePathi;
typedef NavigationPath<int> RotatePathi;
typedef NavigationPath<int> CirclePathi;
typedef NavigationPath<int> BezierPathi;
typedef NavigationPath<int> NavigationPathi;

typedef std::vector<NavigationPath<double>> NavigationPath_vector;
typedef std::vector<NavigationPath<int>> NavigationPathi_vector;

LinePath makeLine( double sx,  double sy,  double ex,  double ey, double limit_v, double limit_w,
                           PathDirection direction = PATH_FORWARD);

LinePath makeLine( double sx,  double sy,  double ex,  double ey, PathDirection direction = PATH_FORWARD);

RotatePath makeRotate(double rotate_angle);

BezierPath makeBezier(const sros::map::net::Edgef &edge);

/**
 * 转换NavigationPathi_vector为NavigationPath_vector
 * @param paths_int
 * @return
 */
void pathIntToDouble(const NavigationPathi_vector &paths_int, NavigationPath_vector &paths_double);

void pathDoubleToInt(const NavigationPath_vector &paths_double, NavigationPathi_vector &paths_int);

}  // namespace core
}  // namespace sros

#endif  // SROS_NAVIGATION_NAVIGATION_PATH_H
