/**
 * @file navigation_path
 *
 * @author pengjiali
 *
 * @date 20-3-5.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "navigation_path.h"
#include "core/util/utils.h"

namespace sros {
namespace core {

void pathDoubleToInt(const NavigationPath_vector &paths_double, NavigationPathi_vector &paths_int) {
    for (auto p : paths_double) {
        sros::core::NavigationPath<int> pp;

        pp.type_ = p.type_;

        pp.sx_ = (p.sx_ * 1000.0);
        pp.sy_ = (p.sy_ * 1000.0);
        pp.ex_ = (p.ex_ * 1000.0);
        pp.ey_ = (p.ey_ * 1000.0);
        pp.cx_ = (p.cx_ * 1000.0);
        pp.cy_ = (p.cy_ * 1000.0);
        pp.dx_ = (p.dx_ * 1000.0);
        pp.dy_ = (p.dy_ * 1000.0);

        pp.radius_ = (p.radius_ * 1000.0);
        pp.rotate_angle_ = (p.rotate_angle_ * 1000);
        pp.limit_v_ = static_cast<int>(p.limit_v_ * 1000);
        pp.limit_w_ = static_cast<int>(p.limit_w_ * 1000);
        pp.direction_ = p.direction_;

        pp.avoid_policy_ = OBSTACLE_AVOID_WAIT;

        pp.updateFacing();

        paths_int.push_back(pp);
    }
}

void pathIntToDouble(const NavigationPathi_vector& paths_int, NavigationPath_vector& paths_double) {
    for (auto p : paths_int) {
        sros::core::NavigationPath<double> pp;

        pp.type_ = p.type_;

        pp.sx_ = (p.sx_ / 1000.0);
        pp.sy_ = (p.sy_ / 1000.0);
        pp.ex_ = (p.ex_ / 1000.0);
        pp.ey_ = (p.ey_ / 1000.0);
        pp.cx_ = (p.cx_ / 1000.0);
        pp.cy_ = (p.cy_ / 1000.0);
        pp.dx_ = (p.dx_ / 1000.0);
        pp.dy_ = (p.dy_ / 1000.0);

        pp.radius_ = (p.radius_ / 1000.0);
        pp.rotate_angle_ = (p.rotate_angle_ / 1000);
        pp.direction_ = p.direction_;

        pp.avoid_policy_ = OBSTACLE_AVOID_WAIT;

        pp.updateFacing();

        paths_double.push_back(pp);
    }
}

LinePath makeLine(double sx, double sy, double ex, double ey, double limit_v, double limit_w, PathDirection direction) {
    NavigationPath<double> line_path(sx, sy, ex, ey, direction, 0, 0, limit_v, limit_w);
    line_path.type_ = PATH_LINE;

    double d_x = line_path.ex_ - line_path.sx_;
    double d_y = line_path.ey_ - line_path.sy_;
    double d = sqrt(pow(d_x, 2) + pow(d_y, 2));
    double theta = acos(d_x / d);
    if (d_y < 0) {
        theta = 2 * M_PI - theta;
    }
    if (direction == PATH_BACKWARD) {
        theta = M_PI + theta;
        if (theta > 2 * M_PI) {
            theta = theta - 2 * M_PI;
        }
    }
    line_path.s_facing_ = normalizeYaw(theta);
    line_path.e_facing_ = normalizeYaw(theta);

    return line_path;
}

LinePath makeLine(double sx, double sy, double ex, double ey, PathDirection direction) {
    return makeLine(sx, sy, ex, ey, 0, 0, direction);
}

RotatePath makeRotate(double rotate_angle) { return RotatePath(rotate_angle); }

BezierPath makeBezier(const sros::map::net::Edgef& edge) {
    return BezierPath(edge.sx / 100, edge.sy / 100, edge.cx / 100, edge.cy / 100, edge.dx / 100, edge.dy / 100,
                      edge.ex / 100, edge.ey / 100, edge.radius / 100, PATH_FORWARD, edge.s_facing, edge.e_facing,
                      edge.limit_v, edge.limit_w);
}
}  // namespace core
}  // namespace sros
