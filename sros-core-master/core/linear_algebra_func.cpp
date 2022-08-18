/**
 * @file linear_algebra_func.cpp
 *
 * @author pengjiali
 * @date 19-7-2.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "linear_algebra_func.h"
#include <glog/logging.h>
#include <boost/math/tools/minima.hpp>

namespace sros {
namespace core {

Location calculateLineDirectionVector(double sx, double sy, double ex, double ey) {
    Location p;
    p.x() = ex - sx;
    p.y() = ey - sy;

    double p_len = sqrt(pow(p.x(), 2) + pow(p.y(), 2));
    p.x() /= p_len;  // 得到单位向量
    p.y() /= p_len;
    return p;
}

double getTurnAngle(const Location &d1, const Location &d2) {
    return normalizeYaw(atan2(d2.y(), d2.x()) - atan2(d1.y(), d1.x()));
}

double calculateInnerAngleBetweenTwoLines(const NavigationPath<double> &p1, const NavigationPath<double> &p2) {
    const double x1 = p1.ex_ - p1.sx_;
    const double y1 = p1.ey_ - p1.sy_;
    const double x2 = p2.ex_ - p2.sx_;
    const double y2 = p2.ey_ - p2.sy_;
    double theta = acos((x1 * y1 + x2 * y2) / std::sqrt(std::pow(x1, 2) + std::pow(y1, 2)) *
                        std::sqrt(std::pow(x2, 2) + std::pow(y2, 2)));
    return theta;
}

double getDistanceToSegment(sros::core::Pose p, sros::core::Pose s, sros::core::Pose e) {
    // sp与se的内积
    double cross = (e.x() - s.x()) * (p.x() - s.x()) + (e.y() - s.y()) * (p.y() - s.y());
    if (cross <= 0) return std::sqrt(std::pow(p.x() - s.x(), 2) + std::pow(p.y() - s.y(), 2));

    // d2 = se长度
    double d2 = (e.x() - s.x()) * (e.x() - s.x()) + (e.y() - s.y()) * (e.y() - s.y());
    if (cross >= d2) return std::sqrt(std::pow(p.x() - e.x(), 2) + std::pow(p.y() - e.y(), 2));

    double r = cross / d2;
    double qx = s.x() + (e.x() - s.x()) * r;
    double qy = s.y() + (e.y() - s.y()) * r;
    return std::sqrt((std::pow(p.x() - qx, 2) + std::pow(p.y() - qy, 2)));
}

std::pair<double, double> getDistanceToBezier(double x, double y, double p0x, double p0y, double p1x, double p1y,
                                              double p2x, double p2y, double p3x, double p3y) {
    /* 解决思路：列出点到贝塞尔曲线距离参数方程d_t，然后在d_t的极小值和贝塞尔起点终点中选取
     * b3t = p_0 * power((1-t),3) + 3*p_1*t.*power((1-t),2) + 3*p_2*power(t,2).*(1-t) + p_3*power(t,3)
     **/

    // 要求的点在起点和终点的情况，这种情况比较多，减少运算量
    if (x == p0x && y == p0y) {
        return std::make_pair<double, double>(0.0, 0.0);
    } else if (x == p3x && y == p3y) {
        return std::make_pair<double, double>(1.0, 0.0);
    }

    std::pair<double, double> result{0.0, DBL_MAX};
    auto minFunc = [](const std::pair<double, double> &l, const std::pair<double, double> &r) {
        if (l.second < r.second) {
            return true;
        }
        return false;
    };

    using boost::math::tools::brent_find_minima;
    using namespace std;
    const int double_bits = std::numeric_limits<double>::digits;
    auto distanceFunc = [&](double const &t) {
        return pow(p0x * pow((1 - t), 3) + 3 * p1x * t * pow((1 - t), 2) + 3 * p2x * pow(t, 2) * (1 - t) +
                       p3x * pow(t, 3) - x,
                   2) +
               pow(p0y * pow((1 - t), 3) + 3 * p1y * t * pow((1 - t), 2) + 3 * p2y * pow(t, 2) * (1 - t) +
                       p3y * pow(t, 3) - y,
                   2);
    };
    // 极小值可能会存在多个
    double step = 1 / 1000.0;
    double min_t = 0.0;
    double max_t = 1.0;
    for (auto i = 0; i < 3; ++i) {
        std::pair<double, double> r = brent_find_minima(distanceFunc, min_t, max_t, double_bits);
        result = min(result, std::pair<double, double>(r.first, sqrt(r.second)), minFunc);
        min_t = r.first + step;
        if (min_t > max_t) {
            break;
        }
    }
    result = min(result, std::pair<double, double>(0.0, getTwoPointDistance(x, y, p0x, p0y)), minFunc);
    result = min(result, std::pair<double, double>(1.0, getTwoPointDistance(x, y, p3x, p3y)), minFunc);

    return result;
}

double getTwoPointDistance(double sx, double sy, double ex, double ey) {
    return sqrt(pow(sx - ex, 2) + pow(sy - ey, 2));
}

double getRotatePathAngle(Location p1, Location p2) {
    Location p;
    p.x() = p2.x() - p1.x();
    p.y() = p2.y() - p1.y();

    double tmp_len = sqrt(pow(p.x(), 2) + pow(p.y(), 2));
    p.x() /= tmp_len;  // 得到单位向量
    p.y() /= tmp_len;

    double k_theta = acos(p.x());

    if (p.y() < 0) {
        k_theta = 2 * M_PI - k_theta;
    }
    return k_theta;
}
}  // namespace core
}  // namespace sros
