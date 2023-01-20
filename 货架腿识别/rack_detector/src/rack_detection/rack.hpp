//
// Created by getup on 18-12-7.
//

#ifndef PROJECT_RACK_HPP
#define PROJECT_RACK_HPP

//#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include "rack_para.h"

namespace rack_detection {
class Rack {
 public:
    std::array<Eigen::Vector2f, 4> legs;  //货架腿坐标
    Eigen::Vector2f center;               //货架中心
    float direction;                      //方向
    bool is_full;                         //是否能完全看到4条腿
    RackInfo type;                        //货架类型
    bool is_inner;                        //是否是内圈
    float cost;                           //损失函数,cost越小则识别准确性越高

    //通过3腿求货架
    explicit Rack(const RackInfo &type_, const bool is_inner_, const Eigen::Vector2f &leg1, const Eigen::Vector2f &leg2,
                  const Eigen::Vector2f &leg3)
        : type(type_), is_inner(is_inner_) {
        legs[0] = leg1;
        legs[1] = leg2;
        legs[2] = leg3;
        center = (leg1 + leg2) * 0.5f;
        Eigen::Vector2f vec_w = leg2 - leg3;
        if (vec_w.dot(center) > 0) vec_w = -vec_w;
        Eigen::Vector2f vec_l = leg1 - leg3;
        if (vec_l.dot(center) > 0) vec_l = -vec_l;
        const float dir_w = std::atan2(vec_w.y(), vec_w.x());
        const float dir_l = std::atan2(vec_l.y(), vec_l.x());
        direction = dir_l;
        LOG(INFO) << "length dir:" << direction << "," << vec_l.x() << "," << vec_l.y();
        LOG(INFO) << "width dir:" << dir_w << "," << leg1.x() << "," << leg1.y() << ",2," << leg2.x() << "," << leg2.y()
                  << "," << leg3.x() << "," << leg3.y();
        legs[3] = center * 2.f - leg3;
        is_full = false;
        getCost();
    }

    //通过4腿求货架
    explicit Rack(const RackInfo &type_, const bool is_inner_, const Eigen::Vector2f &leg1, const Eigen::Vector2f &leg2,
                  const Eigen::Vector2f &leg3, const Eigen::Vector2f &leg4)
        : type(type_), is_inner(is_inner_) {
        legs[0] = leg1;
        legs[1] = leg2;
        legs[2] = leg3;
        legs[3] = leg4;
        center = (leg1 + leg2 + leg3 + leg4) * 0.25f;
        Eigen::Vector2f vec_w = leg2 - leg3;
        if (vec_w.dot(center) > 0) vec_w = -vec_w;
        Eigen::Vector2f vec_l = leg1 - leg3;
        if (vec_l.dot(center) > 0) vec_l = -vec_l;
        const float dir_w = std::atan2(vec_w.y(), vec_w.x());
        const float dir_l = std::atan2(vec_l.y(), vec_l.x());
        direction = dir_l;
        LOG(INFO) << "dir:" << direction;
        is_full = true;
        getCost();
    }

 private:
    void getCost() {
        float length = (legs[2] - legs[0]).norm();
        float width = (legs[2] - legs[1]).norm();
        float l = static_cast<float>(is_inner ? type.leg_groups.front().length : type.leg_groups.back().length);
        float w = static_cast<float>(is_inner ? type.leg_groups.front().width : type.leg_groups.back().width);
        cost = (length - l) * (length - l) + (width - w) * (width - w);
        if (!is_full) {
            Eigen::Vector2f norm = center.normalized();
            const float a = legs[0].dot(norm);
            const float b = legs[1].dot(norm);
            const float c = legs[2].dot(norm);
            const float min = std::min({a, b, c});
            const float d = legs[3].dot(norm);
            if (d < min) {
                cost += (d - min) * (d - min);
            }
        }
    }
    //    void DebugDisplay(visualization_msgs::Marker::_points_type &points,
    //                      visualization_msgs::Marker::_points_type &lines) const {
    //      geometry_msgs::Point point;
    //      point.z = 0;
    //      int size = is_full ? 4 : 3;
    //      for (int i = 0; i < size; i++) {
    //        point.x = legs[i].x();
    //        point.y = legs[i].y();
    //        points.push_back(point);
    //      }
    //      point.x = center.x();
    //      point.y = center.y();
    //      points.push_back(point);
    //      lines.push_back(point);
    //      const float length = type.leg_groups.front().length;
    //      const float width = type.leg_groups.front().width;
    //      point.x = center.x() + std::cos(direction) / 2.0 * width;
    //      point.y = center.y() + std::sin(direction) / 2.0 * width;
    //      lines.push_back(point);
    //      const Eigen::Vector2f dir1(std::cos(direction) / 2.0 * width, std::sin(direction) / 2.0 * width);
    //      const Eigen::Vector2f
    //              dir2(std::cos(direction + M_PI_2) / 2.0 * length, std::sin(direction + M_PI_2) / 2.0 * length);
    //      const auto A = center + dir1 + dir2;
    //      const auto B = center - dir1 + dir2;
    //      const auto C = center - dir1 - dir2;
    //      const auto D = center + dir1 - dir2;
    //      point.x = A.x();
    //      point.y = A.y();
    //      lines.push_back(point);
    //      point.x = B.x();
    //      point.y = B.y();
    //      lines.push_back(point);
    //      lines.push_back(point);
    //      point.x = C.x();
    //      point.y = C.y();
    //      lines.push_back(point);
    //      lines.push_back(point);
    //      point.x = D.x();
    //      point.y = D.y();
    //      lines.push_back(point);
    //      lines.push_back(point);
    //      point.x = A.x();
    //      point.y = A.y();
    //      lines.push_back(point);
    //    }
};
}  // namespace rack_detection

#endif  // PROJECT_RACK_HPP
