//
// Created by getup on 18-12-3.
//

#ifndef PROJECT_RACK_DETECTER_HPP
#define PROJECT_RACK_DETECTER_HPP

//#include "visualization_msgs/Marker.h"
#include <map>
#include <unordered_map>
#include <unordered_set>
#include "kd_tree/kd_tree_2d.hpp"
#include "rack.hpp"
#include "rack_leg_detecter.hpp"
#include "rack_para.h"

namespace rack_detection {
enum class EdgeType {
    LENGTH = 0,
    WIDTH = 1,
};

template <class LaserScan>
class RackDetecter {
 public:
    RackDetecter() = default;

    RackDetecter(const RackDetecter &rhs) = delete;

    RackDetecter &operator=(const RackDetecter &rhs) = delete;

 private:
    static constexpr float valid_leg_range_ = 5.f;    //只识别雷达5m范围内的货架腿
    static constexpr float tolerant_length_ = 0.06f;  //长度冗余阈值
    static constexpr float tolerant_angle_ = 0.06f;   //角度冗余阈值
    static constexpr float center_threshold_ = 0.3f;  //货架中心阈值
    RackLegDetecter<LaserScan> rack_leg_detecter_;
    std::set<RackInfo> rack_types_;
    std::vector<Eigen::Vector2f> rack_legs_;
    std::unordered_map<int, std::vector<std::pair<int, EdgeType>>> edges_;
    std::map<std::pair<int, int>, std::unordered_set<int>> triangles_;
    std::vector<std::tuple<int, int, int, int>> rectangles_;
    std::vector<Rack> racks_;

 public:
    //添加货架类型
    void addRackInfo(const RackInfo &info) { rack_types_.insert(info); }

    //获取识别货架(类型以及对应的位姿)
    void detectRack(std::shared_ptr<LaserScan> scan_ptr, std::vector<RackInfo> &rack_infos,
                    std::vector<Eigen::Vector3d> &center_pose) {
        racks_.clear();
        rack_leg_detecter_.Set(scan_ptr);
        for (const auto &rack_type : rack_types_) {
            getRackLegs(rack_type.leg_d);
            getRacks(rack_type);
        }
        for (const auto &rack : racks_) {
            LOG(INFO) << rack.type.leg_d << ", " << rack.center.x() << ", " << rack.center.y() << ", "
                      << rack.direction;
            rack_infos.push_back(rack.type);
            center_pose.emplace_back(double(rack.center.x()), double(rack.center.y()), double(rack.direction));
        }
    }

    //清空已添加的货架类型
    void clearRackTypes() { rack_types_.clear(); }

    //    template<class RosLaserScan>
    //    void debugDisplay(RosLaserScan &scan,
    //                      visualization_msgs::Marker::_points_type &plane_frame,
    //                      visualization_msgs::Marker::_points_type &plane_map,
    //                      visualization_msgs::Marker::_points_type &corner_frame,
    //                      visualization_msgs::Marker::_points_type &corner_map) const {
    //      rack_leg_detecter_.DebugDisplay(scan);
    //      geometry_msgs::Point point;
    //      point.z = -0.2;
    //      for (const auto &leg: rack_legs_) {
    //        point.x = leg.x();
    //        point.y = leg.y();
    //        corner_map.push_back(point);
    //      }
    //      for (const auto &rack : racks_) {
    //        rack.DebugDisplay(corner_frame, plane_frame);
    //      }
    //    }

 private:
    void getRackLegs(const float leg_d) {
        rack_leg_detecter_.GetRackLegs(leg_d * 0.5 + 0.035, rack_legs_);
        auto iter = std::remove_if(rack_legs_.begin(), rack_legs_.end(),
                                   [](const Eigen::Vector2f &leg) { return leg.norm() > valid_leg_range_; });
        rack_legs_.erase(iter, rack_legs_.end());
    }

    void getEdges(const float length, const float width) {
        edges_.clear();
        KdTree2D kd_tree_2d;
        kd_tree_2d.build(rack_legs_);
        std::vector<int> indices;
        std::vector<float> squared_distances;
        const float radius = std::max(length, width) + tolerant_length_;
        for (int leg_index1 = 0; leg_index1 < kd_tree_2d.size(); leg_index1++) {
            kd_tree_2d.radiusSearch(kd_tree_2d[leg_index1], radius, indices, squared_distances);
            for (int i = 0; i < indices.size(); i++) {
                int leg_index2 = indices[i];
                if (leg_index1 < leg_index2) {
                    const float distance = std::sqrt(squared_distances[i]);
                    if (std::abs(distance - length) < tolerant_length_) {
                        edges_[leg_index1].emplace_back(leg_index2, EdgeType::LENGTH);
                        edges_[leg_index2].emplace_back(leg_index1, EdgeType::LENGTH);
                    }
                    if (std::abs(distance - width) < tolerant_length_) {
                        edges_[leg_index1].emplace_back(leg_index2, EdgeType::WIDTH);
                        edges_[leg_index2].emplace_back(leg_index1, EdgeType::WIDTH);
                    }
                }
            }
        }
    }

    void getTriangles() {
        triangles_.clear();
        for (const auto &leg_info : edges_) {
            const int leg_index0 = leg_info.first;
            const auto &edges = leg_info.second;
            for (int i = 0; i < edges.size(); i++) {
                const int leg_index1 = edges[i].first;
                const bool is_length = edges[i].second == EdgeType::LENGTH;
                const EdgeType search_edge_type = is_length ? EdgeType::WIDTH : EdgeType::LENGTH;
                for (int j = i + 1; j < edges.size(); j++) {
                    const int leg_index2 = edges[j].first;
                    if (edges[j].second == search_edge_type && isRightAngle(leg_index0, leg_index1, leg_index2)) {
                        if (is_length) {
                            triangles_[std::make_pair(leg_index1, leg_index2)].emplace(leg_index0);
                        } else {
                            triangles_[std::make_pair(leg_index2, leg_index1)].emplace(leg_index0);
                        }
                    }
                }
            }
        }
    }

    void getRectangles() {
        rectangles_.clear();
        for (auto iter = triangles_.begin(); iter != triangles_.end(); iter++) {
            const int leg_index1 = iter->first.first;
            const int leg_index2 = iter->first.second;
            auto find_iter = triangles_.find(std::make_pair(leg_index2, leg_index1));
            if (find_iter != triangles_.end()) {
                std::vector<std::pair<int, int>> pairs;
                for (const int leg_index3 : iter->second) {
                    for (const int leg_index4 : find_iter->second) {
                        auto tmp = triangles_.find(std::make_pair(leg_index3, leg_index4));
                        if (tmp != triangles_.end()) {
                            const auto &leg_indices = tmp->second;
                            if (leg_indices.find(leg_index1) != leg_indices.end()) {
                                triangles_[std::make_pair(leg_index3, leg_index4)].erase(leg_index1);
                                pairs.emplace_back(leg_index3, leg_index4);
                            }
                        }
                        tmp = triangles_.find(std::make_pair(leg_index4, leg_index3));
                        if (tmp != triangles_.end()) {
                            const auto &leg_indices = tmp->second;
                            if (leg_indices.find(leg_index2) != leg_indices.end()) {
                                triangles_[std::make_pair(leg_index4, leg_index3)].erase(leg_index2);
                                pairs.emplace_back(leg_index3, leg_index4);
                            }
                        }
                    }
                }
                auto unique_iter = std::unique(pairs.begin(), pairs.end());
                pairs.erase(unique_iter, pairs.end());
                for (const auto &pair : pairs) {
                    rectangles_.emplace_back(leg_index1, leg_index2, pair.first, pair.second);
                    iter->second.erase(pair.first);
                    find_iter->second.erase(pair.second);
                }
            }
        }
    }

    void getRacks(const RackInfo &type, const bool is_inner) {
        for (const auto &rectangle : rectangles_) {
            Rack new_rack(type, is_inner, rack_legs_[std::get<0>(rectangle)], rack_legs_[std::get<1>(rectangle)],
                          rack_legs_[std::get<2>(rectangle)], rack_legs_[std::get<3>(rectangle)]);
            bool is_add = true;
            for (auto &rack : racks_) {
                if ((rack.center - new_rack.center).norm() < center_threshold_) {
                    is_add = false;
                    if (rack.cost > new_rack.cost) {
                        rack = new_rack;
                        break;
                    }
                }
            }
            if (is_add) {
                racks_.push_back(new_rack);
            }
        }
        for (const auto &diagonal : triangles_) {
            const auto &leg1 = rack_legs_[diagonal.first.first];
            const auto &leg2 = rack_legs_[diagonal.first.second];
            for (const auto i : diagonal.second) {
                Rack new_rack(type, is_inner, leg1, leg2, rack_legs_[i]);
                bool is_add = true;
                for (auto &rack : racks_) {
                    if ((rack.center - new_rack.center).norm() < center_threshold_) {
                        is_add = false;
                        if (rack.cost > new_rack.cost) {
                            rack = new_rack;
                            break;
                        }
                    }
                }
                if (is_add) {
                    racks_.push_back(new_rack);
                }
            }
        }
    }

    void getRacks(const RackInfo &type) {
        getEdges(type.leg_groups.front().length, type.leg_groups.front().width);
        getTriangles();
        getRectangles();
        getRacks(type, true);
        if (type.leg_groups.size() > 1) {
            getEdges(type.leg_groups.back().length, type.leg_groups.back().width);
            getTriangles();
            getRectangles();
            getRacks(type, false);
        }
    }

    bool isRightAngle(const int leg_index0, const int leg_index1, const int leg_index2) const {
        const auto edge1 = rack_legs_[leg_index1] - rack_legs_[leg_index0];
        const auto edge2 = rack_legs_[leg_index2] - rack_legs_[leg_index0];
        const float cos_theta = edge1.dot(edge2) / (edge1.norm() * edge2.norm());
        return std::abs(std::acos(cos_theta) - M_PI_2) < tolerant_angle_;
    }
};
}  // namespace rack_detection

#endif  // PROJECT_RACK_DETECTER_HPP
