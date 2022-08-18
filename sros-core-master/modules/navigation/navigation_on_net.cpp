/**
 * @file NavigationOnNet
 *
 * @author pengjiali
 * @date 2/20/20.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "navigation_on_net.h"
#include <glog/logging.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <set>
#include "core/linear_algebra_func.h"

using namespace std;
using namespace sros::core;
using namespace sros::map::net;
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

namespace nav {

const double MIN_HANDLE_DISTANCE = 0.0002;         // 最短处理距离（m）
const double MIN_HANDLE_ROTATE = M_PI / 1800 * 2;  // 最小处理角度(rad)

void NavigationOnNet::setParameters(double nearest_edge_distance_threshold, double start_pose_rotate_threshold,
                                    double rotate_between_path_threshold, double end_pose_rotate_threshold,
                                    bool is_move_to_nearest_object_first, double nearest_station_distance_threshold,
                                    bool enable_regress_keep_agv_facing_follow_edge, double vehicle_width,
                                    double rotate_cost_rate) {
    nearest_edge_distance_threshold_ = nearest_edge_distance_threshold;
    start_pose_rotate_threshold_ = start_pose_rotate_threshold;
    rotate_between_path_threshold_ = rotate_between_path_threshold;
    end_pose_rotate_threshold_ = end_pose_rotate_threshold;
    is_move_to_nearest_object_first_ = is_move_to_nearest_object_first;
    nearest_station_distance_threshold_ = nearest_station_distance_threshold;
    enable_regress_keep_agv_facing_follow_edge_ = enable_regress_keep_agv_facing_follow_edge;
    vehicle_width_ = vehicle_width;
    rotate_cost_rate_ = rotate_cost_rate;
}

bool NavigationOnNet::checkIsOnNet(::StationNo_t station_no, const ::Pose &pose) {
    if (station_no != 0) {  // 如果站点编号非0, 则检查站点是否在路网上
        auto start_station = station_group_.getItem(station_no);
        return (start_station.edge_id != 0);  // 如果station_no==0, 说明不在路网站点上
    } else {                                  // 如果站点编号=0, 则检查pose是否在路网一定范围之内
        for (const auto &edge : edge_group_.getItemList()) {
            if (edge.id == 0) {
                LOG(ERROR) << "edge id is 0";
                continue;
            }

            auto distance = getDistancePointToEdge(pose, edge);

            if (distance < nearest_edge_distance_threshold_) {
                return true;
            }
        }
        return false;  // 如果能够找到edge，说明pose在edge附近
    }
}

NavigationPath_vector NavigationOnNet::getNavigationPathOnNet(StationNo_t start_station_no, StationNo_t dst_station_no,
                                                              Pose input_start_pose, const Pose &input_dst_pose,
                                                              bool keep_rotate_path_in_end, Pose &out_dst_pose) {
    out_dst_pose = input_dst_pose;

    LOG(INFO) << "getNavigationPathOnNet! Find path from station " << start_station_no << " -> " << dst_station_no;

    if (start_station_no != 0) {
        start_station_ = station_group_.getItem(start_station_no);
        // NOTE：start_station_ 必须在input_start_pose附近
        LOG(INFO) << "distance of start_station_ and input_start_pose is "
                  << getTwoPointDistance(start_station_.pos.x, start_station_.pos.y, input_start_pose.x(),
                                         input_start_pose.y());

        // 虽然传入了起始站点，但是用的还是input_start_pose的位置
        //        start_station_.pos.x = input_start_pose.x();
        //        start_station_.pos.y = input_start_pose.y();
        start_station_.pos.yaw = input_start_pose.yaw();
    } else {
        if (is_move_to_nearest_object_first_) {
            // 插入多条从起点到最近边终点的临时路径
            addTmpPathsFromStartPoseToNearestEdgesEnd(input_start_pose);
        } else {
            // 插入多条到附近站点的临时路径
            if (!addTmpPathsFromStartPoseToNearestStations(input_start_pose)) {
                return paths_;
            }
        }
    }

    if (dst_station_no != 0) {
        dst_station_ = station_group_.getItem(dst_station_no);
    } else {
        addTmpPathFromNearestEdgeEndToDstPose(input_dst_pose);
    }

    start_edge_ = edge_group_.getItem(start_station_.edge_id);
    dst_edge_ = edge_group_.getItem(dst_station_.edge_id);
    LOG(INFO) << "start_station_.edge_id " << start_station_.edge_id << " dst_station_.edge_id "
              << dst_station_.edge_id;
    LOG(INFO) << "start_station_.id  " << start_station_.id << " dst_station_.id " << dst_station_.id;
    if (start_edge_.id == 0 || dst_edge_.id == 0) {
        LOG(ERROR) << "起点或终点所在edge不存在.";  // UNREACHABLE
        return paths_;
    }

    if (!handOnePathSituation()) {
        dijkstraGetTheMiniCostPath();
    }

    // 如果以上步骤没有生成任何路径，则返回空的path，不继续下面的处理
    if (paths_.empty()) {
        LOG(ERROR) << "==> paths_.empty()";
        return paths_;
    }

    out_dst_pose.x() = dst_station_.pos.x;
    out_dst_pose.y() = dst_station_.pos.y;
    out_dst_pose.yaw() = dst_station_.pos.yaw;

    if (!move_to_nearest_station_paths_.empty()) {
        paths_.insert(paths_.begin(), move_to_nearest_station_paths_.begin(), move_to_nearest_station_paths_.end());
    }

    // 过滤无效路径
    filterInvalidPath();

    // 处理paths中相接处方向不连续的路径
    addRotateBetweenPath();

    // 插入开始处的旋转
    addStartRotate(input_start_pose);

    // 在结尾的地方插入旋转
    addEndRotate(input_start_pose, keep_rotate_path_in_end);

    return paths_;
}
void NavigationOnNet::addTmpPathsFromStartPoseToNearestEdgesEnd(const Pose &input_start_pose) {
    sros::map::net::Nodef node;
    if (!getNearestNodeOnNet(input_start_pose, node)) {
        // 路网上找不到最近的已有节点，则以当前位置重新生成一个节点
        node = sros::map::net::Nodef(node_group_.getAvailableID(), input_start_pose.x(), input_start_pose.y());
        node_group_.addItem(node);
    }

    bool has_line = false;
    sros::map::net::Edgef edge;
    sros::map::net::Edgef line_edge;
    auto nearest_edges = getNearestEdgesOnNet(input_start_pose);
    for (const auto &nearest_edge : nearest_edges) {
        LOG(INFO) << "## nearest edge: " << nearest_edge;
        if (nearest_edge.s_node == node.id) {
            edge = nearest_edge;
            continue;  // 起始点和路径的起点一致，不处理
        }

        switch (nearest_edge.type) {
            case sros::map::net::EDGE_LINE: {
                edge =
                    sros::map::net::makeLine(edge_group_.getAvailableID(), node,
                                             node_group_.getItem(nearest_edge.e_node), nearest_edge.vehicle_direction,
                                             nearest_edge.limit_v, nearest_edge.limit_w, nearest_edge.execute_type);
                
                // 保存直线路径
                has_line = true;
                line_edge = edge;

                // 回归路径的朝向始终和nearest_edge的朝向相同（华为需求，窄通道的回归路径）
                // 角（edge的起点，nearest_edge的起点，nearest_edge的终点）
                auto theta1 = getTurnAngle(Location(edge.sx - nearest_edge.sx, edge.sy - nearest_edge.sy),
                                           Location(nearest_edge.ex - nearest_edge.sx, edge.ey - nearest_edge.sy));
                // 角（edge的起点，nearest_edge的终点，nearest_edge的起点）
                auto theta2 =
                    getTurnAngle(Location(edge.sx - nearest_edge.ex, edge.sy - nearest_edge.ey),
                                 Location(nearest_edge.sx - nearest_edge.ex, nearest_edge.sy - nearest_edge.ey));

                Pose p1(edge.sx, edge.sy), ps(nearest_edge.sx, nearest_edge.sy), pe(nearest_edge.ex, nearest_edge.ey);
                if (std::abs(theta1) > M_PI_2 || std::abs(theta2) > M_PI_2) {
                    // input_start_pose 在边的附近，但是edge与nearest_edge的夹角大于90°
                    LOG(INFO) << "## edge " << nearest_edge.id << " is out : " << theta1 << ", " << theta2;

                    // 启用规划回归路径时车头方向更随路径的方向，解决长走廊不允许旋转的问题,特殊处理
                    if (enable_regress_keep_agv_facing_follow_edge_) {
                        if (std::abs(theta1) <= M_PI_2) {
                            edge.s_facing = fmod(edge.s_facing + M_PI, M_PI * 2);
                            edge.e_facing = fmod(edge.e_facing + M_PI, M_PI * 2);
                            if (nearest_edge.vehicle_direction == VehicleDirection::BACKWARD) {
                                LOG(INFO) << "nearest_edge是一条后退路径，并且edge的起点靠近nearest_edge的终点";
                                edge.vehicle_direction = VehicleDirection::FORWARD;
                            } else {
                                LOG(INFO) << "nearest_edge是一条前进路径，并且edge的起点靠近nearest_edge的终点";
                                edge.vehicle_direction = VehicleDirection::BACKWARD;
                            }
                        }
                    }
                }
                break;
            }
            case sros::map::net::EDGE_CIRCLE: {
                break;
            }
            case sros::map::net::EDGE_BEZIER: {
                // 该接口计算起点和终点有问题，待后续修复
                edge = fitSubBezier(nearest_edge, node, node_group_.getItem(nearest_edge.e_node));
            }
            default: {
                break;
            }
        }

        if (edge_group_.addItem(edge.id, edge)) {  // 添加临时路径到group中
            LOG(INFO) << "Insert tmp edge: " << edge;
        }
    }

    // 以起点pose新建一个临时站点作为起点站点
    // 如果最近的路径中有直线，则优先使用直线
    start_station_.id = station_group_.getAvailableID();
    start_station_.edge_id = has_line ? line_edge.id : edge.id;
    start_station_.pos = sros::map::MarkPointf(input_start_pose.x(), input_start_pose.y(), input_start_pose.yaw());
    station_group_.addItem(start_station_.id, start_station_);
}

bool NavigationOnNet::addTmpPathsFromStartPoseToNearestStations(const ::Pose &input_start_pose) {
    /*
     * 需求背景：
     * 假设有一个T字形的路网，横代表主路，竖代表支路。当agv在支路的终点时，但agv实际位置偏离了支路的终点，且偏离较大。
     * 当agv需要导航去主路时，agv就会规划一条先到主路和支路交叉点的回归路径，但是客户需要agv沿着支路走，
     * 所以就先导航到附近的支路的终点，然后再去沿着支路去主路。
     */
    auto nearest_station_no = getNearestStationOnNet(input_start_pose, nearest_station_distance_threshold_);
    LOG(INFO) << "nearest_station_no = " << nearest_station_no;

    if (nearest_station_no != 0) {
        auto nearest_station = station_group_.getItem(nearest_station_no);

        // NOTE: 此处可以考虑用自由导航生成路径
        auto p = makeLine(input_start_pose.x(), input_start_pose.y(), nearest_station.pos.x, nearest_station.pos.y);
        move_to_nearest_station_paths_.push_back(p);
        // 搜索路径时的起始站点设置为距离当前位置最近的站点
        start_station_ = nearest_station;
    } else {
        LOG(INFO) << "move to nearest station first, but there none station around AGV!";
        failed_code_ = ERROR_CODE_NET_NAV_NONE_STATION_AROUND_AGV;
        return false;
    }

    return true;
}

void NavigationOnNet::addTmpPathFromNearestEdgeEndToDstPose(const Pose &input_dst_pose) {
    // 插入一条从边的终点到目标点的临时路径
    sros::map::net::Nodef node;
    if (!getNearestNodeOnNet(input_dst_pose, node)) {
        node = sros::map::net::Nodef(node_group_.getAvailableID(), input_dst_pose.x(), input_dst_pose.y());
        node_group_.addItem(node);
    }

    sros::map::net::Edgef edge;
    auto nearest_edges = getNearestEdgesOnNet(input_dst_pose);
    for (const auto &nearest_edge : nearest_edges) {
        LOG(INFO) << "## nearest edge: " << nearest_edge;
        if (nearest_edge.e_node == node.id) {
            edge = nearest_edge;
            continue;  // 目标节点和路径的结束节点一致，不处理
        }

        switch (nearest_edge.type) {
            case sros::map::net::EDGE_LINE: {
                edge = sros::map::net::makeLine(edge_group_.getAvailableID(), node_group_.getItem(nearest_edge.s_node),
                                                node, nearest_edge.vehicle_direction, nearest_edge.limit_v,
                                                nearest_edge.limit_w, nearest_edge.execute_type);
                break;
            }
            case sros::map::net::EDGE_CIRCLE: {
                break;
            }
            case sros::map::net::EDGE_BEZIER: {
                edge = fitSubBezier(nearest_edge, node_group_.getItem(nearest_edge.s_node), node);
            }
            default: {
                break;
            }
        }

        edge_group_.addItem(edge.id, edge);  // 添加临时路径到group中
    }

    // 以起点pose新建一个临时站点作为起点站点
    dst_station_.id = station_group_.getAvailableID();
    dst_station_.edge_id = edge.id;  // 将站点加入到临时路径上
    dst_station_.pos = sros::map::MarkPointf(node.x, node.y, input_dst_pose.yaw());

    station_group_.addItem(dst_station_.id, dst_station_);  // 添加临时站点到group中
}

bool NavigationOnNet::handOnePathSituation() {
    if (start_station_.edge_id == dst_station_.edge_id) {  // 出发站点与目标站点在同一条边上
        LOG(WARNING) << "start_station_.edge_id == dst_station_.edge_id";

        if ((cur_load_state_ == LOAD_FREE && start_edge_.execute_type == sros::map::net::EXECUTE_LOAD) ||
            (cur_load_state_ == LOAD_FULL && start_edge_.execute_type == sros::map::net::EXECUTE_FREE)) {
            // 不符合载荷条件，跳过此条路径
            LOG(INFO) << "start station and dst station in the same edge, but load state not the same, no way!";
            return false;
        }

        NavigationPath<double> p;
        switch (start_edge_.type) {
            case sros::map::net::EDGE_LINE: {
                auto start_distance =
                    pow(start_station_.pos.x - start_edge_.sx, 2) + pow(start_station_.pos.y - start_edge_.sy, 2);
                auto dst_distance =
                    pow(dst_station_.pos.x - start_edge_.sx, 2) + pow(dst_station_.pos.y - start_edge_.sy, 2);

                LOG(INFO) << "start_distance = " << start_distance << ", dst_distance = " << dst_distance;

                if (start_distance > dst_distance) {  // 终点在起点前面
                    return false;
                }

                p = makeLine(start_station_.pos.x, start_station_.pos.y, dst_station_.pos.x, dst_station_.pos.y,
                             start_edge_.limit_v, start_edge_.limit_w,
                             static_cast<PathDirection>(start_edge_.vehicle_direction));
                break;
            }
            case sros::map::net::EDGE_CIRCLE: {
                break;
            }
            case sros::map::net::EDGE_BEZIER: {
                auto edge = fitSubBezier(start_edge_,
                                         sros::map::net::Node<double>(0, start_station_.pos.x, start_station_.pos.y),
                                         sros::map::net::Node<double>(0, dst_station_.pos.x, dst_station_.pos.y));

                if (start_station_.pos.x != edge.sx || start_station_.pos.y != edge.sy) {  // 终点在起点前面
                    return false;
                }

                p = castEdgeToPath(edge);
                break;
            }
            default: {
                break;
            }
        }
        paths_.push_back(p);
        return true;
    }
    return false;
}

void NavigationOnNet::dijkstraGetTheMiniCostPath() {
    // 起始站点到所属边起点的距离
    double start_2_s_node_d = std::sqrt(std::pow(start_station_.pos.x - start_edge_.sx, 2) +
                                        std::pow(start_station_.pos.y - start_edge_.sy, 2));

    // 起始站点到所属边终点的距离
    double start_2_e_node_d = std::sqrt(std::pow(start_station_.pos.x - start_edge_.ex, 2) +
                                        std::pow(start_station_.pos.y - start_edge_.ey, 2));

    // 目标站点到所属边起点的距离
    double dst_2_s_node_d =
        std::sqrt(std::pow(dst_station_.pos.x - dst_edge_.sx, 2) + std::pow(dst_station_.pos.y - dst_edge_.sy, 2));

    // 目标站点到所属边终点的距离
    double dst_2_e_node_d =
        std::sqrt(std::pow(dst_station_.pos.x - dst_edge_.ex, 2) + std::pow(dst_station_.pos.y - dst_edge_.ey, 2));

    // 初始化图数据结构

    uint16_t s_node, e_node;    // 搜索的node起点与终点
    double s_facing, e_facing;  // 起点和终点的朝向

    if (start_2_s_node_d <= 0.01) {
        // 如果起始站点与所属边起点的距离过近（站点在边起点上）, 则搜索时以边的起点node作为起始点
        s_node = start_edge_.s_node;
        s_facing = start_edge_.s_facing;
    } else {
        s_node = start_edge_.e_node;
        s_facing = start_edge_.e_facing;
    }

    if (dst_2_e_node_d <= 0.01) {
        // 如果终点站点与所属边终点的距离过近（站点在边终点上）, 则搜索时以边的终点node作为目标点
        e_node = dst_edge_.e_node;
        e_facing = dst_edge_.e_facing;
    } else {
        e_node = dst_edge_.s_node;
        e_facing = dst_edge_.s_facing;
    }

    std::map<uint16_t, double> min_cost_map;  // <节点Id， 起点到此节点的最小花费(m)>， 算法中的d
    // <节点Id， <前一条边的id, 选择此条边的花费>>,用于回溯
    std::map<uint16_t, std::map<sros::map::net::edge_id_t, double> > previous_info_map;

    std::set<sros::map::net::node_id_t> no_search_node_set;  // 还没有被搜索过的节点，算法中的Q

    const int UNDEFINED = 0;
    const double INFINITY_COST = numeric_limits<double>::max();
    const double rotate_cast_per_rad = vehicle_width_ * rotate_cost_rate_ / 2;  // 每一弧度旋转的花费
    LOG(INFO) << "start node " << s_node << " end node " << e_node;

    if (s_node != 0 && s_node == e_node) {
        LOG(INFO) << "起点与终点在同一个节点上, 调整车朝向";
        double rotate_value = dst_station_.pos.yaw;
        paths_.push_back(makeRotate(rotate_value));
        return;
    }

    for (const auto &node : node_group_.getItemList()) {
        // LOG(INFO) << " node id "  << node.id;

        min_cost_map[node.id] = INFINITY_COST;  // 將各點的已知最短距離先設成無窮大infinity
        no_search_node_set.insert(node.id);
        previous_info_map[node.id] = std::map<sros::map::net::edge_id_t, double>();
    }

    min_cost_map[s_node] = 0.0;  // 起点到起点代价为0

    // Dijkstra 主循环
    while (!no_search_node_set.empty()) {
        double cur_min_cost = INFINITY_COST;

        uint16_t min_const_node = 0;  // 最小花费的节点，算法中的u

        map<uint16_t, double>::iterator it;
        for (it = min_cost_map.begin(); it != min_cost_map.end(); ++it) {
            if (no_search_node_set.count(it->first) != 0 && it->second < cur_min_cost) {
                cur_min_cost = it->second;   // 更新当前最小值
                min_const_node = it->first;  // 更新node id
            }
        }

        if (min_const_node == 0) {  // 没有找到任何更小的代价, 停止搜索
            break;
        }

        no_search_node_set.erase(min_const_node);  // 从Q中清除u

        if (min_const_node == e_node) {
            break;  // 如果u=e_node, 说明已搜索到终点,停止搜索
        }

        for (const auto &edge : edge_group_.getItemList()) {
            if ((cur_load_state_ == LOAD_FREE && edge.execute_type == sros::map::net::EXECUTE_LOAD) ||
                (cur_load_state_ == LOAD_FULL && edge.execute_type == sros::map::net::EXECUTE_FREE)) {
                // 不符合载荷条件，跳过此条路径
                continue;
            }

            if (edge.s_node == min_const_node) {  // 以u为出度的edge
                uint16_t v = edge.e_node;

                // 拓展边（u,v）。w(u,v)为从u到v的路径长度。
                double new_min_cost = INFINITY_COST;

                if (min_const_node == s_node) {  // 起始点
                    new_min_cost = edge.cost + minRotate(s_facing, edge.s_facing) * rotate_cast_per_rad;
                } else {
                    const auto &node_previous_info = previous_info_map.at(min_const_node);
                    for (const auto &it : node_previous_info) {
                        auto previous_edge = edge_group_.getItem(it.first);
                        auto tmp_cost = edge.cost +
                                        minRotate(previous_edge.e_facing, edge.s_facing) * rotate_cast_per_rad +
                                        it.second;
                        if (new_min_cost > tmp_cost) {
                            new_min_cost = tmp_cost;
                        }
                    }
                }
                previous_info_map[v][edge.id] = new_min_cost;

                if (min_cost_map[v] > new_min_cost) {
                    min_cost_map[v] = new_min_cost;
                }
            }
        }
    }

    if (min_cost_map[e_node] == INFINITY_COST) {
        // s到达t的cost为无穷大,说明无路径可达
        LOG(INFO) << "==> NOWAY";
        failed_code_ = ERROR_CODE_NET_NAV_FIND_PATH_NO_WAY;
        return;
    }

    // 回溯得到边序列,从结尾开始，将其顺序反过来
    std::list<sros::map::net::edge_id_t> edge_list;
    auto node = e_node;
    while (node != s_node) {                                          // 从终点找到起点
        const auto &node_previous_info = previous_info_map.at(node);  // 当前节点前面每条边的话费
        double min_cost = INFINITY_COST;             // 选择当前节点到上一个节点最小花费
        sros::map::net::edge_id_t mini_edge_id = 0;  // 选择当前节点到上一个节点最小花费对应的边id
        if (node == e_node) {                        // 结束点特殊处理
            // 选择加上结束时朝向，走那条边花费最小
            for (const auto &it : node_previous_info) {
                auto previous_edge = edge_group_.getItem(it.first);
                auto rotate = minRotate(e_facing, previous_edge.e_facing);
                auto tmp_cost = rotate * rotate_cast_per_rad + it.second;
                if (min_cost > tmp_cost) {
                    min_cost = tmp_cost;
                    mini_edge_id = it.first;
                    node = previous_edge.s_node;
                }
            }
        } else {
            // 选择走那条边花费最小
            for (const auto &it : node_previous_info) {
                auto previous_edge = edge_group_.getItem(it.first);
                if (min_cost > it.second) {
                    min_cost = it.second;
                    mini_edge_id = it.first;
                    node = previous_edge.s_node;
                }
            }
        }
        edge_list.push_front(mini_edge_id);
    }

    LOG(INFO) << "edge list size " << edge_list.size() << " ==> edge list: ";
    for (const auto &edge_id : edge_list) {
        LOG(INFO) << "edge id: " << edge_id;
    }

    // 从站点位置到达所属edge终点的路径
    // 如果站点到边起点或终点的距离过近, 则忽略掉这条直线路径
    if (start_2_s_node_d >= 0.01 && start_2_e_node_d >= 0.01) {
        LinePath p;
        switch (start_edge_.type) {
            case sros::map::net::EDGE_LINE: {
                p = makeLine(start_station_.pos.x, start_station_.pos.y, start_edge_.ex, start_edge_.ey,
                             start_edge_.limit_v, start_edge_.limit_w,
                             static_cast<PathDirection>(start_edge_.vehicle_direction));
                break;
            }
            case sros::map::net::EDGE_CIRCLE: {
                break;
            }
            case sros::map::net::EDGE_BEZIER: {
                auto edge =
                    fitSubBezier(dst_edge_, sros::map::net::Node<double>(0, start_station_.pos.x, start_station_.pos.y),
                                 sros::map::net::Node<double>(0, start_edge_.ex, start_edge_.ey));
                p = castEdgeToPath(edge);
                break;
            }
            default: {
                break;
            }
        }

        paths_.push_back(p);
        LOG(INFO) << "Add start station to start edge end node " << p;
    }

    auto it = edge_list.begin();
    for (const auto &edge_id : edge_list) {
        // 可能从edge_s_node和edge_e_node之间存在多条边，需要选择上次记录搜索时话费最小的
        auto edge = edge_group_.getItem(edge_id);

        LOG(INFO) << edge;
        paths_.push_back(castEdgeToPath(edge));
    }

    // 从目标站点所属edge起点到达站点位置的路径
    // 如果站点与边起点或终点的距离过近, 则忽略掉这条直线路径
    if (dst_2_s_node_d >= 0.01 && dst_2_e_node_d >= 0.01) {
        LinePath p;
        switch (dst_edge_.type) {
            case sros::map::net::EDGE_LINE: {
                p = makeLine(dst_edge_.sx, dst_edge_.sy, dst_station_.pos.x, dst_station_.pos.y, dst_edge_.limit_v,
                             dst_edge_.limit_w, static_cast<PathDirection>(dst_edge_.vehicle_direction));
                break;
            }
            case sros::map::net::EDGE_CIRCLE: {
                break;
            }
            case sros::map::net::EDGE_BEZIER: {
                auto edge = fitSubBezier(dst_edge_, sros::map::net::Node<double>(0, dst_edge_.sx, dst_edge_.sy),
                                         sros::map::net::Node<double>(0, dst_station_.pos.x, dst_station_.pos.y));
                p = castEdgeToPath(edge);
                break;
            }
            default: {
                break;
            }
        }

        paths_.push_back(p);
        LOG(INFO) << "Add dst edge start node to dst station " << p;
    }
}

// 过滤无效路径
void NavigationOnNet::filterInvalidPath() {
    // 过滤无效的贝塞尔路径
    removeInvalidBezierPath();

    // 过滤太短的路径
    removeTooShortPath();
}

void NavigationOnNet::removeInvalidBezierPath() {
    LOG(INFO) << "start check bezier path : " << paths_.size();
    for (auto it = paths_.begin(); it != paths_.end();) {
        if (it->type_ != PATH_BEZIER) {
            ++it;
            continue;
        }

        ::NavigationPath<double> path = BezierPath(it->sx(), it->sy(), it->cx(), it->cy(), it->dx(), it->dy(), it->ex(), it->ey(), it->radius_,
                       (PathDirection)it->direction(), it->s_facing_, it->e_facing_, it->limit_v_, it->limit_w_);
        if (!checkBezierCurveIsValid(path)) {
            it = paths_.erase(it);
        } else {
            ++it;
        }
    }
}

void NavigationOnNet::removeTooShortPath() {
    for (auto it = paths_.begin(); it != paths_.end();) {
        double distance = getTwoPointDistance(it->sx_, it->sy_, it->ex_, it->ey_);

        if (distance < MIN_HANDLE_DISTANCE) {
            it = paths_.erase(it);
        } else {
            ++it;
        }
    }
}

void NavigationOnNet::addRotateBetweenPath() {
    NavigationPath_vector ps;

    if (paths_.size() < 2) {
        return;
    }

    auto path_it = paths_.begin();
    NavigationPath<double> p = *path_it;
    p.avoid_policy_ = OBSTACLE_AVOID_WAIT;  // 路网上的避障策略始终为WAIT
    ps.push_back(p);

    for (int i = 0; i < paths_.size() - 1; i++) {
        NavigationPath<double> path1 = *path_it;

        path_it++;
        NavigationPath<double> path2 = *path_it;

        LOG(INFO) << "path type: " << path1.type_ << " path1.s_facing_: " << std::to_string(path1.s_facing_)
                  << " path1.e_facing_: " << std::to_string(path1.e_facing_);
        LOG(INFO) << "path type: " << path2.type_ << " path2.s_facing_: " << std::to_string(path2.s_facing_)
                  << " path2.e_facing_: " << std::to_string(path2.e_facing_);

        double theta = minRotate(path1.e_facing_, path2.s_facing_);
        LOG(INFO) << "theta: " << theta << " angle1: " << path1.e_facing_ << " angle2: " << path2.s_facing_;

        if (fabs(theta) > rotate_between_path_threshold_) {  // 如果theta大于一个阈值, 则插入原地旋转路径
            ps.push_back(RotatePath(path1.ex_, path1.ey_, path2.s_facing_));
            LOG(INFO) << "Add " << ps.back();
        }

        path2.avoid_policy_ = OBSTACLE_AVOID_WAIT;
        ps.push_back(path2);
    }

    paths_ = std::move(ps);
}

void NavigationOnNet::addStartRotate(const Pose &input_start_pose) {
    if (paths_.empty()) {
        return;
    }

    auto first_path = paths_.begin();

    double theta = minRotate(input_start_pose.yaw(), first_path->s_facing_);
    LOG(INFO) << "rotate_value: " << theta;
    if (theta >= start_pose_rotate_threshold_) {
        // 如果旋转角度大于起点旋转阈值（默认为1°）, 则加入原地旋转
        paths_.insert(paths_.begin(), RotatePath(start_station_.pos.x, start_station_.pos.y, first_path->s_facing_));
        LOG(INFO) << "Add start pose " << paths_.front();
    }
}

void NavigationOnNet::addEndRotate(const Pose &input_start_pose, bool keep_rotate_path_in_end) {
    if (paths_.empty()) {  // 处理起点到终点路径太近，导致所有路径都被删除掉了，这时候只要旋转一下就够了
        auto delta = minRotate(input_start_pose.yaw(), dst_station_.pos.yaw);
        if (fabs(delta) >= end_pose_rotate_threshold_ || keep_rotate_path_in_end) {
            // 如果旋转角度大于0.1度, 则加入原地旋转,或者强制加入旋转，来矫正src行走过程中的偏差
            paths_.push_back(RotatePath(dst_station_.pos.x, dst_station_.pos.y, dst_station_.pos.yaw));
            LOG(INFO) << "Add end rotate " << paths_.back();
        }
        return;
    }

    //处理最后一条路径是旋转路径
    //获取当前路径中的最后一条的方向
    NavigationPath<double> cur_path_last = paths_.back();  //保证这个变量名是首次被使用

    // 此处与起始点处相反, 需要获取edge相对于站点朝向的旋转角度
    double delta = minRotate(cur_path_last.e_facing_, dst_station_.pos.yaw);

    if (fabs(delta) >= end_pose_rotate_threshold_ || keep_rotate_path_in_end) {
        // 如果旋转角度大于0.1度, 则加入原地旋转,或者强制加入旋转，来矫正src行走过程中的偏差
        paths_.push_back(RotatePath(dst_station_.pos.x, dst_station_.pos.y, dst_station_.pos.yaw));
        LOG(INFO) << "Add end rotate " << paths_.back();
    }
}

std::vector<sros::map::net::Edgef> NavigationOnNet::getNearestEdgesOnNet(const ::Pose &pose) const {
    using sros::map::net::Edgef;

    vector<Edgef> nearest_edges;

    auto time_start = sros::core::util::get_time_in_ns();
    for (const auto &edge : edge_group_.getItemList()) {
        if (edge.id == 0) {
            LOG(ERROR) << "edge id is 0";
            continue;
        }

        auto distance = getDistancePointToEdge(pose, edge);

        if (distance < nearest_edge_distance_threshold_) {
            nearest_edges.push_back(edge);
        }
    }
    auto time_end = sros::core::util::get_time_in_ns();

    LOG(INFO) << "nearest_edge_distance_threshold_: " << nearest_edge_distance_threshold_
              << ", nearest_edges.size: " << nearest_edges.size() << ", time cost " << time_end - time_start << "ns";

    return nearest_edges;
}

double NavigationOnNet::getDistancePointToEdge(const sros::core::Pose &pose, const sros::map::net::Edgef &edge) const {
    using sros::map::net::EdgeType;

    double distance = numeric_limits<double>::max();
    switch (edge.type) {
        case EdgeType::EDGE_LINE: {
            distance = getDistanceToSegment(pose, Pose(Location(edge.sx, edge.sy)), Pose(Location(edge.ex, edge.ey)));
            break;
        }
        case EdgeType::EDGE_BEZIER: {
            /**
             * 由于直接找点到贝塞尔曲线距离太耗时了，我们先过滤一下距离贝塞尔太远的点.
             * 思路是：点到贝塞尔四个点组成的任意三角形的距离小于在路径上的阈值时就需要用贝塞尔算路径了
             */
            polygon_type poly;
            boost::geometry::append(poly.outer(), point_type(edge.sx, edge.sy));  // 0
            boost::geometry::append(poly.outer(), point_type(edge.cx, edge.cy));  // 1
            boost::geometry::append(poly.outer(), point_type(edge.dx, edge.dy));  // 2
            boost::geometry::append(poly.outer(), point_type(edge.sx, edge.sy));  // 0
            boost::geometry::append(poly.outer(), point_type(edge.ex, edge.ey));  // 3
            boost::geometry::append(poly.outer(), point_type(edge.cx, edge.cy));  // 1
            boost::geometry::append(poly.outer(), point_type(edge.dx, edge.dy));  // 2
            boost::geometry::append(poly.outer(), point_type(edge.ex, edge.ey));  // 3
            distance = boost::geometry::distance(point_type(pose.x(), pose.y()), poly);
            if (distance < nearest_edge_distance_threshold_) {
                distance = getDistanceToBezier(pose.x(), pose.y(), edge.sx, edge.sy, edge.cx, edge.cy, edge.dx, edge.dy,
                                               edge.ex, edge.ey)
                               .second;
            }
            break;
        }
        case EdgeType::EDGE_CIRCLE: {
            // FIXME(pengjiali): 恢复圆弧时
            break;
        }
    }

    return distance;
}

bool NavigationOnNet::getNearestNodeOnNet(const ::Pose &pose, sros::map::net::Nodef &node_out) const {
    for (const auto &node : node_group_.getItemList()) {
        if (node.id == 0) {
            LOG(WARNING) << "node id is 0";
            continue;
        }

        if (getTwoPointDistance(pose.x(), pose.y(), node.x, node.y) < MIN_HANDLE_DISTANCE) {
            node_out = node;
            return true;
        }
    }

    return false;
}

double NavigationOnNet::getPathRotateAngle(const sros::map::net::Edgef &edge, double s_facing, double e_facing) const {
    // 如果地图不包含s_facing等数据，则不能够计算路径上的旋转角度
    if (map_file_version_ < sros::map::FILE_VER_5) {
        return 0;
    }

    double theta = 0;

    if (edge.direction == sros::map::net::EDGE_FOLLOW_S) {
        theta = 0;
    } else if (edge.direction == sros::map::net::EDGE_FOLLOW_E) {
        // TODO(nobody): 实现EDGE_FOLLOW_E
    } else if (edge.direction == sros::map::net::EDGE_ROTATE) {
        theta = e_facing - s_facing;

        // 当计算出的旋转方向与用户要求的不相同时，需要改变旋转方向
        if ((edge.rotate_direction == sros::map::net::EDGE_ROTATE_ACW && theta < 0) ||
            (edge.rotate_direction == sros::map::net::EDGE_ROTATE_CW && theta > 0)) {
            theta = -1 * (theta / fabs(theta)) * (2 * M_PI - fabs(theta));
        }
    }

    return theta;
}

::StationNo_t NavigationOnNet::getNearestStationOnNet(const ::Pose &pose, double min_threshold) {
    double cur_min_distance = min_threshold;
    StationNo_t cur_min_station = 0;
    for (const auto &s : station_group_.getItemList()) {
        if (s.edge_id == 0) {  // 仅考虑在路网上的站点
            continue;
        }

        auto distance = pose.distance_to(Pose(Location(s.pos.x, s.pos.y)));

        if (distance < cur_min_distance) {
            cur_min_distance = distance;
            cur_min_station = s.id;
        }
    }

    return cur_min_station;
}

sros::map::net::Edgef NavigationOnNet::fitSubBezier(const sros::map::net::Edgef &bezier,
                                                    const sros::map::net::Node<double> &node1,
                                                    const sros::map::net::Node<double> &node2) {
    // 先求哪个是起点，哪个是终点
    auto result1 = getDistanceToBezier(node1.x, node1.y, bezier.sx, bezier.sy, bezier.cx, bezier.cy, bezier.dx,
                                       bezier.dy, bezier.ex, bezier.ey);
    auto result2 = getDistanceToBezier(node2.x, node2.y, bezier.sx, bezier.sy, bezier.cx, bezier.cy, bezier.dx,
                                       bezier.dy, bezier.ex, bezier.ey);
    sros::map::net::Node<double> start_node;
    sros::map::net::Node<double> end_node;
    std::pair<double, double> start_result;
    if (result1.first < result2.first) {
        start_node = node1;
        end_node = node2;
        start_result = result1;
    } else {
        start_node = node2;
        end_node = node1;
        start_result = result2;
    }

    // 先拟合一段从start_node到bezier终点的贝塞尔
    double t = start_result.first;
    auto getNewPointFunc = [&](double a, double b, double t) {
        // c = (1-t1)*a + t1*b
        return (1 - t) * a + t * b;
    };
    double p2x = getNewPointFunc(bezier.dx, bezier.ex, t);
    double p2y = getNewPointFunc(bezier.dy, bezier.ey, t);
    double p12x = getNewPointFunc(bezier.cx, bezier.dx, t);
    double p12y = getNewPointFunc(bezier.cy, bezier.dy, t);
    double p1x = getNewPointFunc(p12x, p2x, t);
    double p1y = getNewPointFunc(p12y, p2y, t);

    // 在上一次拟合的基础上，拟合一条从start_node到end_node的贝塞尔
    auto end_result = getDistanceToBezier(end_node.x, end_node.y, start_node.x, start_node.y, p1x, p1y, p2x, p2y,
                                          bezier.ex, bezier.ey);
    t = end_result.first;
    double final_p1x = getNewPointFunc(start_node.x, p1x, t);
    double final_p1y = getNewPointFunc(start_node.y, p1y, t);
    double final_p12x = getNewPointFunc(p1x, p2x, t);
    double final_p12y = getNewPointFunc(p1y, p2y, t);
    double final_p2x = getNewPointFunc(final_p1x, final_p12x, t);
    double final_p2y = getNewPointFunc(final_p1y, final_p12y, t);
    return sros::map::net::makeBezier(edge_group_.getAvailableID(), start_node, end_node, final_p1x, final_p1y,
                                      final_p2x, final_p2y, bezier.vehicle_direction, bezier.limit_v, bezier.limit_w,
                                      bezier.execute_type);
}

::NavigationPath<double> NavigationOnNet::castEdgeToPath(const sros::map::net::Edgef &edge) {
    ::NavigationPath<double> p;
    if (edge.type == sros::map::net::EDGE_LINE) {
        p = LinePath(edge.sx, edge.sy, edge.ex, edge.ey, edge.s_facing, edge.e_facing, edge.limit_v, edge.limit_w);
    } else if (edge.type == sros::map::net::EDGE_CIRCLE) {
        // 区分顺圆逆圆
        p = CirclePath(edge.sx, edge.sy, edge.ex, edge.ey, edge.cx, edge.cy, edge.radius, edge.s_facing, edge.e_facing,
                       edge.limit_v, edge.limit_w);
    } else if (edge.type == sros::map::net::EDGE_BEZIER) {
        p = BezierPath(edge.sx, edge.sy, edge.cx, edge.cy, edge.dx, edge.dy, edge.ex, edge.ey, edge.radius,
                       PATH_FORWARD, edge.s_facing, edge.e_facing, edge.limit_v, edge.limit_w);
    }
    p.rotate_angle_ = getPathRotateAngle(edge, edge.s_facing, edge.e_facing);
    p.direction_ = static_cast<int>(edge.vehicle_direction);

    return p;
}

// 计算贝塞尔曲线长度
double NavigationOnNet::calcuBezierCurveLength(sros::core::BezierPath bezier_path) {
    double length = 0;
    int n = 10; //将贝塞尔曲线分段的段数，越大精度越高
    std::vector<sros::core::Pose> curve;
    for(int i = 0; i < n + 1; i++) {
        double t = i * 1.0 / n;
        double x = pow(1-t, 3) * bezier_path.sx() + 3 * pow(1-t, 2) * t * bezier_path.cx() + 3 * (1-t) * pow(t, 2) * bezier_path.dx() + pow(t, 3) * bezier_path.ex();
        double y = pow(1-t, 3) * bezier_path.sy() + 3 * pow(1-t, 2) * t * bezier_path.cy() + 3 * (1-t) * pow(t, 2) * bezier_path.dy() + pow(t, 3) * bezier_path.ey();
        curve.push_back(sros::core::Pose(x, y, 0));
    }

    for(int i = 1; i < curve.size(); i++) {
        length += hypot(curve[i].y() - curve[i-1].y(), curve[i].x() - curve[i-1].x());
    }
    LOG(INFO) << "calcute the length of bezier path is : " << length;
    return length;
}

// 计算贝塞尔曲线曲率
double NavigationOnNet::calcuBezierCurveCurvature(sros::core::BezierPath bezier_path) {
    int n = 20; //将贝塞尔曲线分段的段数，越大精度越高
    std::vector<sros::core::Pose> curve;
    for(int i = 0; i < n + 1; i++) {
        double t = i * 1.0 / n;
        double x = pow(1-t, 3) * bezier_path.sx() + 3 * pow(1-t, 2) * t * bezier_path.cx() + 3 * (1-t) * pow(t, 2) * bezier_path.dx() + pow(t, 3) * bezier_path.ex();
        double y = pow(1-t, 3) * bezier_path.sy() + 3 * pow(1-t, 2) * t * bezier_path.cy() + 3 * (1-t) * pow(t, 2) * bezier_path.dy() + pow(t, 3) * bezier_path.ey();
        curve.push_back(sros::core::Pose(x, y, 0));
    }

    double max_curvature = 0;
    for(int i = 2; i < curve.size(); i++) {
        double c = hypot(curve[i-1].y() - curve[i-2].y(), curve[i-1].x() - curve[i-2].x());
        double a = hypot(curve[i].y() - curve[i-1].y(), curve[i].x() - curve[i-1].x());
        double b = hypot(curve[i].y() - curve[i-2].y(), curve[i].x() - curve[i-2].x());
        if(a == 0 || c == 0) {
            continue;
        }
        double cos_B = 0.5 * (a * a + c * c - b * b) / (a * c);
        double curvature = 2 * sqrt(0.5 * (1 + cos_B)) / c;
        max_curvature = max_curvature > curvature ? max_curvature : curvature;
    }
    LOG(INFO) << "calcute the vature of bezier path is : " << max_curvature;
    return max_curvature;
}

/**
* @brief 判断贝赛尔曲线是否合理
* @param bezier_path 贝赛尔曲线，坐标值的单位m
* @param length_threshold 曲线最短长度阈值，单位m
* @param curvature_threshold 最大曲率阈值，单位1/m
* @return 0表示不合理，1表示合理
*/
bool NavigationOnNet::checkBezierCurveIsValid(sros::core::BezierPath bezier_path, double length_threshold /*= 0.1*/, double curvature_threshold /*= 10*/) {
    LOG(INFO) << "bezier path : " << bezier_path;
    double bezier_path_length = calcuBezierCurveLength(bezier_path);
    if(bezier_path_length < length_threshold) {
        LOG(WARNING) << "bezier path is too short";
        return false;
    }

    // 经与运动控制协商，曲率过滤条件暂不放开
    /*double bezier_path_max_curvature = calcuBezierCurveCurvature(bezier_path);
    if(bezier_path_max_curvature > curvature_threshold) {
        LOG(WARNING) << "curvature of bezier path is too large";
        return false;
    }*/
    return true;
}

}  // namespace nav