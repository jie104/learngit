/**
 * @file navigation_no_net
 *
 * @author pengjiali
 * @date 2/20/20.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_NAVIGATION_ON_NET_H
#define SROS_NAVIGATION_ON_NET_H

#include "core/map/NavigationMap.hpp"
#include "core/map/net/edge.hpp"
#include "core/navigation_path.h"
#include "core/pose.h"
#include "core/state.h"
#include "core/util/utils.h"

namespace nav {

class NavigationOnNet {
 public:
    NavigationOnNet(sros::map::StationMarkGroup station_group, sros::map::net::EdgeGroup edge_group,
                    sros::map::net::NodeGroup node_group, sros::core::LoadState cur_load_state = sros::core::LOAD_NONE,
                    sros::map::MapFileVersion map_file_version = sros::map::FILE_VER_6)
        : station_group_(std::move(station_group)),
          edge_group_(std::move(edge_group)),
          node_group_(std::move(node_group)),
          cur_load_state_(cur_load_state),
          map_file_version_(map_file_version) {}

    void setParameters(double nearest_edge_distance_threshold, double start_pose_rotate_threshold,
                       double rotate_between_path_threshold, double end_pose_rotate_threshold,
                       bool is_move_to_nearest_object_first, double nearest_station_distance_threshold,
                       bool enable_regress_keep_agv_facing_follow_edge, double vehicle_width, double rotate_cost_rate);

    bool checkIsOnNet(sros::core::StationNo_t station_no, const sros::core::Pose& pose);

    /**
     * 获取路网导航路径
     * @param start_station_no
     * 当站点不为0时，起始点只搜索站点所属的边，附近的边不搜索,虽然传入了起始站点，但是用的还是input_start_pose的位置
     * @param dst_station_no
     * @param input_start_pose 保证此位置和当前车辆位置绝对一致
     * @param input_dst_pose
     * @param keep_rotate_path_in_end 是否在路径结尾强制加入旋转，比如路网导航到了直线的末尾就结束了不需要旋转，\
     * 但是src走的过程中可能会走偏，对于有些精度要求搞的场景最好在路径末尾强制旋转一下，让src矫正偏差
     * @param out_dst_pose
     * @param cur_pose 当
     * @return
     */
    sros::core::NavigationPath_vector getNavigationPathOnNet(sros::core::StationNo_t start_station_no,
                                                             sros::core::StationNo_t dst_station_no,
                                                             sros::core::Pose input_start_pose,
                                                             const sros::core::Pose& input_dst_pose,
                                                             bool keep_rotate_path_in_end,
                                                             sros::core::Pose &out_dst_pose);

    sros::core::ErrorCode getFailedCode() const { return failed_code_; }

 private:
    /**
     * 插入多条临时路径到附件边的终点
     * @return
     */
    void addTmpPathsFromStartPoseToNearestEdgesEnd(const sros::core::Pose &input_start_pose);

    bool addTmpPathsFromStartPoseToNearestStations(const sros::core::Pose &input_start_pose);

    void addTmpPathFromNearestEdgeEndToDstPose(const sros::core::Pose &input_dst_pose);

    /**
     * 处理一条路径的情况，就是起点和终点在一条路径上，直接生成从起点到终点的路径即可，这样就避免回归直线的起点再移动到终点
     * 当生成失败时，就交个dijkstra去处理
     * @return
     */
    bool handOnePathSituation();

    void dijkstraGetTheMiniCostPath();

    // 过滤无效路径
    void filterInvalidPath();

    /**
     * @brief 删除无效的贝塞尔路径
     */
    void removeInvalidBezierPath();

    /**
     * @brief 删除太短的路径，很有可能路径的起点等于终点
     */
    void removeTooShortPath();

    void addRotateBetweenPath();

    void addStartRotate(const sros::core::Pose &input_start_pose);

    void addEndRotate(const sros::core::Pose &input_start_pose, bool keep_rotate_path_in_end);

    /**
     * 获取路网上距离pose最近的LineEdge
     * @param pose
     * @return
     */
    std::vector<sros::map::net::Edgef> getNearestEdgesOnNet(const sros::core::Pose &pose) const;

    /**
     * 获取点到边的距离
     * @param pose
     * @return
     */
    double getDistancePointToEdge(const sros::core::Pose &pose, const sros::map::net::Edgef &edge) const;

    /**
     * 获取路径上靠近pose的节点,一个就够，Matrix会提示两个相近的节点
     * @param pose
     * @param node_out
     * @return
     */
    bool getNearestNodeOnNet(const sros::core::Pose &pose, sros::map::net::Nodef &node_out) const;

    /**
     * 计算在路径上需要旋转的角度，仅对全向底盘有效
     * @param edge
     * @param s_facing
     * @param e_facing
     * @return
     */
    double getPathRotateAngle(const sros::map::net::Edgef &edge, double s_facing, double e_facing) const;

    sros::core::StationNo_t getNearestStationOnNet(const sros::core::Pose &pose, double min_threshold = 1e100);

    /**
     * 根据一条贝塞尔拟合一条子贝塞尔曲线，拟合出来的贝塞尔和原始贝塞尔的朝向、load_state、limit限速 一致，
     * @note node1 和node2的坐标不能一致，node1和node2需要在贝塞尔附近
     * @param bezier 原始贝塞尔
     * @param node1 子贝塞尔的一个端点，需要保证在原始贝塞尔曲线附近
     * @param node2 子贝塞尔的一个端点，需要保证在原始贝塞尔曲线附近
     * @return 子贝塞尔
     */
    sros::map::net::Edgef fitSubBezier(const sros::map::net::Edgef &bezier, const sros::map::net::Node<double> &node1,
                                       const sros::map::net::Node<double> &node2);

    /**
     * 将边转换成路径
     * @param edge
     * @return
     */
    sros::core::NavigationPath<double> castEdgeToPath(const sros::map::net::Edgef &edge);

    // 计算贝塞尔曲线长度
    double calcuBezierCurveLength(sros::core::BezierPath bezier_path);

    // 计算贝塞尔曲线曲率
    double calcuBezierCurveCurvature(sros::core::BezierPath bezier_path);

    /**
    * @brief 判断贝赛尔曲线是否合理
    * @param bezier_path 贝赛尔曲线，坐标值的单位m
    * @param length_threshold 曲线最短长度阈值，单位m
    * @param curvature_threshold 最大曲率阈值，单位1/m
    * @return 0表示不合理，1表示合理
    */
    bool checkBezierCurveIsValid(sros::core::BezierPath bezier_path, double length_threshold = 0.1, double curvature_threshold = 10);

 public:
    sros::map::MapFileVersion map_file_version_ = sros::map::FILE_VER_6;  // 地图版本
    // 本类的所有数据单位都为（m）、弧度
    // 临时性的，会被动态增加
    sros::map::StationMarkGroup station_group_;  // 站点列表
    sros::map::net::EdgeGroup edge_group_;
    sros::map::net::NodeGroup node_group_;
    sros::core::NavigationPath_vector paths_;
    sros::core::NavigationPath_vector move_to_nearest_station_paths_;  // 需要加在paths_前面的路径

    sros::map::StationMark start_station_;
    sros::map::StationMark dst_station_;
    sros::map::net::Edgef start_edge_;
    sros::map::net::Edgef dst_edge_;
    sros::core::ErrorCode failed_code_ = sros::core::ERROR_CODE_NONE;  // 失败代码
    sros::core::LoadState cur_load_state_ = sros::core::LOAD_NONE;

    double vehicle_width_ = 0.7;  // 车辆宽度
    double rotate_cost_rate_ = 1.0;  // 旋转比例，比如我们车旋转消耗代价太大，增大此系数避免旋转，1.0时为默认值
    double nearest_edge_distance_threshold_ = 0.16;  // 路网导航判断最近边时允许距离的最大值,单位m
    double start_pose_rotate_threshold_ =
        1 * DEGREE_TO_RAD;  // 路径规划中，agv位置和启动点朝向偏差多少时才需要插入旋转路径.单位弧度
    double rotate_between_path_threshold_ =
        1 * DEGREE_TO_RAD;  // 路径规划中，两条路径之间角度偏差多少时才需要插入旋转路径.单位弧度
    double end_pose_rotate_threshold_ =
        0.1 * DEGREE_TO_RAD;  // 路径规划中，agv位置和终点朝向偏差多少时才需要插入旋转路径.单位弧度
    bool is_move_to_nearest_object_first_ = true;  // 路网导航开始时移动到最近的物体类型，有些情况是先到站点
    double nearest_station_distance_threshold_ = 0.16;  // 附件站点距离，单位m
    bool enable_regress_keep_agv_facing_follow_edge_ =
        false;  // 启用路径回归时车头方向跟随路径前进方向，解决长走廊路径下不允许旋转的问题
};

}  // namespace nav

#endif  // SROS_NAVIGATION_ON_NET_H
