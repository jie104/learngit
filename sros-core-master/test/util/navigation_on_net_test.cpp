/**
 * @file navigation_no_net_test
 *
 * @author pengjiali
 * @date 2/21/20.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include "core/map_manager.h"
#include "modules/navigation/navigation_on_net.h"
#include "navigation_info_peep.h"

using namespace std;
using namespace nav;
using namespace sros::core;
using namespace sros::map;
using namespace sros::map::net;

class NavigationOnNetTest : public testing::Test {
 protected:
    void SetUp() override {}

    void printPaths(const NavigationPath_vector &paths) {
        LOG(INFO) << std::endl;
        LOG(INFO) << "-------------------paths--------------------";
        for (auto i = 0; i < paths.size(); ++i) {
            LOG(INFO) << i << "th --> " << paths.at(i);
        }
        LOG(INFO) << std::endl;
    }

    Pose getPose(const StationMark &station) { return Pose(station.pos.x, station.pos.y, station.pos.yaw); }

    Pose getPose(const Nodef &node) { return Pose(node.x, node.y, node.yaw); }

    /**
     * @brief 绘制生产的路径
     * @note 此函数会阻塞
     * @param navigation_on_net
     */
    void show(const NavigationOnNet &navigation_on_net) {
        NavigationInfoPeep peep;
        peep.drawNodes(navigation_on_net.node_group_.getItemList());
        peep.drawEdges(navigation_on_net.edge_group_.getItemList());
        peep.drawPaths(navigation_on_net.paths_);
        peep.drawStations(navigation_on_net.start_station_, navigation_on_net.dst_station_);
        peep.show();
    }
};

/**
 * 测试一条向上走的路径
 */
TEST_F(NavigationOnNetTest, OneLineTest) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef node1(1, 0, 0, 0);
    nodes.addItem(node1);
    Nodef node2(2, 0, 100, 0);
    nodes.addItem(node2);
    edges.addItem(makeLine(1, node1, node2));
    StationMark station1(1, 1, 0, 0, M_PI);
    stations.addItem(station1);
    StationMark station2(2, 1, 0, 100, -M_PI);
    stations.addItem(station2);

    NavigationOnNet navigation_no_net(stations, edges, nodes);
    Pose out_dst_pose;
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, Pose(0.1, 50, 0)));
    auto paths = navigation_no_net.getNavigationPathOnNet(1, 2, getPose(station1), Pose(), true, out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 3);
    ASSERT_EQ(paths.front().type_, PATH_ROTATE);
    ASSERT_EQ(paths.at(1).type_, PATH_LINE);
    ASSERT_EQ(paths.back().type_, PATH_ROTATE);
    ASSERT_TRUE(out_dst_pose == getPose(station2));
    ASSERT_EQ(paths.front().rotate_angle_, paths.at(1).s_facing_);
    ASSERT_EQ(paths.back().rotate_angle_, station2.pos.yaw);
    ASSERT_EQ(paths.at(1).direction_, PATH_FORWARD);
}

/**
 * 测试一条向右的倒退路径
 */
TEST_F(NavigationOnNetTest, OneBackLineTest) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef node1(1, 0, 0, 0);
    nodes.addItem(node1);
    Nodef node2(2, -100, 0, 0);
    nodes.addItem(node2);
    edges.addItem(makeLine(1, node1, node2, VehicleDirection::BACKWARD));
    StationMark station1(1, 1, 0, 0, M_PI_2);
    stations.addItem(station1);
    StationMark station2(2, 1, -100, 0, -M_PI_2);
    stations.addItem(station2);

    NavigationOnNet navigation_no_net(stations, edges, nodes);
    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(1, 2, getPose(station1), Pose(), true, out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 3);
    ASSERT_EQ(paths.front().type_, PATH_ROTATE);
    ASSERT_EQ(paths.at(1).type_, PATH_LINE);
    ASSERT_EQ(paths.back().type_, PATH_ROTATE);
    ASSERT_TRUE(out_dst_pose == getPose(station2));
    ASSERT_EQ(paths.front().rotate_angle_, paths.at(1).s_facing_);
    ASSERT_EQ(paths.back().rotate_angle_, station2.pos.yaw);
    ASSERT_EQ(paths.at(1).direction_, PATH_BACKWARD);
}

/**
 * 测试贝塞尔回归
 */
TEST_F(NavigationOnNetTest, OneBezierTest) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef node1(1, -5, 0, 0);
    nodes.addItem(node1);
    Nodef node2(2, 5, 0, 0);
    nodes.addItem(node2);
    edges.addItem(makeBezier(1, node1, node2, 0.40, 0.80, -0.30, -0.10));

    NavigationOnNet navigation_no_net(stations, edges, nodes);
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, Pose(-5, 0, 0)));
    ASSERT_FALSE(navigation_no_net.checkIsOnNet(0, Pose(0, 0, 0)));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, Pose(5, 0, 0)));

    Pose input_start_pose(-1.10799, 0.257211, 0), input_dst_pose;
    input_dst_pose = getPose(node2);
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, input_start_pose));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, input_dst_pose));

    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(0, 0, input_start_pose, input_dst_pose, true, out_dst_pose);
    //    show(navigation_no_net);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 3);
    ASSERT_EQ(paths.front().type_, PATH_ROTATE);
    ASSERT_EQ(paths.at(1).type_, PATH_BEZIER);
    ASSERT_EQ(paths.back().type_, PATH_ROTATE);
    ASSERT_TRUE(out_dst_pose == getPose(node2));
    ASSERT_EQ(paths.front().rotate_angle_, paths.at(1).s_facing_);
    ASSERT_EQ(paths.back().rotate_angle_, node2.yaw);
    ASSERT_EQ(paths.at(1).direction_, PATH_FORWARD);
}

/**
 * 路径旋转中加入旋转花费
 */
TEST_F(NavigationOnNetTest, RotateCastTest) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef node_left(1, -1, 0, 0);
    Nodef node_right(2, 1, 0, 0);
    Nodef node_top(3, 0, 1, 0);
    Nodef node_bottom(4, 0, -1, 0);
    Nodef node_s(5, -5, 0, 0);
    Nodef node_e(6, 5, 0, 0);
    nodes.addItem(node_s);
    nodes.addItem(node_e);
    nodes.addItem(node_left);
    nodes.addItem(node_right);
    nodes.addItem(node_top);
    nodes.addItem(node_bottom);
    edges.addItem(makeLine(1, node_left, node_top));
    edges.addItem(makeLine(2, node_top, node_right));
    edges.addItem(makeLine(3, node_left, node_bottom));
    edges.addItem(makeLine(4, node_bottom, node_right));
    edges.addItem(makeLine(5, node_s, node_left));
    edges.addItem(makeLine(6, node_right, node_e));

    StationMark station_s(1, 5, node_s.x, node_s.y, edges.getItem(5).s_facing);
    stations.addItem(station_s);
    StationMark station_e(2, 2, node_right.x, node_right.y, edges.getItem(2).e_facing);
    stations.addItem(station_e);

    // 测试终点方向选择
    NavigationOnNet navigation_no_net(stations, edges, nodes);
    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(1, 2, getPose(station_s), Pose(), true, out_dst_pose);
    printPaths(paths);
    //    show(navigation_no_net);
    ASSERT_EQ(paths.size(), 6);
    ASSERT_EQ(paths.front().type_, PATH_LINE);
    ASSERT_EQ(paths.at(1).type_, PATH_ROTATE);
    ASSERT_EQ(paths.at(4).type_, PATH_LINE);
    ASSERT_EQ(paths.at(4).sy_, node_top.y);
    ASSERT_TRUE(out_dst_pose == getPose(station_e));

    station_e = StationMark(2, 4, node_right.x, node_right.y, edges.getItem(4).e_facing);
    stations.updateItem(station_e);
    navigation_no_net = NavigationOnNet(stations, edges, nodes);
    paths = navigation_no_net.getNavigationPathOnNet(1, 2, getPose(station_s), Pose(), true, out_dst_pose);
    printPaths(paths);
    //    show(navigation_no_net);
    ASSERT_EQ(paths.size(), 6);
    ASSERT_EQ(paths.front().type_, PATH_LINE);
    ASSERT_EQ(paths.at(1).type_, PATH_ROTATE);
    ASSERT_EQ(paths.at(4).type_, PATH_LINE);
    ASSERT_EQ(paths.at(4).sy_, node_bottom.y);
    ASSERT_TRUE(out_dst_pose == getPose(station_e));

    // 测试起点方向的选择
    station_e = StationMark(2, 6, node_e.x, node_e.y, edges.getItem(6).e_facing);
    stations.updateItem(station_e);
    station_s = StationMark(1, 1, node_left.x, node_left.y, edges.getItem(1).e_facing);
    stations.updateItem(station_s);
    navigation_no_net = NavigationOnNet(stations, edges, nodes);
    paths = navigation_no_net.getNavigationPathOnNet(1, 2, getPose(station_s), Pose(), false, out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 5);
    ASSERT_EQ(paths.front().type_, PATH_LINE);
    ASSERT_EQ(paths.front().ey_, node_top.y);
    ASSERT_EQ(paths.at(1).type_, PATH_ROTATE);
    ASSERT_EQ(paths.back().type_, PATH_LINE);
    ASSERT_EQ(paths.back().ex(), node_e.x);
    ASSERT_TRUE(out_dst_pose == getPose(station_e));
//        show(navigation_no_net);
    station_s = StationMark(1, 3, node_left.x, node_left.y, edges.getItem(3).e_facing);
    stations.updateItem(station_s);
    navigation_no_net = NavigationOnNet(stations, edges, nodes);
    paths = navigation_no_net.getNavigationPathOnNet(1, 2, getPose(station_s), Pose(), false, out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 5);
    ASSERT_EQ(paths.front().type_, PATH_LINE);
    ASSERT_EQ(paths.front().ey_, node_bottom.y);
    ASSERT_EQ(paths.at(1).type_, PATH_ROTATE);
    ASSERT_EQ(paths.back().type_, PATH_LINE);
    ASSERT_EQ(paths.back().ex(), node_e.x);
    ASSERT_TRUE(out_dst_pose == getPose(station_e));
}

/**
 * 路径是否有货测试
 */
TEST_F(NavigationOnNetTest, LoadFree) {
    // 倒日字
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef node_left_top(1, -5, 5, 0);
    Nodef node_middle_top(2, 0, 5, 0);
    Nodef node_right_top(3, 5, 5, 0);
    Nodef node_left_bottom(4, -5, 0, 0);
    Nodef node_middle_bottom(5, 0, 0, 0);
    Nodef node_right_bottom(6, 5, 0, 0);
    nodes.addItem(node_left_top);
    nodes.addItem(node_middle_top);
    nodes.addItem(node_right_top);
    nodes.addItem(node_left_bottom);
    nodes.addItem(node_middle_bottom);
    nodes.addItem(node_right_bottom);
    //  ￣│
    edges.addItem(makeLine(1, node_middle_bottom, node_middle_top));
    edges.addItem(makeLine(2, node_middle_top, node_left_top));
    // │＿
    edges.addItem(makeLine(3, node_middle_bottom, node_left_bottom));
    edges.addItem(makeLine(4, node_left_bottom, node_left_top));
    // ￣]
    edges.addItem(makeLine(5, node_middle_bottom, node_right_bottom));
    edges.addItem(makeLine(6, node_right_bottom, node_right_top));
    edges.addItem(makeLine(7, node_right_top, node_middle_top));

    StationMark station_s(1, 0, node_middle_bottom.x + 0.05, node_middle_bottom.y + 0.05, edges.getItem(1).s_facing);
    stations.addItem(station_s);
    StationMark station_e(2, 0, node_left_top.x + 0.05, node_left_top.y + 0.05, edges.getItem(2).e_facing);
    stations.addItem(station_e);

    NavigationOnNet navigation_no_net(stations, edges, nodes);

    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, getPose(station_s)));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, getPose(station_e)));

    Pose out_dst_pose;
    auto paths =
        navigation_no_net.getNavigationPathOnNet(0, 0, getPose(station_s), getPose(station_e), true, out_dst_pose);
    printPaths(paths);
    //    show(navigation_no_net);
    ASSERT_EQ(paths.size(), 4);
    ASSERT_EQ(paths.front().type_, PATH_LINE);
    ASSERT_EQ(paths.front().ex(), node_middle_top.x);
    ASSERT_EQ(paths.front().ey(), node_middle_top.y);

    // 将中间路径的loadstate改为free
    auto edge_middle = edges.getItem(1);
    edge_middle.execute_type = EXECUTE_FREE;
    edges.updateItem(edge_middle);
    navigation_no_net = NavigationOnNet(stations, edges, nodes, LOAD_FULL);
    paths = navigation_no_net.getNavigationPathOnNet(0, 0, getPose(station_s), getPose(station_e), true, out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 5);
    ASSERT_EQ(paths.at(1).type_, PATH_LINE);
    ASSERT_EQ(paths.at(1).ex(), node_left_bottom.x);
    ASSERT_EQ(paths.at(1).ey(), node_left_bottom.y);
    //        show(navigation_no_net);

    // 将左边中间路径的loadstate改为free
    auto edge_left_middle = edges.getItem(4);
    edge_left_middle.execute_type = EXECUTE_FREE;
    edges.updateItem(edge_left_middle);
    navigation_no_net = NavigationOnNet(stations, edges, nodes, LOAD_FULL);
    paths = navigation_no_net.getNavigationPathOnNet(0, 0, getPose(station_s), getPose(station_e), true, out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 8);
    ASSERT_EQ(paths.at(1).type_, PATH_LINE);
    ASSERT_EQ(paths.at(1).ex(), node_right_bottom.x);
    ASSERT_EQ(paths.at(1).ey(), node_right_bottom.y);
    //        show(navigation_no_net);
}

/**
 * 测试一下站点在直线路径的中间
 */
TEST_F(NavigationOnNetTest, EndStationOnMiddleOfLine) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef n1(1, -2.59, -0.3, 0);
    Nodef n2(2, 0.97, -0.30, 0);
    Nodef n7(7, -3.75, -0.31, 0);
    Nodef n8(8, -3.75, -1.62, 0);
    nodes.addItem(n1);
    nodes.addItem(n2);
    nodes.addItem(n7);
    nodes.addItem(n8);
    edges.addItem(makeBezier(8, n8, n1, -3.75, -0.96, -3.17, -0.96));
    edges.addItem(makeLine(9, n1, n2, VehicleDirection::BACKWARD));
    edges.addItem(makeLine(7, n7, n8));
    StationMark station_e(5, 9, -0.9012, -0.30, 0);
    stations.addItem(station_e);
    StationMark station_s(4, 7, n7.x, n7.y, edges.getItem(7).s_facing);
    stations.addItem(station_s);

    NavigationOnNet navigation_no_net(stations, edges, nodes);
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, getPose(station_s)));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, getPose(station_e)));

    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(station_s.id, station_e.id, getPose(station_s), Pose(), true,
                                                          out_dst_pose);
    printPaths(paths);
    //    show(navigation_no_net);
    ASSERT_EQ(paths.size(), 6);
    ASSERT_EQ(paths.at(4).type_, PATH_LINE);
    ASSERT_EQ(paths.at(4).direction_, PATH_BACKWARD);
}

/**
 * 测试一下站点在直线路径的中间
 */
TEST_F(NavigationOnNetTest, StartStationOnMiddleOfLine) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef n1(1, -2.59, -0.3, 0);
    Nodef n2(2, 0.97, -0.30, 0);
    Nodef n7(7, -3.75, -0.31, 0);
    Nodef n8(8, -3.75, -1.62, 0);
    nodes.addItem(n1);
    nodes.addItem(n2);
    nodes.addItem(n7);
    nodes.addItem(n8);
    edges.addItem(makeBezier(8, n8, n1, -3.75, -0.96, -3.17, -0.96));
    edges.addItem(makeLine(9, n1, n2, VehicleDirection::BACKWARD));
    edges.addItem(makeLine(7, n7, n8, VehicleDirection::BACKWARD));
    StationMark station_e(5, 9, -0.9012, -0.30, 0);
    stations.addItem(station_e);
    StationMark station_s(4, 7, n7.x + 0.05, n7.y - 0.5, edges.getItem(7).s_facing + 0.234);
    stations.addItem(station_s);

    NavigationOnNet navigation_no_net(stations, edges, nodes);
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(4, getPose(station_s)));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(5, getPose(station_e)));

    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(station_s.id, station_e.id, getPose(station_s), Pose(), true,
                                                          out_dst_pose);
    printPaths(paths);
    //    show(navigation_no_net);
    ASSERT_EQ(paths.size(), 7);
    ASSERT_EQ(paths.at(1).type_, PATH_LINE);
    ASSERT_EQ(paths.at(1).direction_, PATH_BACKWARD);
}

/**
 * 测试单条直线路径，若起始站点和终点站点的方向和路径的方向刚好相反时，不允许导航
 */
TEST_F(NavigationOnNetTest, OneLinePathDirection) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef n1(1, -1, 0, 0);
    Nodef n2(2, 1, 0, 0);
    nodes.addItem(n1);
    nodes.addItem(n2);
    edges.addItem(makeLine(1, n1, n2, VehicleDirection::FORWARD));
    StationMark station_1(1, 1, -0.5, 0, 0);
    stations.addItem(station_1);
    StationMark station_2(2, 1, 0.5, 0, 0);
    stations.addItem(station_2);

    // 正常情况
    NavigationOnNet navigation_no_net(stations, edges, nodes);
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(1, getPose(station_1)));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(2, getPose(station_2)));

    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(station_1.id, station_2.id, getPose(station_1), Pose(), false,
                                                          out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 1);

    // 异常情况
    navigation_no_net = NavigationOnNet(stations, edges, nodes);
    paths = navigation_no_net.getNavigationPathOnNet(station_2.id, station_1.id, getPose(station_2), Pose(), false,
                                                     out_dst_pose);
    printPaths(paths);
    ASSERT_TRUE(paths.empty());

    // 添加一条后退路径，看是否好使
    edges.addItem(makeLine(2, n2, n1, VehicleDirection::BACKWARD));
    std::cout << 2 * M_PI << std::endl;
    navigation_no_net = NavigationOnNet(stations, edges, nodes);
    paths = navigation_no_net.getNavigationPathOnNet(station_2.id, station_1.id, getPose(station_2), Pose(), false,
                                                     out_dst_pose);
    printPaths(paths);
    ASSERT_FALSE(paths.empty());

    // 起始点和终点位置一致
    StationMark station_3(3, 1, -0.5, 0, 1.5);
    stations.addItem(station_3);
    navigation_no_net = NavigationOnNet(stations, edges, nodes);
    paths = navigation_no_net.getNavigationPathOnNet(station_1.id, station_3.id, getPose(station_1), Pose(), false,
                                                     out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 1);
}

/**
 * 测试单条贝塞尔路径，若起始站点和终点站点的方向和路径的方向刚好相反时，不允许导航
 */
TEST_F(NavigationOnNetTest, OneBezierPathDirection) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    Nodef n1(1, -1, 0, 0);
    Nodef n2(2, 1, 0, 0);
    nodes.addItem(n1);
    nodes.addItem(n2);
    edges.addItem(makeBezier(1, n1, n2, -1.0, 1.0, 1.0, -1.0));
    StationMark station_1(1, 1, -0.83, 0.27, 0);
    stations.addItem(station_1);
    StationMark station_2(2, 1, 0.67, -0.28, 0);
    stations.addItem(station_2);

    // 正常情况
    NavigationOnNet navigation_no_net(stations, edges, nodes);
    //    show(navigation_no_net);
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(1, getPose(station_1)));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(2, getPose(station_2)));

    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(station_1.id, station_2.id, getPose(station_1), Pose(), false,
                                                          out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 3);

    // 异常情况
    navigation_no_net = NavigationOnNet(stations, edges, nodes);
    paths = navigation_no_net.getNavigationPathOnNet(station_2.id, station_1.id, getPose(station_2), Pose(), false,
                                                     out_dst_pose);
    printPaths(paths);
    ASSERT_TRUE(paths.empty());
}

/**
 * 测试车辆处于两条路径附近时如何选择回归路径
 */
TEST_F(NavigationOnNetTest, LinePathReturn) {
    StationMarkGroup stations;
    EdgeGroup edges;
    NodeGroup nodes;
    // __|
    Nodef n1(1, -1, 0, 0);
    Nodef n2(2, 1, 0, 0);
    Nodef n3(3, 1, 1, 0);
    nodes.addItem(n1);
    nodes.addItem(n2);
    nodes.addItem(n3);
    edges.addItem(makeLine(1, n1, n2, VehicleDirection::FORWARD));
    edges.addItem(makeLine(2, n2, n1, VehicleDirection::FORWARD));
    edges.addItem(makeLine(3, n3, n2, VehicleDirection::FORWARD));
    StationMark station_1(1, 1, -0.5, 0.1, 0);
    stations.addItem(station_1);
    StationMark station_2(2, 3, 1, 0, 0);
    stations.addItem(station_2);

    // 正常情况
    NavigationOnNet navigation_no_net(stations, edges, nodes);
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(0, getPose(station_1)));
    ASSERT_TRUE(navigation_no_net.checkIsOnNet(2, getPose(station_2)));

    Pose out_dst_pose;
    auto paths = navigation_no_net.getNavigationPathOnNet(0, station_2.id, getPose(station_1), Pose(), false,
                                                          out_dst_pose);
    printPaths(paths);
    ASSERT_EQ(paths.size(), 3);
}