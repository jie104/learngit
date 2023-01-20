//
// Created by zxj on 2023/1/4.
//

//
// Created by zxj on 1/3/23.
//

#include "action_270.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
using namespace std;
using namespace sros::core;

namespace ac {

    void Action270::doStart() {
        LOG(INFO) << "begin to calib obstacle point cloud!!!";

        auto &s = sros::core::Settings::getInstance();
        //获取主雷达安装参数
        auto laser_coord_x = s.getValue("posefilter.laser_coordx", 0.920);
        auto laser_coord_y = s.getValue("posefilter.laser_coordy", 0.0);
        auto laser_coord_yaw = s.getValue("posefilter.laser_coordyaw", -0.0196);

        //获取左避障雷达安装参数
        auto ust_left_coord_x = s.getValue("obstacle.ust_left_coord_x", 1.0);
        auto ust_left_coord_y = s.getValue("obstacle.ust_left_coord_y", 0.425);
        auto ust_left_coord_yaw = s.getValue("obstacle.ust_left_coord_yaw", 0.270);

        //获取右避障雷达安装参数
        auto ust_right_coord_x = s.getValue("obstacle.ust_right_coord_x", 1.0);
        auto ust_right_coord_y = s.getValue("obstacle.ust_right_coord_y", -0.425);
        auto ust_right_coord_yaw = s.getValue("obstacle.ust_right_coord_yaw", -0.270);

        subscribeTopic("NAV_COMMAND", CALLBACK(&NavigationModule::onCommandMsg2));


    }



    Eigen::Isometry2f Action270::CoordinateTf(const Eigen::Vector3f& src_point){
        Eigen::Matrix2f R;
        float theta=src_point[2];
        R << cos(theta),-sin(theta),
                sin(theta),cos(theta);
        Eigen::Vector2f t=Eigen::Vector2f(src_point[0],src_point[1]);
        Eigen::Isometry2f T=Eigen::Isometry2f::Identity();
        T.rotate(R);
        T.pretranslate(t);

        return T;
    }


}