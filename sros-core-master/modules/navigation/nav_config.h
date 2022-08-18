//
// Created by yj on 20-3-25.
//
// nav中所有需要在界面配置的参数都需要在此处。不用配置的且独一余单个模块的可以不用。
#ifndef SROS_NAV_CONFIG_H
#define SROS_NAV_CONFIG_H

#include "core/settings.h"
#include "core/rack_para.h"
#include "core/rack/rack_operator_instance.hpp"

class NavConfig{

public:
    struct Robot{
        float length;  // AGV 车体的长度 单位m。

        float width; // AGV车体的宽度单位m。

        float max_vel_x ;  //允许的最大的移动速度 单位m/s


        float max_vel_x_backwards ;//允许的最大的后退移动速度 单位m/s


        float max_vel_theta ;//允许的最大的移动角速度 单位rad/s


        float acc_lim_x ; // 允许的最大的移动线加速度 单位 m/s2.

        float acc_lim_theta ; //允许的最大的移动角加速度单位 rad/s2.

        float rotate_max_w;  // 原地旋转最大的角速度。

        float rotate_max_acc_w; // 原地旋转最大的角加速度。

    }robot;



    struct MapParam{

    }map_param;


    struct HybridPlan{

        float penaltyTurning ; // 转弯的损失系数。比如从左转变为执行。

        float penaltyReversing ; // 后退的损失系数。尽量减少后退。

        float penaltyCOD ; // 从前进变为后退的损失系数。 避免反复前进后退。


    }hybrid_plan;

    struct SmootherParam{

        float minRoadWidth ; // 由于1格表示0.02m 所以如果需要0.4m则需要20格。

        float kappaMax ;
        /// maximum distance to obstacles that is penalized
        float obsDMax ; // 距离障碍物要求的最小距离
        /// maximum distance for obstacles to influence the voronoi field
        float vorObsDMax ;
        /// falloff rate for the voronoi field
        float alpha ;
        /// weight for the obstacle terma
        float wObstacle ;  // 离障碍物距离的权重。

        /// weight for the curvature term
        float wCurvature ; // 曲率参数的权重。
        /// weight for the smoothness term
        float wSmoothness ;  // 平滑度的权重。

    }smoother_param;


    struct TebParam{

        float min_obstacle_dist ; //要求的局部路径离障碍物最近的距离单位m。

        float inflation_dist ; // 要求的局部路径离障碍物尽可能远的缓存距离 单位m。

        float weight_max_vel_x ;// teb图优化权重，最大速度。

        float weight_max_vel_theta; // teb图优化权重，最大角速度。

        float weight_acc_lim_x ;// teb图优化权重，线加速度。

        float weight_acc_lim_theta ;//teb图优化权重，角角速度

        float weight_kinematics_nh ;// teb图优化权重，底盘运动模型约束。

        float weight_kinematics_forward_drive ;// teb图优化权重，向前运动约束。

        float weight_optimaltime ;// teb图优化权重，运行时间。

        float weight_obstacle ;//teb图优化权重，远离障碍物

        float weight_inflation ;//teb图优化权重，远离缓存距离。

    }teb_param;

    //自由导航时，遇到障碍物暂停。多久后才开始重新规划单位s
    float replan_wait_time_s;
    float replan_dist_error;

    float map_expansion_size=0; // 地图膨胀尺寸。单位m。

     float rack_length = 0;
     float rack_width = 0;
    // 初始化参数。


public:

    NavConfig(){
        // robot
        robot.width = 0.65;
        robot.length = 0.75;

        nav_resolution = 4;

        //hybrid
        hybrid_plan.penaltyTurning = 1.05;
        hybrid_plan.penaltyReversing = 3.55;
        hybrid_plan.penaltyCOD = 4.8;

        // smoother
        smoother_param.minRoadWidth = 30;
        smoother_param.obsDMax = smoother_param.minRoadWidth;

        //teb
        teb_param.min_obstacle_dist = robot.length/2.0f+0.1;
        teb_param.inflation_dist=0.52;


        enable_local_planner = false;

        enable_backward = true;
        replan_dist_error = 5.0;

    }

    void updateNavParam(){
        auto &s = sros::core::Settings::getInstance();

        enable_local_planner = s.getValue<std::string>("nav.enable_free_nav", "False") == "True";
        LOG(INFO) << "nav:是否使能自由导航功能。enable_free_nav: " << enable_local_planner;

        enable_backward = s.getValue<std::string>("nav.enable_backward", "False") == "True";
        LOG(INFO) << "nav:是否开启路径后退功能enable_backward："<<enable_backward;

        nav_resolution = s.getValue<double>("nav.nav_resolution", 4);

        if(nav_resolution<1.5||nav_resolution>10){
            nav_resolution=2;
        }
        // robot。
        robot.length = s.getValue<double>("nav.vehicle_length", 0.8);
        LOG(INFO) << "nav:AGV的长度单位：m.                                   ：" << robot.length;

        robot.width = s.getValue<double>("nav.vehicle_width", 0.6);
        LOG(INFO) << "nav:AGV的宽度单位：m.                                   ：" << robot.width;

        robot.max_vel_x = s.getValue<double>("nav.robot_max_vel_x", 0.6);
        LOG(INFO) << "nav:允许的最大的移动速度robot.max_vel_x                   :" << robot.max_vel_x;

        robot.max_vel_x_backwards = s.getValue<double>("nav.max_vel_x_backwards", 0.3);
        LOG(INFO) << "nav:允许的最大的后退移动速度robot.max_vel_x_backwards      :" << robot.max_vel_x_backwards;

        robot.max_vel_theta = s.getValue<double>("nav.robot_max_vel_theta", 0.5);
        LOG(INFO) << "nav:自由导航最大移动角速度      :" << robot.max_vel_theta;

        robot.acc_lim_x = s.getValue<double>("nav.robot_acc_lim_x", 0.6);
        LOG(INFO) << "nav:允许的最大的线加速度robot.robot_acc_lim_x      :" << robot.acc_lim_x;

        robot.acc_lim_theta = s.getValue<double>("nav.robot_acc_lim_theta", 0.6);
        LOG(INFO) << "nav:允许的最大的角加速度robot.robot_acc_lim_theta      :" << robot.acc_lim_theta;

        aisle_margin = s.getValue<double>("nav.aisle_margin", 0.0);
        LOG(INFO) << "nav:通道余量的值 aisle_margin:" << aisle_margin;


        replan_dist_error = s.getValue<double>("nav.replan_dist_error", 5.0);
        LOG(INFO) << "nav:重规划距离差replan_dist_error:" << replan_dist_error;


        replan_wait_time_s = s.getValue<double>("nav.replan_wait_time_s", 0.0);
        LOG(INFO) << "nav:重规划等待时间replan_wait_time_s:" << replan_wait_time_s;


         rack_length = sros::core::Settings::getInstance().getValue("rack.max_contour_length", 1200) * 0.001;
         rack_width = sros::core::Settings::getInstance().getValue("rack.max_contour_width", 600) * 0.001;
        auto rack_op =rack::RackOperatorInstance::getInstance();
        if (rack_op) {
            auto rack_info = rack_op->getRackInfo();
            if (rack_info) {
                rack_length = rack_info->avd_oba_length;
                rack_width = rack_info->avd_oba_width;
            }
        }
        LOG(INFO)<<"货架尺寸长："<<rack_length<<";宽："<<rack_width;

        // robot.length  =std::max(robot.length,rack_length);
        // robot.width = std::max(robot.width,rack_width);

        // teb优化局部路径 离障碍物的最小距离。
        teb_param.min_obstacle_dist = std::max(robot.length,robot.width)/2.0f+0.1+aisle_margin;

        smoother_param.minRoadWidth = (std::max(robot.length,robot.width)/2.0f+aisle_margin)*100.0/nav_resolution;
        // 地图膨胀尺寸。目前地图膨胀都是按照 用到膨胀的 a star  和hybrid两个。a star 的膨胀要稍微大于hybrid的膨胀一个栅格。保证hybrid能够顺利通过窄通道。

        // smoother和teb采用不膨胀的原始地图，用于其中比较重要的参数就是 优化的离散点离障碍物的最小距离L。 该距离应该大于等于膨胀尺寸。用于保证agv的安全(大多少合适，我觉得
        // 不压到人的脚为合适。)。L = 车长一半+脚长（0.26m）。

        //检测障碍物 采用原始地图和没有膨胀的实时障碍物。通过计算agv长方形内部是否有障碍物来判断。 障碍物检测都是基于路径来判断的。所以需要保证路径的连续，平滑，曲率连续等要求。
        // 一般需要检测障碍物的矩形要比车体大一点用来保证安全。
        // 为了保证agv前方被全部挡着时还可以继续后退。在检测前方障碍物时。从当前点前方一段小距离开始检测障碍物。


        // 不允许后退情况。 如果么有安装后退雷达不允许后退的化。则不需要采用hybrid 算法了。 直接采用原地旋转+导航即可。如果遇到无法原地旋转的则直接暂停报错即可。

        //





//        // hybrid
//        hybrid_plan.penaltyTurning = s.getValue<double>("nav.hybrid_plan_penaltyTurning", 0.6);
//        LOG(INFO) << "nav:转弯的损失系数,比如从左转变为直行。hybrid_plan_penaltyTurning :" << hybrid_plan.penaltyTurning;
//
//        hybrid_plan.penaltyReversing = s.getValue<double>("nav.hybrid_plan_penaltyTurning", 0.6);
//        LOG(INFO) << "nav:后退的损失系数。尽量减少后退。hybrid_plan_penaltyTurning :" << hybrid_plan.penaltyReversing;
//
//        hybrid_plan.penaltyCOD = s.getValue<double>("nav.hybrid_plan_penaltyTurning", 0.6);
//        LOG(INFO) << "nav:从前进变为后退的损失系数。hybrid_plan_penaltyCOD :" << hybrid_plan.penaltyCOD;
//
//        // smoother
//        smoother_param.obsDMax = s.getValue<double>("nav.hybrid_plan_penaltyTurning", 0.6);
//        LOG(INFO) << "nav:smoother中需要远离障碍物的安全距离。smoother_param.obsDMax :" << smoother_param.obsDMax;
//
//        smoother_param.vorObsDMax = s.getValue<double>("nav.hybrid_plan_penaltyTurning", 0.6);
//        LOG(INFO) << "nav:smoother中需要远离障碍物的缓冲距离。smoother_param.vorObsDMax :" << smoother_param.vorObsDMax;

//        //teb
//        teb_param.min_obstacle_dist = s.getValue<double>("nav.min_obstacle_dist", 0.6);
//        LOG(INFO) << "nav:需要远离障碍物的安全距离。smoother_param.obsDMax :" << teb_param.min_obstacle_dist;
//
//        teb_param.inflation_dist = s.getValue<double>("nav.hybrid_plan_penaltyTurning", 0.6);
//        LOG(INFO) << "nav:需要远离障碍物的缓冲距离。smoother_param.vorObsDMax :" << teb_param.inflation_dist ;

    }



    bool enable_local_planner ;
    //LOG(INFO) << "local_planner:是否使能主动绕障功能。open_local_planner: " << enable_local_planner;

   // float open_local_planner ;
    //LOG(INFO) << "local_planner:是否允许开启局部路径规划open_local_planner: " << open_local_planner;

    float enable_loadfull_localplanner ;
    //LOG(INFO) << "local_planner:是否开启载货时区域绕障功能: " << enable_loadfull_localplanner;
    float aisle_margin = 0;

    float nav_resolution; // 栅格地图的分辨率，表示一格代表几个cm。

    bool enable_backward; // 是否开启后退功能，一般情况下只有包含后退的激光雷达时，才会开启后退移动功能。否则会撞到实时障碍物。





};

#endif //SROS_NAV_CONFIG_H
