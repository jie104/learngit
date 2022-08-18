//
// Created by caoyan on 4/22/21.
//

#include "path_utils.h"
#include "core/util/utils.h"
#include "core/state.h"
#include "core/settings.h"

using namespace std;
using namespace sros::core;

namespace ac {

void genRotateBetweenPaths(sros::core::NavigationPath_vector& dst_paths) {
    NavigationPath_vector ps;

    if (dst_paths.size() < 2) {
        return;
    }

    auto& s = sros::core::Settings::getInstance();
    auto rotate_between_path_threshold = s.getValue<double>("nav.rotate_between_path_threshold", 10) / 10.0 * DEGREE_TO_RAD;

    auto path_it = dst_paths.begin();
    NavigationPath<double> p = *path_it;
    p.avoid_policy_ = OBSTACLE_AVOID_WAIT;  // 路网上的避障策略始终为WAIT
    ps.push_back(p);

    for (int i = 0; i < dst_paths.size() - 1; i++) {
        NavigationPath<double> path1 = *path_it;

        path_it++;
        NavigationPath<double> path2 = *path_it;

        LOG(INFO) << "path type: " << path1.type_ << " path1.s_facing_: " << std::to_string(path1.s_facing_)
                  << " path1.e_facing_: " << std::to_string(path1.e_facing_);
        LOG(INFO) << "path type: " << path2.type_ << " path2.s_facing_: " << std::to_string(path2.s_facing_)
                  << " path2.e_facing_: " << std::to_string(path2.e_facing_);

        double theta = minRotate(path1.e_facing_, path2.s_facing_);
        LOG(INFO) << "theta: " << theta << " angle1: " << path1.e_facing_ << " angle2: " << path2.s_facing_;

        if (fabs(theta) > rotate_between_path_threshold) {  // 如果theta大于一个阈值, 则插入原地旋转路径
            ps.push_back(RotatePath(path1.ex_, path1.ey_, path2.s_facing_));
            LOG(INFO) << "Add " << ps.back();
        }

        path2.avoid_policy_ = OBSTACLE_AVOID_WAIT;
        ps.push_back(path2);
    }

    dst_paths = std::move(ps);
}

void genStartRotatePath(sros::core::NavigationPath_vector& dst_paths, double srt_pose_yaw) {
    if (dst_paths.empty()) {
        return;
    }

    auto& s = sros::core::Settings::getInstance();
    auto start_pose_rotate_threshold = s.getValue<double>("nav.start_pose_rotate_threshold", 10) / 10.0 * DEGREE_TO_RAD;

    auto first_path = dst_paths.begin();

    double theta = minRotate(srt_pose_yaw, first_path->s_facing_);
    LOG(INFO) << "rotate_value: " << theta;
    if (theta >= start_pose_rotate_threshold) {
        // 如果旋转角度大于起点旋转阈值（默认为1°）, 则加入原地旋转
        auto curr_pose = src_sdk->getCurPose();
        dst_paths.insert(dst_paths.begin(),
                         RotatePath(curr_pose.x(), curr_pose.y(), first_path->s_facing_));
        LOG(INFO) << "Add start pose " << dst_paths.front();
    }
}

void genEndRotatePath(sros::core::NavigationPath_vector& dst_paths, double dst_pose_yaw) {
    //处理最后一条路径是旋转路径
    //获取当前路径中的最后一条的方向

    auto& s = sros::core::Settings::getInstance();
    auto end_pose_rotate_threshold = s.getValue<double>("nav.end_pose_rotate_threshold", 1) / 10.0 * DEGREE_TO_RAD;

    NavigationPath<double> cur_path_last = dst_paths.back();  //保证这个变量名是首次被使用

    // 此处与起始点处相反, 需要获取edge相对于站点朝向的旋转角度
    double delta = minRotate(cur_path_last.e_facing_, dst_pose_yaw);

    // LOG(INFO) << "EndRotate: [delta: " << delta 
    //           << "][e_facing_: " << cur_path_last.e_facing_ 
    //           << "][dst_pose_yaw:" << dst_pose_yaw << "][" << M_PI / 1800 << "]";

    if (fabs(delta) >= end_pose_rotate_threshold) {
        // 如果旋转角度大于0.1度, 则加入原地旋转,或者强制加入旋转，来矫正src行走过程中的偏差
        dst_paths.push_back(RotatePath(cur_path_last.ex(), cur_path_last.ey(), dst_pose_yaw));
        LOG(INFO) << "Add end rotate " << dst_paths.back();
    }
}

void string_split(string s, string delim, std::vector<string>& ans) {
    string::size_type pos_1,pos_2=0;
    while(pos_2!=s.npos){
        pos_1=s.find_first_not_of(delim,pos_2);
        if(pos_1==s.npos) break;
        pos_2=s.find_first_of(delim,pos_1);
        ans.push_back(s.substr(pos_1,pos_2-pos_1));
    }
}

void debugOuputPaths(const sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "debugOuputPaths生成的路径 :" << dst_paths.size();
    for (auto p : dst_paths) {
        LOG(INFO) << "-> 路径"
                  << ": 类型" << p.type_ << "(sx,sy)= (" << p.sx_ << "," << p.sy_ << "); (ex,ey)= (" << p.ex_ << ","
                  << p.ey_ << ") rotate_angle = " << p.rotate_angle_;
        LOG(INFO) << "-> 方向"
                  << ": " << p.direction_;
    }
}

double calcForkEndCoordinateLength() {
    auto& s = sros::core::Settings::getInstance();
    string fork_end_coordinate_str = s.getValue<string>("forklift.fork_end_coordinate", "1;0;0.1;");

    std::vector<string> vec;
    string_split(fork_end_coordinate_str, ";", vec);

    double fork_end_coordinate_x = std::stod(vec[0]);

    return fork_end_coordinate_x;
}

double calcAdjustPoseOptimalDistance() {

    auto& s = sros::core::Settings::getInstance();
    double fork_arm_length = s.getValue<double>("forklift.fork_arm_length", 1.0);

    string fork_end_coordinate_str = s.getValue<string>("forklift.fork_end_coordinate", "1;0;0.1;");

    std::vector<string> vec;
    string_split(fork_end_coordinate_str, ";", vec);

    double fork_end_coordinate_x = std::stod(vec[0]);

    //添加计算调整量
    double infix_adjust_length = s.getValue<double>("forklift.infix_adjust_length", 0.1);

    return (fork_arm_length - fork_end_coordinate_x + infix_adjust_length); //多加10cm

}

double calcDetectDistance() {

    auto& s = sros::core::Settings::getInstance();
    double fork_arm_length = s.getValue<double>("forklift.fork_arm_length", 1.0);

    string fork_end_coordinate_str = s.getValue<string>("forklift.fork_end_coordinate", "1;0;0.1;");

    std::vector<string> vec;
    string_split(fork_end_coordinate_str, ";", vec);

    double fork_end_coordinate_x = std::stod(vec[0]);

    //添加计算调整量
    double detect_distance = s.getValue<double>("forklift.detect_distance", 0.1);

    return (fork_arm_length - fork_end_coordinate_x + detect_distance); 

}

bool isForkPoseAdjust() {
    auto& s = sros::core::Settings::getInstance();
    bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
    if(!enable_load_detect) {
        return false;
    }

    if(!g_state.is_fork_pose_adjust) {
        return false;
    }

    return true;
}

double calcGoodsLength(int goal_id) {
    auto& s = sros::core::Settings::getInstance();
    auto fork_goods_type = s.getValue<string>("perception.detect_goods_type", "CIRCLE");

    double goods_length;
    string goods_info_str;

    if(fork_goods_type == "CARD") {
        //卡板检测
        if(goal_id == 1) {
            goods_info_str = s.getValue<string>("perception.card_first_info", "0;1.2;0.11;0.1;0.295,0.295;0.07,0.07,0.07;");
        } else if(goal_id == 2) {
            goods_info_str = s.getValue<string>("perception.card_second_info", "1;1.2;0.11;0.1;0.355,0.355;0.04,0.04,0.04;");
        } else if(goal_id == 3) {
            goods_info_str = s.getValue<string>("perception.card_third_info", "2;1.2;0.14;0.2;0.27,0.27;0.07,0.10,0.07;");
        } else if(goal_id == 4) {
            goods_info_str = s.getValue<string>("perception.card_fourth_info", "3;1.2;0.15;0.12;0.3,0.3;0.11,0.13,0.11;");
        } else if(goal_id == 5) {
            goods_info_str = s.getValue<string>("perception.card_fifth_info", "4;1.2;0.14;0.12;0.35,0.35;0.16,0.18,0.16;");
        } else {
            goods_info_str = s.getValue<string>("perception.card_sixth_info", "5;1.2;0.15;0.08;0.3,0.3;0.16,0.30,0.16");
        }

    } else if(fork_goods_type == "CIRCLE"){
        //圆盘检测
        goods_info_str = s.getValue<string>("perception.circle_disk_info", "0;0.8;0.5;");
    } else {
        //手推车
        goods_info_str = s.getValue<string>("perception.handcart_info", "1;0.6;0.04;0.65");

    }

    std::vector<string> vec;
    string_split(goods_info_str, ";", vec);

    goods_length = std::stod(vec[1]);
    return goods_length;
}

void calcDstPose(const Pose& pose_src, Pose& pose_dst, double vec) {
    pose_dst.x() = pose_src.x() + cos(pose_src.yaw()) * vec;
    pose_dst.y() = pose_src.y() + sin(pose_src.yaw()) * vec;
    pose_dst.yaw() = pose_src.yaw();
}

BezierPath genBezierPath(const Pose& pose_src, const Pose& pose_dst, PathDirection dir) {
    double d = hypot(pose_src.y() - pose_dst.y(), pose_src.x() - pose_dst.x());
    double l = d / 2;
    l = l > 2 ? 2 : l;

    Pose src_to_dst(pose_dst.x() - pose_src.x(), pose_dst.y() - pose_src.y(), 0);
    Pose pose_src_vector(cos(pose_src.yaw()), sin(pose_src.yaw()), 0);
    if(src_to_dst.x() * pose_src_vector.x() + src_to_dst.y() * pose_src_vector.y() < 0) {
        l = -l; //大于90度为-l，
    }

    Pose point1(pose_src.x() + l * cos(pose_src.yaw()), pose_src.y() + l * sin(pose_src.yaw()), 0);
    Pose point2(pose_dst.x() - l * cos(pose_dst.yaw()), pose_dst.y() - l * sin(pose_dst.yaw()), 0);

    return BezierPath(pose_src.x(), pose_src.y(), point1.x(), point1.y(), point2.x(), point2.y(),
                      pose_dst.x(), pose_dst.y(), 0, dir);

}

double distanceTwoPose (const Pose &p1,const Pose &p2) { 
    return sqrt((p1.x()- p2.x()) * (p1.x()- p2.x()) + (p1.y()- p2.y()) * (p1.y()- p2.y()));
};


}