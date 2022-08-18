/*
 * MyThread.cpp
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#include "station_recognition_module.h"

#include "core/src.h"
#include "core/msg/laser_scan_msg.hpp"
#include "core/msg/parameter_msg.hpp"
#include "core/msg/common_command_msg.hpp"
#include "core/settings.h"

#include "recog_processor/recog_processor_factory.h"

using namespace std;

using namespace sros::core;

namespace sros {
void normalizeAngle(double &angle) {
    angle = fmod(angle, 2.0 * M_PI);
    if (angle >= M_PI) {
        angle -= 2.0f * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
}
StationRecognitionModule::StationRecognitionModule() :
        Module("StationRecognition"),
        find_state_(FIND_NONE),
        find_cnt_(0),
        param_average_cnt_(1),
        param_station_distance_threshold_(4.0) {
        recog_type = recog::TYPE_RECOGPRO_CHARGINGPILE;
        recog_para = recog::RecogProcessorFactory::getRecogParam(recog_type);

}

StationRecognitionModule::~StationRecognitionModule() {

}

void StationRecognitionModule::run() {
    auto &s = sros::core::Settings::getInstance();

    enable_station_recognation_ = (s.getValue<string>("main.enable_station_recognation", "False") == "True");

    if (!enable_station_recognation_) {
        LOG(WARNING) << "StationRecognitionModule module stop running(disable)";
        stop();
        return;
    }
    LOG(INFO) << "StationRecognition module start running";

    subscribeTopic("FEATURE_RECOG_COMMAND", CALLBACK(&StationRecognitionModule::onNavigationCommandMsg));

    subscribeTopic("TOPIC_LASER", CALLBACK(&StationRecognitionModule::onLaserScanMsg));

    subscribeTopic("POSEFILTER_PARAMETER", CALLBACK(&StationRecognitionModule::onParameterMsg));
    subscribeTopic("STATION_RECOG_PARAMETER", CALLBACK(&StationRecognitionModule::onParameterMsg));

    dispatch();
}


void StationRecognitionModule::onNavigationCommandMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<CommonCommandMsg<NavigationCommand>>(m);
    //msg->param1 1->angle 2->hack lmk
    switch (msg->command) {
        case COMMAND_NAV_FIND_FEATURE: {
            LOG(INFO) << "COMMAND_NAV_FIND_FEATURE";

            find_state_ = FIND_WAITING_SCAN;
            cur_finding_pose_ = msg->pose;
            if (msg->param1 == FIND_ANGLE_CMD) {
                recog_type = recog::TYPE_RECOGPRO_ANGLE;
            }else if(msg->param1 == FIND_LMKRACK_CMD){
                recog_type = recog::TYPE_RECOGPRO_LMKRACK;
            }else if(msg->param1 = FIND_CHARGINGPILE_CMD){
                recog_type = recog::TYPE_RECOGPRO_CHARGINGPILE;
            }
            break;
        }
        default:
            break;
    }
}

void StationRecognitionModule::onLaserScanMsg(sros::core::base_msg_ptr m) {
    auto scan_ptr = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(m);

    if (find_state_ != FIND_WAITING_SCAN) {
        return;
    }

    LOG(INFO) << "正在检测站点位置..." << find_cnt_;

    if (find_cnt_ < param_average_cnt_) {
        Pose pose;
        if (findStationPose(scan_ptr, pose)) {
            LOG(ERROR) << "  > 检测到站点位置: " << pose.x() << ", " << pose.y() << ", " << pose.yaw();
            station_pose_list_.push_back(pose);
        } else {
            LOG(INFO) << "  > 未检测到站点";
        }

        find_cnt_ += 1;
    } else {
        find_state_ = FIND_NONE;
        find_cnt_ = 0;

        // 返回站点识别结果
        auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("FEATURE_RECOG_RESULT");
        mm->command = COMMAND_NAV_FIND_FEATURE_RESULT;

        //TODO:需要添加一个para或者type,来确定当前识别的特征是什么特征

        // 获取param_average_cnt_次检测的平均值
        mm->param0 = getFindAverageResult(mm->pose);

        sendMsg(mm);

        // 清除list
        station_pose_list_.clear();
    }

}

bool StationRecognitionModule::findStationPose(sros::core::LaserScan_ptr scan_ptr, Pose& result_pose) {
    // 求激光雷达位置坐标(src_sdk->getCurPose())
    // 使用param_laser_coordx_, param_laser_coordy_, param_laser_coordyaw_

    Pose laser_pose;
    Pose p = src_sdk->getCurPose();//当前位姿，应该是比cur_finding_pose更靠谱
//    Pose& p = cur_finding_pose_;
    laser_pose.x() = param_laser_coordx * cos(p.yaw()) - param_laser_coordy * sin(p.yaw()) + p.x();
    laser_pose.y() = param_laser_coordx * sin(p.yaw()) + param_laser_coordy * cos(p.yaw()) + p.y();
    laser_pose.yaw() = param_laser_coordyaw + p.yaw();

    // 设置待识别的特征

//     = recog::TYPE_RECOGPRO_ANGLE;//TODO:修改该部分即可完成切换

    feature_recog_ = recog::RecogProcessorFactory::getRecogProcessor(recog_type);
    recog_para = recog::RecogProcessorFactory::getRecogParam(recog_type);

    feature_recog_->setRecogPara(recog_para);

    // TODO 测试 获取所有识别到的feature
    Eigen::Vector3f center_pose(laser_pose.x(), laser_pose.y(), laser_pose.yaw());

     auto pose_infos = feature_recog_->getRecogInfo(scan_ptr, center_pose);
    // 构造结果
//    Pose_Vector pose_list;
//    pose_list.push_back(Pose(Location(4.2, 8), Rotation(M_PI + M_PI_2)));

    const double MAX_DISTANCE = 1000000;
    double cur_min_distance = MAX_DISTANCE; // 当前pose_list中的最小距离值
    if (pose_infos) {
        auto &pose_list = pose_infos->recog_infos;
        // 找到距离cur_finding_pose最近的Pose
        for (auto pose : pose_list) {

            sros::core::Pose sros_pose;
            sros_pose.x() = pose.pose[0];
            sros_pose.y() = pose.pose[1];
            sros_pose.yaw() = pose.pose[2];
            double distance = getDistance(sros_pose, cur_finding_pose_);
            LOG(INFO) << "distance is:" << distance;
            LOG(INFO) << "rack pose is:" << sros_pose.x() << "," << sros_pose.y() << "," << sros_pose.yaw();
            if (distance < cur_min_distance) {
                cur_min_distance = distance;
                result_pose = sros_pose;
            }
        }
    }


    // 如果最小距离小于阈值,则任务成功找到站点
    return (cur_min_distance != MAX_DISTANCE && cur_min_distance < param_station_distance_threshold_);
}

double StationRecognitionModule::getDistance(Pose &p1, Pose &p2) const {
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return sqrt(dx * dx + dy * dy);
}

void StationRecognitionModule::onParameterMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<ParameterMsg>(m);
    string name = msg->name;
    string value = msg->value;
    DLOG(INFO) << "StationRecognitionModule::onParameterMsg -> " << name << ": " << value;

    if (name == "station_recog.feature_theta") {
        param_feature_theta_ = atof(value.c_str());
        recog_para->angle_feat_theta = atof(value.c_str());
    } else if (name == "station_recog.feature_d1") {
        param_feature_d1_ = atof(value.c_str());
        recog_para->angle_feat_dist_1 = atof(value.c_str());
    } else if (name == "station_recog.feature_d2") {
        param_feature_d2_ = atof(value.c_str());
        recog_para->angle_feat_dist_2 = atof(value.c_str());
    } else if (name == "station_recog.distance_threshold") {
            param_station_distance_threshold_ = atof(value.c_str());
    } else if (name == "station_recog.average_cnt") {
            param_average_cnt_ = atoi(value.c_str());
    } else if (name == "posefilter.laser_coordx") {
        param_laser_coordx = atof(value.c_str());
    } else if (name == "posefilter.laser_coordy") {
        param_laser_coordy = atof(value.c_str());
    } else if (name == "posefilter.laser_coordyaw") {
        param_laser_coordyaw = atof(value.c_str());
    }
//    if(recog_type == recog::TYPE_RECOGPRO_LMKRACK){
//        std::shared_ptr<recog::RecogRackPara> recog_rack_para = std::dynamic_pointer_cast<recog::RecogRackPara>(recog_para);
//        if(name == "station_recog.rack_length") {
//            recog_rack_para->rack_length = atof(value.c_str());
//        }else if(name == "station_recog.rack_width"){
//            recog_rack_para->rack_width = atof(value.c_str());
//        }
//    }else if(recog_type == recog::TYPE_RECOGPRO_CHARGINGPILE) {
//        std::shared_ptr<recog::RecogChargingPilePara> recog_charging_pile_para = std::dynamic_pointer_cast<recog::RecogChargingPilePara>(recog_para);
//        if (name == "station.reflective_stickers_length"){
//            recog_charging_pile_para->reflective_stickers_length = atof(value.c_str());
//        }
//    }
}

bool StationRecognitionModule::getFindAverageResult(sros::core::Pose &out_pose) {
    // 获取station_pose_list_中的Pose平均值
    //在M_PI附近会出问题
    Pose p;
    auto size = station_pose_list_.size();

    if(!size)
        return  false;
    double base_angle = station_pose_list_[0].yaw();
    for (auto pose : station_pose_list_) {
        p.x() += pose.x();
        p.y() += pose.y();
        double delta_angle = base_angle - pose.yaw();
        normalizeAngle(delta_angle);
        p.yaw() += delta_angle;

    }

    out_pose.x() = p.x() / size;
    out_pose.y() = p.y() / size;
    out_pose.yaw() = base_angle - p.yaw() / size;

    return true;
}

} /* namespace sros */
