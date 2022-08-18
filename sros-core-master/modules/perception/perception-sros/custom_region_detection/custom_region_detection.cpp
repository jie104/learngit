/**
 * @file custom_region_detector_module.cpp
 * @brief Module for custom area detection.
 *
 * It is used to process whether there are obstacles in the defined detection area, subscribe
 * PerceptionCommandMsg topic messages, and publish detection results to ELECTRODES_DETECT_RESULT
 * topics. The configuration parameters of user-defined area detection are in the perception
 * parameter section. (e.g perception.custom_get_good_region)
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/1/12
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "custom_region_detection.h"
#include "../detector_base/unit.hpp"
#include "core/module.h"
#include "core/msg/ObstacleMsg.hpp"
#include "core/msg/perception_state_msg.hpp"
#include "core/settings.h"

// CODE
namespace object_detector{

using PerceptionCommandMsg = sros::core::PerceptionCommandMsg;
using PerceptionStateMsg = sros::core::PerceptionStateMsg;
using DetectResult = sros::core::PerceptionStateMsg::DetectResult;
using Command = sros::core::PerceptionCommandMsg::Command;
using DetectStage = sros::core::PerceptionCommandMsg::DetectStage;
using ObjectType = sros::core::PerceptionCommandMsg::ObjectType;

CustomRegionDetectorModule::CustomRegionDetectorModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic)
    : DetectorModuleBase("CustomRegionObstacleQuery", sendMsg, subscribeTopic){
}

bool CustomRegionDetectorModule::isEnable(){
    auto &s = sros::core::Settings::getInstance();
    return false;
}

bool CustomRegionDetectorModule::init() {
    if (!isEnable()) {
        LOG(WARNING) << "custom region detector module stop running(disable)";
        return false;
    }

    loadDetectParam();

    LOG(INFO) << "custom region detector module start running";
    subscribeTopic_("TIMER_50MS", CALLBACK(&CustomRegionDetectorModule::checkDetectTime));
    subscribeTopic_("05LA_SCAN", CALLBACK(&CustomRegionDetectorModule::onSensorMsg));

    is_init_detector_ = true;
    return true;
}

void CustomRegionDetectorModule::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
    if (!is_init_detector_) {
        LOG(WARNING) << "custom region detector is disable!";
        return;
    }

    // Analyse detect command and parameter.
    std::shared_ptr<PerceptionCommandMsg> cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);
    LOGGER(INFO, ACTION_TASK) << "receive custom region detect command : cmd:"
                              << cmd->command.detect_stage * 10 + cmd->command.object_type
                              << " pose:" << cmd->theory_pose.x() << "," << cmd->theory_pose.y() << ","
                              << cmd->theory_pose.yaw() << " goal_id," << cmd->goal_id;
    this->theory_pose_[0] = cmd->theory_pose.x();
    this->theory_pose_[1] = cmd->theory_pose.y();
    this->theory_pose_[2] = cmd->theory_pose.yaw();
    this->command_ = cmd->command;
    enable_process_ = true;
}

void CustomRegionDetectorModule::onSensorMsg(const sros::core::base_msg_ptr &msg) {
    LOG(WARNING) << "zx test: receive 05LA_SCAN topic";

    if (!enable_process_) {
        return;
    }

    const sros::core::LaserScan_ptr scan_msg = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(msg);

    if (!isScanDataIntegrity(scan_msg)) {
        return;
    }

    slam::tf::TransForm curr_pose;
    if (!tf_base_to_world_->lookUpTransForm(msg->time_, curr_pose, 50000)) {
        LOG(INFO) << "delta time is wrong! cannot get real time pose!";
        return;
    }
    
    this->cur_pose_[0] = curr_pose.position.x();
    this->cur_pose_[1] = curr_pose.position.y();
    this->cur_pose_[2] = curr_pose.rotation.yaw();

    showLaserScanOnMatrix(this->cur_pose_, scan_msg);

    auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
    result_msg->command = this->command_;
    if(ObjectType::OBJECT_TYPE_CUSTOM_REGION_HAVE_OBSTACLE == this->command_.object_type){
        if (isRectangleRegionOccupy(this->rectangle_, scan_msg)){
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_SUCCESS;
            result_msg->error_code - PerceptionStateMsg::ERROR_CODE_INVALID;
        } else {
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_NOT_EXISTENCE_OBSTACLE;
        }
    } else {
        if (isRectangleRegionOccupy(this->rectangle_, scan_msg)) {
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_EXISTENCE_OBSTACLE;
        } else {
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_SUCCESS;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_INVALID;
        }
    }
    sendMsg_(result_msg);
    enable_process_ = false;
}

bool CustomRegionDetectorModule::isScanDataIntegrity(const sros::core::LaserScan_ptr &scan) {
    const double scan_range = scan->angle_max - scan->angle_min;
    const int POINT_SIZE = static_cast<int>(scan_range / scan->angle_increment);
    const int ranges_size = scan->ranges.size();

    if (ranges_size != POINT_SIZE) {
        LOG(INFO) << "scan data wrong!"
                  << " scan->range.size()=" << scan->ranges.size() << " size=" << POINT_SIZE;
        return false;
    }
    return true;
}

void CustomRegionDetectorModule::loadDetectParam() {
    auto &s = sros::core::Settings::getInstance();
    std::string custom_region_str = s.getValue<std::string>("perception.custom_check_region", "");
    buildRectangleInfo(custom_region_str, this->rectangle_);
}

bool CustomRegionDetectorModule::buildRectangleInfo(const std::string &info_str, Rectangle &rectangle) {
    auto infos = common_func::splitStringToVector<float>(info_str, ';');
    if (infos.size() == 8) {
        Eigen::Vector2d temp;
        rectangle.resize(4);
        for (size_t i = 0; i < 8; i += 2) {
            temp[0] = infos[i];
            temp[1] = infos[i + 1];
            rectangle[i / 2] = temp;
        }
        LOG(INFO) << "rectangle: A(" << rectangle[0].x() << "," << rectangle[0].y() << ") B(" << rectangle[1].x() << ","
                  << rectangle[1].y() << ") C(" << rectangle[2].x() << "," << rectangle[2].y() << ") D("
                  << rectangle[3].x() << "," << rectangle[3].y() << ")";
        return true;
    } else {
        LOG(INFO) << "custom detect rectangle region info is wrong!" << info_str << "," << infos.size();
    }
    return false;
}

bool CustomRegionDetectorModule::isSectorRegionOccupy(const SectorRegion &region,
                                                      const sros::core::LaserScan_ptr &scan) {
    const int ranges_size = scan->ranges.size();
    const int begin_angle = static_cast<int>(region.begin_angle / scan->angle_increment);
    const int end_angle = static_cast<int>(region.end_angle / scan->angle_increment);

    if (begin_angle < 0 || begin_angle > end_angle || end_angle > ranges_size) {
        return true;
    }

    // check the point of region point.distance < radius.
    for (size_t i = begin_angle; i < end_angle; ++i) {
        if (scan->ranges[i] < region.radius * 1000) {
            return true;
        }
    }
    return false;
}

bool CustomRegionDetectorModule::isRectangleRegionOccupy(const Rectangle &rectangle,
                                                         const sros::core::LaserScan_ptr &scan) {
    const int ranges_size = scan->ranges.size();
    double temp_angle;
    Eigen::Vector2d temp_point;
    for (size_t i = 0; i < ranges_size; ++i) {
        temp_angle = i * scan->angle_increment;
        temp_point[0] = scan->ranges[i] * cos(temp_angle);
        temp_point[1] = scan->ranges[i] * sin(temp_angle);
        if (common_func::isPointInRectangle(rectangle, temp_point)) {
            return true;
        }
    }
    return false;
}

void CustomRegionDetectorModule::showLaserScanOnMatrix(const Eigen::Vector3d &cur_pose,
                                                       const sros::core::LaserScan_ptr &scan_msg) {
    const size_t size = scan_msg->ranges.size();

    Eigen::Translation2d translation2D(cur_pose[0], cur_pose[1]);
    Eigen::Rotation2Dd rotation2D(cur_pose[2]);
    Eigen::Affine2d world_tf = translation2D * rotation2D;

    Eigen::Vector2d temp_point;
    sros::core::ObstacleMsg_ptr point_could;

    sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("TOPIC_OBSTACLE"));
    oba_msg->oba_name = "R2100_OBA";
    oba_msg->time_ = scan_msg->time_;
    oba_msg->point_cloud.resize(size);
    double angle = scan_msg->angle_min;
    Eigen::Vector2d temp, temp_word;
    for (size_t i = 0; i < size; ++i) {
        temp[0] = scan_msg->ranges[i] * cos(angle);
        temp[1] = scan_msg->ranges[i] * sin(angle);

        temp_word = world_tf * temp;

        oba_msg->point_cloud[i].x() = temp_word.x();
        oba_msg->point_cloud[i].y() = temp_word.y();

        angle += scan_msg->angle_increment;
    }
    sendMsg_(oba_msg);
}

}  // end of namespace space_detector

