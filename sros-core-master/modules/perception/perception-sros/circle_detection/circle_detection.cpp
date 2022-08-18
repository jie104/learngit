/**
 * @file circle_detector_module.cpp
 * @brief The class responsible for the circle awareness module.
 *
 * If the parameter of main.enable_circle_detect is set to true, the circle sensing module
 * will be turned on and waiting for the collect to be DETECT_COMMAND and IFM3D_IMG message,
 * once received DETECT_COMMAND message, that is enable_img_process_ will be set to true,
 * the next IFM3D_IMG received will be processed by run(), than the circle detection
 * program will be executed and the result will be sent to DETECT_RESULT topic.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/11/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "circle_detection.h"
#include "../detector_base/unit.hpp"

#include "core/msg/image_msg.hpp"
#include "core/msg/perception_state_msg.hpp"
#include "core/msg/perception_command_msg.hpp"
#include "core/settings.h"

// CODE
namespace object_detector {

using PerceptionCommandMsg = sros::core::PerceptionCommandMsg;
using PerceptionCommandMsgPtr = sros::core::PerceptionCommandMsgPtr;
using PerceptionStateMsg = sros::core::PerceptionStateMsg;
using PerceptionStateMsgPtr = sros::core::PerceptionStateMsgPtr;
using Command = sros::core::PerceptionCommandMsg::Command;
using DetectStage = sros::core::PerceptionCommandMsg::DetectStage;
using ObjectType = sros::core::PerceptionCommandMsg::ObjectType;

CircleDetectModule::CircleDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic)
    : DetectorModuleBase("CircleDetect", sendMsg, subscribeTopic) {

}

bool CircleDetectModule::isEnable() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_load_detect = (s.getValue<std::string>("main.enable_load_detect", "False") == "True");
    bool enable_circle_detect = (s.getValue<std::string>("perception.detect_goods_type", "") == "CIRCLE");
    return enable_load_detect && enable_circle_detect;
}

bool CircleDetectModule::init() {
    if (!isEnable()) {
        LOG(WARNING) << "CircleDetect module stop running(disable)";
        return false;
    }

    loadDetectParam();
    loadTargetParam();
    if (!circle_detector_.init(param_, circles_map_)) {
        LOG(WARNING) << "Circle Detector init fail";
        return false;
    }

    LOG(INFO) << "CircleDetect module start running";
    subscribeTopic_("TIMER_50MS", CALLBACK(&CircleDetectModule::checkDetectTime));
    subscribeTopic_("IFM3D_IMG", CALLBACK(&CircleDetectModule::onSensorMsg));

    is_init_detector_ = true;
    return true;
}

void CircleDetectModule::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
    if (!is_init_detector_) {
        LOG(WARNING) << "circle detector is disable!";
        return;
    }

    // Analyse detect command and parameter.
    PerceptionCommandMsgPtr cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);
    LOGGER(INFO, ACTION_TASK) << "receive circle detect command : cmd:"
                              << cmd->command.detect_stage * 10 + cmd->command.object_type
                              << " pose:" << cmd->theory_pose.x() << "," << cmd->theory_pose.y() << ","
                              << cmd->theory_pose.yaw() << " goal_id," << cmd->goal_id;
    this->theory_pose_[0] = cmd->theory_pose.x();
    this->theory_pose_[1] = cmd->theory_pose.y();
    this->theory_pose_[2] = cmd->theory_pose.yaw();
    this->command_ = cmd->command;
    this->is_receive_cmd_ = true;
    this->goal_id_ = cmd->goal_id;
    this->receive_cmd_time_ = sros::core::util::get_time_in_ms();
    if (0 == this->goal_id_) {
        LOG(WARNING) << "cmd->goal_id = 0 (zero is invalid value)" << this->goal_id_;
    } else if (-1 == this->goal_id_ || this->circles_map_.find(this->goal_id_) != this->circles_map_.end()) {
        startDetector();
    } else {
        LOG(WARNING) << "Command parameter error:"
                     << " cmd->goal_id_=" << cmd->goal_id;
        stopDetector();
    }
}

void CircleDetectModule::onSensorMsg(const sros::core::base_msg_ptr &msg) {
    //    auto begin_time = sros::core::util::get_time_in_us();
    //    LOG(ERROR) << "receive 03d3xx("<<msg->time_<<") frame time point: " << sros::core::util::get_time_in_us();

    //    enable_process_ = true; //调试状态,确保每次都识别展板
    if (!enable_process_) {
        return;
    }

    detected_count_++;
    sros::core::image_msg_ptr img = std::dynamic_pointer_cast<sros::core::ImageMsg>(msg);

    O3d3xxFrame o3d3xx_frame;
    img->amplitude_img_.convertTo(o3d3xx_frame.amplitude_img, CV_16UC1);
    img->mat_.convertTo(o3d3xx_frame.confidence_img, CV_8UC1);
    img->mat_xyz_.convertTo(o3d3xx_frame.xyz_img, CV_16SC3);
    common_func::convert2PointCloud(o3d3xx_frame.xyz_img, o3d3xx_frame.cloud);

    if (this->is_record_) {
        recordSensorData("/sros/log/", o3d3xx_frame);
    }

    int64_t start = common_func::get_time_in_ms();
    DetectResult result = circle_detector_.detect(o3d3xx_frame);
    int64_t finish = common_func::get_time_in_ms();
    showResult(result, finish - start);

    if (result.is_available) {
        Eigen::Vector3f circle_pose(result.x, result.y, result.angle);
        detected_poses_.push_back(circle_pose);
        this->goal_id_ = result.id;
    }

    if (detected_count_ < DETECTED_COUNT_THRESH) {
        return;
    }

    if (this->is_record_ && out_put_.is_open()) {
        LOG(INFO) << "close sensor data file: " << this->sensor_data_file_path_;
        out_put_.close();
    }

    // If no target is detected, send (0,0,0) and return.
    auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
    result_msg->command = this->command_;
    result_msg->goal_in_global_pose.x() = .0f;
    result_msg->goal_in_global_pose.y() = .0f;
    result_msg->goal_in_global_pose.yaw() = .0f;
    result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
    result_msg->error_code = PerceptionStateMsg::ERROR_CODE_NOT_FIND_GOAL;

    result_msg->goal_id = -1;
    if (detected_poses_.empty()) {
        LOG(INFO) << "detect pose empty!";
        stopDetector();
        sendResultMsg(result_msg);
        return;
    }

    // Only keep the log file when the target is not detected, and delete the rest.
    if (this->is_record_) {
        LOG(INFO) << "remove sensor data file: " << this->sensor_data_file_path_;
        remove(this->sensor_data_file_path_.c_str());
    }

    // Judge that the target angle and Y-direction offset are greater than the set boundary.
    auto mean_pose = computeTargetMeanPose(detected_poses_);

    // else init the detect result message and send to topic:DETECT_RESULT
    slam::tf::TransForm curr_pose;
    if (!tf_base_to_world_->lookUpTransForm(msg->time_, curr_pose, 50000)) {
        LOG(INFO) << "delta time is wrong! cannot get real time pose!";
        return;
    }
    Eigen::Vector3f agv_in_world_tf(curr_pose.position.x(), curr_pose.position.y(), curr_pose.rotation.yaw());
    auto camera_in_world_pose = common_func::cvtToWorldPose(agv_in_world_tf, camera_to_base_pose_);
    auto goal_in_world_pose = common_func::cvtToWorldPose(camera_in_world_pose, mean_pose);
    auto goal_in_agv_pose = common_func::cvtToWorldPose(camera_to_base_pose_, mean_pose);
    result_msg->goal_in_global_pose.x() = goal_in_world_pose[0];
    result_msg->goal_in_global_pose.y() = goal_in_world_pose[1];
    result_msg->goal_in_global_pose.yaw() = goal_in_world_pose[2];

    result_msg->goal_in_agv_pose.x() = goal_in_agv_pose[0];
    result_msg->goal_in_agv_pose.y() = goal_in_agv_pose[1];
    result_msg->goal_in_agv_pose.yaw() = goal_in_agv_pose[2];

    result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_SUCCESS;
    result_msg->error_code = PerceptionStateMsg::ERROR_CODE_INVALID;
    result_msg->goal_id = this->goal_id_;
    
    LOG(INFO) << "        agv in world pose: " << curr_pose.position.x() << ", " << curr_pose.position.y() << ", "
              << curr_pose.rotation.yaw();
    LOG(INFO) << "     camera in world pose: " << camera_in_world_pose[0] << ", " << camera_in_world_pose[1] << ", "
              << camera_in_world_pose[2];
    LOG(INFO) << "goal in camera world pose: " << mean_pose[0] << ", " << mean_pose[1] << ", " << mean_pose[2];
    LOG(INFO) << "goal in global world pose: " << goal_in_world_pose[0] << ", " << goal_in_world_pose[1]
              << ", " << goal_in_world_pose[2];
    LOG(INFO) << "   goal in agv world pose: " << goal_in_agv_pose[0] << ", " << goal_in_agv_pose[1]
              << ", " << goal_in_agv_pose[2];
    LOG(INFO) << "  theory in world pose: " << theory_pose_[0] << " ," << theory_pose_[1] << " ," << theory_pose_[2];

    stopDetector();
    sendResultMsg(result_msg);

    //    auto end_time = sros::core::util::get_time_in_us();
    //    LOG(ERROR) << "process 03d3xx("<<msg->time_<<") frame consume time : " << (end_time-begin_time) / 1.0e6;
    //    LOG(ERROR) << "--------------------------------------------------------------------";
}

void CircleDetectModule::loadDetectParam() {
    using namespace perception;

    // Load detect common parameter
    sros::core::Settings &s = sros::core::Settings::getInstance();
    common_func::loadCommonParam<CircleDetectParam>(param_);
    is_record_ = (s.getValue<std::string>("perception.is_record", "True") == "True");

    // load detect range parameter.
    std::vector<float> range3d_str_vct;
    const int range3d_param_num = sizeof(param_.detect_range) / sizeof(float);
    const int decode_param_num = common_func::decodeStringParam("perception.circle_disk_range3d", range3d_str_vct);
    if (decode_param_num == range3d_param_num) {
        param_.detect_range.min_x = range3d_str_vct[0];
        param_.detect_range.max_x = range3d_str_vct[1];
        param_.detect_range.min_y = range3d_str_vct[2];
        param_.detect_range.max_y = range3d_str_vct[3];
        param_.detect_range.min_z = range3d_str_vct[4];
        param_.detect_range.max_z = range3d_str_vct[5];
    } else {
        param_.detect_range.min_x = CIRCLE_DEFAULT_MIN_X;
        param_.detect_range.max_x = CIRCLE_DEFAULT_MAX_X;
        param_.detect_range.min_y = CIRCLE_DEFAULT_MIN_Y;
        param_.detect_range.max_y = CIRCLE_DEFAULT_MAX_Y;
        param_.detect_range.min_z = CIRCLE_DEFAULT_MIN_Z;
        param_.detect_range.max_z = CIRCLE_DEFAULT_MAX_Z;
    }

    param_.min_cluster_size =
        s.getValue<int>("perception.circle_disk_min_cluster_size", CIRCLE_DEFAULT_MIN_CLUSTER_SIZE);
    param_.max_cluster_size =
        s.getValue<int>("perception.circle_disk_max_cluster_size", CIRCLE_DEFAULT_MAX_CLUSTER_SIZE);
    param_.point_dist_tolerance =
        s.getValue<float>("perception.circle_disk_point_dist_tolerance", CIRCLE_DEFAULT_POINT_DIST_TOLERANCE);
    param_.threshold = s.getValue<int>("perception.circle_disk_threshold", CIRCLE_DEFAULT_THRESHOLD);
    param_.normal_angle_tolerance =
        s.getValue<float>("perception.circle_disk_normal_angle_tolerance", CIRCLE_DEFAULT_NORMAL_ANGLE_TOLERANCE);
    param_.max_iteration = s.getValue<int>("perception.circle_disk_max_iteration", CIRCLE_DEFAULT_MAX_ITERATION);
    param_.fitting_rate = s.getValue<float>("perception.circle_disk_fitting_rate", CIRCLE_DEFAULT_FITTING_RATE);
    param_.pixel_dist_allow =
        s.getValue<float>("perception.circle_disk_pixel_dist_allow", CIRCLE_DEFAULT_PIXEL_DIST_ALLOW);
    param_.contour_min_size =
        s.getValue<int>("perception.circle_disk_contour_min_size", CIRCLE_DEFAULT_CONTOUR_MIN_SIZE);
}

void CircleDetectModule::loadTargetParam() {
    auto &s = sros::core::Settings::getInstance();
    camera_to_base_pose_[0] = s.getValue<double>("camera.o3d303_install_x_offset", 0) / 1000.0;
    camera_to_base_pose_[1] = s.getValue<double>("camera.o3d303_install_y_offset", 0) / 1000.0;
    camera_to_base_pose_[2] = s.getValue<double>("camera.o3d303_install_yaw", 0) / 180.0 * M_PI;

    auto get_circle_param = [&](const std::string &name, const std::string &default_value) {
        auto first_circle_info_str = s.getValue<std::string>(name, default_value);
        perception::Circle circle;
        LOG(INFO) << first_circle_info_str;
        if (buildCircleInfo(first_circle_info_str, circle)) {
            this->circles_map_.insert(std::make_pair(circle.getId(), std::move(circle)));
        }
    };

    get_circle_param("perception.circle_disk_info", "0");

    LOG(INFO) << "load circles size: " << this->circles_map_.size();
}

bool CircleDetectModule::buildCircleInfo(const std::string &info_str, perception::Circle &circle_info) {
    auto infos = common_func::splitStringToVector<std::string>(info_str, ';');
    if (infos.size() == 3) {
        circle_info.setId(std::stoi(infos[0]));
        circle_info.setLength(std::stof(infos[1]));
        circle_info.setRadius(std::stof(infos[2]));
        circle_info.setWidth(std::stof(infos[2]) * 2);
        LOG(INFO) << "circle id=" << circle_info.getId() << " length=" << circle_info.getLength()
                  << " radius=" << circle_info.getRadius() << " width=" << circle_info.getWidth();
        return true;
    } else {
        LOG(INFO) << "circle info is wrong!" << info_str << "," << infos.size();
    }
    return false;
}

}  // namespace object_detector