/**
 * @file handcart_detector_module.cpp
 * @brief The class responsible for the handcart awareness module.
 *
 * If the parameter of main.enable_handcart_detect is set to true, the handcart sensing module will
 * be turned on and waiting for the collect to be DETECT_COMMAND and IFM3D_IMG message,
 * once received DETECT_COMMAND message, that is enable_img_process_ will be set to
 * true, the next IFM3D_IMG received will be processed by run(), than the handcart detection
 * program will be executed and the result will be sent to DETECT_RESULT topic.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/11/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "handcart_detection.h"
#include "../perception-sros/detector_base/unit.hpp"

#include "core/msg/common_msg.hpp"
#include "core/msg/point_cloud_msg.hpp"
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

HandcartDetectModule::HandcartDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic)
    : DetectorModuleBase("HandcartDetect", sendMsg, subscribeTopic) {

}

bool HandcartDetectModule::isEnable() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_load_detect = (s.getValue<std::string>("main.enable_load_detect", "False") == "True");
    bool enable_handcart_detect =  (s.getValue<std::string>("perception.detect_goods_type", "") == "HANDCART");
    return enable_load_detect && enable_handcart_detect;
}

void HandcartDetectModule::enableSensor(){
    LOG(INFO)  << "send TOPIC_LIVOX_ENABLE_PUBLISH msg";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_LIVOX_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_LIDAR_LIVOX;
    cmd_msg->flag = true;
    sendMsg_(cmd_msg);
}

void HandcartDetectModule::disableSensor() {
    LOG(INFO)  << "send TOPIC_LIVOX_ENABLE_PUBLISH msg";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_LIVOX_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_LIDAR_LIVOX;
    cmd_msg->flag = false;
    sendMsg_(cmd_msg);
}

bool HandcartDetectModule::init() {
    if (!isEnable()) {
        LOG(WARNING) << "HandcartDetect module init fail running(disable)";
        return false;
    }

    loadDetectParam();
    loadTargetParam();
    if (!handcart_detector_.init(param_, handcarts_map_)) {
        LOG(ERROR) << "Handcart Detector init fail";
        return false;
    };

    LOG(INFO) << "HandcartDetect module start running";
    subscribeTopic_("TIMER_50MS", CALLBACK(&HandcartDetectModule::checkDetectTime));
    subscribeTopic_("LIVOX_MID70", CALLBACK(&HandcartDetectModule::onSensorMsg));

    is_init_detector_ = true;
    return true;
}

void HandcartDetectModule::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
    if (!is_init_detector_) {
        LOG(WARNING) << "handcart detector is disable!";
        return;
    }

    // Analyse detect command and parameter.
    PerceptionCommandMsgPtr cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);
    LOGGER(INFO, ACTION_TASK) << "receive handcart detect command : cmd:"
                              << cmd->command.detect_stage * 10 + cmd->command.object_type
                              << " pose:" << cmd->theory_pose.x() << "," << cmd->theory_pose.y() << ","
                              << cmd->theory_pose.yaw() << " goal_id," << cmd->goal_id;
    this->theory_pose_[0] = cmd->theory_pose.x();
    this->theory_pose_[1] = cmd->theory_pose.y();
    this->theory_pose_[2] = cmd->theory_pose.yaw();
    this->command_ = cmd->command;
    this->goal_id_ = cmd->goal_id;
    this->is_receive_cmd_ = true;
    this->receive_cmd_time_ = sros::core::util::get_time_in_ms();
    if (0 == this->goal_id_) {
        LOG(WARNING) << "cmd->goal_id = 0 (zero is invalid value)" << this->goal_id_;
    } else if (-1 == this->goal_id_ || this->handcarts_map_.find(this->goal_id_) != handcarts_map_.end()) {
        startDetector();
        frame_queue_.clear();
    } else {
        LOG(WARNING) << "Command parameter error: cmd->goal_id_=" << cmd->goal_id;
        stopDetector();
        frame_queue_.clear();
    }
}

static void convert2PointCloud(const sros::core::PointCloudMsg &frame, const PointCloudPtr &cloud){
    cloud->reserve(frame.cloud.size());
    int index = 0;
    for (auto const &point : frame.cloud) {
        cloud->points.emplace_back(point.x(), point.y(), point.z());
        cloud->image_indices[index] = index;
        ++index;
    }
    cloud->time = frame.seq;
}

void HandcartDetectModule::onSensorMsg(const sros::core::base_msg_ptr &msg) {
    //    auto begin_time = sros::core::util::get_time_in_us();
    //    LOG(ERROR) << "receive 03d3xx("<<msg->time_<<") frame time point: " << sros::core::util::get_time_in_us();

    //    enable_process_ = true; // 调试状态,确保每次都识别展板
    if (!enable_process_) {
        return;
    }

    detected_count_++;
    auto frame = std::dynamic_pointer_cast<sros::core::PointCloudMsg>(msg);
    PointCloudPtr cloud(new PointCloud());
    convert2PointCloud(*frame, cloud);

    if (cloud->empty()) {
        LOG(WARNING) << "receive cloud.size=0";
    } else {
        frame_queue_.push_back(cloud);
    }

    // accumulated frame.
    const int QUEUE_SIZE = frame_queue_.size();
    if (QUEUE_SIZE < ACCUMULATE_QUEUE_SIZE_COUNT){
        if (0 == QUEUE_SIZE % 100) {
            LOG(WARNING) << "receive frame_queue_.size=" << std::setw(4) << std::setfill(' ') << QUEUE_SIZE;
        }
        return;
    }

    // The accumulated multi frame data is merged together.
    int index = 0;
    PointCloudPtr multi_cloud(new PointCloud());
    multi_cloud->resize(frame_queue_.size() * 96);
    for (auto const &f : frame_queue_) {
        for (auto const &point : f->points) {
//            multi_cloud->points.emplace_back(point.x/1000.0f, point.y/1000.0f, point.z/1000.0f);
//            multi_cloud->image_indices.push_back(index);
            multi_cloud->points[index].x = point.x / 1000.0f;
            multi_cloud->points[index].y = point.y / 1000.0f;
            multi_cloud->points[index].z = point.z / 1000.0f;
            multi_cloud->image_indices[index] = index;
            ++index;
        }
    }

    // record data.
    if (this->is_record_) {
        recordSensorData("/sros/log/", multi_cloud);
    }

    // detect object from sensor data.
    int64_t start = common_func::get_time_in_ms();
    DetectResult result = handcart_detector_.detect(multi_cloud);
    int64_t finish = common_func::get_time_in_ms();
    showResult(result, finish - start);

    if (result.is_available) {
        Eigen::Vector3f handcart_pose(result.x, result.y, result.angle);
        detected_poses_.push_back(handcart_pose);
        this->goal_id_ = result.id;
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
    LOG(INFO) << "     sensor in world pose: " << camera_in_world_pose[0] << ", " << camera_in_world_pose[1] << ", "
              << camera_in_world_pose[2];
    LOG(INFO) << "goal in sensor world pose: " << mean_pose[0] << ", " << mean_pose[1] << ", " << mean_pose[2];
    LOG(INFO) << "goal in global world pose: " << goal_in_world_pose[0] << ", " << goal_in_world_pose[1]
              << ", " << goal_in_world_pose[2];
    LOG(INFO) << "   goal in agv world pose: " << goal_in_agv_pose[0] << ", " << goal_in_agv_pose[1]
              << ", " << goal_in_agv_pose[2];
    LOG(INFO) << "     theory in world pose: " << theory_pose_[0] << " ," << theory_pose_[1] << " ," << theory_pose_[2];

    stopDetector();
    sendResultMsg(result_msg);

    //    auto end_time = sros::core::util::get_time_in_us();
    //    LOG(ERROR) << "process 03d3xx("<<msg->time_<<") frame consume time : " << (end_time-begin_time) / 1.0e6;
    //    LOG(ERROR) << "--------------------------------------------------------------------";
}

void HandcartDetectModule::loadDetectParam() {
    using namespace perception;

    // Load detect common parameter
    sros::core::Settings &s = sros::core::Settings::getInstance();
    common_func::loadCommonParam<HandcartDetectParam>(param_);
    is_record_ = (s.getValue<std::string>("perception.is_record", "True") == "True");

    // load detect range parameter.
    std::vector<float> range3d_str_vct;
    const int range3d_param_num = sizeof(param_.detect_range) / sizeof(float);
    const int decode_param_num = common_func::decodeStringParam("perception.handcart_range3d", range3d_str_vct);
    if (decode_param_num == range3d_param_num) {
        param_.detect_range.min_x = range3d_str_vct[0];
        param_.detect_range.max_x = range3d_str_vct[1];
        param_.detect_range.min_y = range3d_str_vct[2];
        param_.detect_range.max_y = range3d_str_vct[3];
        param_.detect_range.min_z = range3d_str_vct[4];
        param_.detect_range.max_z = range3d_str_vct[5];
    } else {
        param_.detect_range.min_x = HANDCART_DEFAULT_MIN_X;
        param_.detect_range.max_x = HANDCART_DEFAULT_MAX_X;
        param_.detect_range.min_y = HANDCART_DEFAULT_MIN_Y;
        param_.detect_range.max_y = HANDCART_DEFAULT_MAX_Y;
        param_.detect_range.min_z = HANDCART_DEFAULT_MIN_Z;
        param_.detect_range.max_z = HANDCART_DEFAULT_MAX_Z;
    }

    param_.min_cluster_size = s.getValue<int>("perception.handcart_min_cluster_size", HANDCART_DEFAULT_MIN_CLUSTER_SIZE);
    param_.max_cluster_size = s.getValue<int>("perception.handcart_max_cluster_size", HANDCART_DEFAULT_MAX_CLUSTER_SIZE);
    param_.point_dist_tolerance =
        s.getValue<float>("perception.handcart_point_dist_tolerance", HANDCART_DEFAULT_POINT_DIST_TOLERANCE);
    param_.rate_threshold = s.getValue<int>("perception.handcart_rate_threshold", HANDCART_DEFAULT_RATE_THRESHOLD);
    param_.normal_angle_tolerance =
        s.getValue<float>("perception.handcart_normal_angle_tolerance", HANDCART_DEFAULT_NORMAL_ANGLE_TOLERANCE);
    param_.across_dist_allow =
        s.getValue<float>("perception.across_dist_allow", HANDCART_DEFAULT_ACROSS_DIST_ALLOW);
}

void HandcartDetectModule::loadTargetParam() {
    auto &s = sros::core::Settings::getInstance();
    camera_to_base_pose_[0] = s.getValue<double>("perception.livox_install_x_offset", 0) / 1000.0;
    camera_to_base_pose_[1] = s.getValue<double>("perception.livox_install_y_offset", 0) / 1000.0;
    camera_to_base_pose_[2] = s.getValue<double>("perception.livox_install_yaw", 0) / 180.0 * M_PI;

    auto get_handcart_param = [&](const std::string &handcart_name, const std::string &default_value) {
        auto first_pallet_info_str = s.getValue<std::string>(handcart_name, default_value);
        perception::Handcart handcart;
        LOG(INFO) << first_pallet_info_str;
        if (buildHandcartInfo(first_pallet_info_str, handcart)) {
            handcarts_map_.insert(std::make_pair(handcart.getId(), handcart));
        }
    };

    get_handcart_param("perception.handcart_info", "0");

    LOG(INFO) << "load handcarts size: " << handcarts_map_.size();
}

bool HandcartDetectModule::buildHandcartInfo(const std::string &info_str, perception::Handcart &handcart_info) {
    auto infos = common_func::splitStringToVector<std::string>(info_str, ';');
    if (infos.size() == 4) {
        handcart_info.setId(std::stoi(infos[0]));
        handcart_info.setLength(std::stof(infos[1]));
        handcart_info.setAcrossHeight(std::stof(infos[2]));
        handcart_info.setWidth(std::stof(infos[3]));

        LOG(INFO) << "handcart id=" << infos[0] << " length="  << infos[1] << " across_height=" << infos[2]
                  << " width=" << infos[3];
        return true;
    } else {
        LOG(WARNING) << "pallet info is wrong! parame_string=" << info_str << ", decode size=" << infos.size();
    }
    return false;
}

void HandcartDetectModule::checkDetectTime(const sros::core::base_msg_ptr &msg) {
    if (is_receive_cmd_) {
        auto cur_time = sros::core::util::get_time_in_ms();
//        LOG(INFO) << "cur_time-receive_cmd_time_=" << cur_time-receive_cmd_time_ << " DETECTED_TIME_OUT="<< DETECTED_TIME_OUT;
        if (cur_time > DETECTED_TIME_OUT + receive_cmd_time_) {
            auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
            result_msg->command = this->command_;
            result_msg->goal_in_global_pose.x() = .0f;
            result_msg->goal_in_global_pose.y() = .0f;
            result_msg->goal_in_global_pose.yaw() = .0f;
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_TIMEOUT;
            LOGGER(INFO, SROS) << "detect time out! " << cur_time - receive_cmd_time_ << ">" << DETECTED_TIME_OUT;
            is_receive_cmd_ = false;
            stopDetector();
            sendMsg_(result_msg);
        }
    }
}

}  // namespace object_detector