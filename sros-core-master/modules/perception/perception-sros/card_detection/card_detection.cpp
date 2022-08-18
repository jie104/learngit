/**
 * @file card_detector_module.cpp
 * @brief The class responsible for the card awareness module.
 *
 * If the parameter of main.enable_card_detect is set to true, the card sensing module will
 * be turned on and waiting for the collect to be DETECT_COMMAND and IFM3D_IMG message,
 * once received DETECT_COMMAND message, that is enable_img_process_ will be set to
 * true, the next IFM3D_IMG received will be processed by run(), than the card detection
 * program will be executed and the result will be sent to DETECT_RESULT topic.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/11/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "card_detection.h"
#include "../perception-sros/detector_base/unit.hpp"

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

CardDetectModule::CardDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic)
    : DetectorModuleBase("CardDetect", sendMsg, subscribeTopic) {

}

bool CardDetectModule::isEnable() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_load_detect = (s.getValue<std::string>("main.enable_load_detect", "False") == "True");
    bool enable_card_detect =  (s.getValue<std::string>("perception.detect_goods_type", "") == "CARD");
    return enable_load_detect && enable_card_detect;
}

bool CardDetectModule::init() {
    if (!isEnable()) {
        LOG(WARNING) << "CardDetect module init fail running(disable)";
        return false;
    }

    loadDetectParam();
    loadTargetParam();
    if (!card_detector_.init(param_, cards_map_)) {
        LOG(ERROR) << "Card Detector init fail";
        return false;
    };

    LOG(INFO) << "CardDetect module start running";
    subscribeTopic_("TIMER_50MS", CALLBACK(&CardDetectModule::checkDetectTime));
    subscribeTopic_("IFM3D_IMG", CALLBACK(&CardDetectModule::onSensorMsg));

    is_init_detector_ = true;
    return true;
}

void CardDetectModule::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
    if (!is_init_detector_) {
        LOG(WARNING) << "card detector is disable!";
        return;
    }

    // Analyse detect command and parameter.
    PerceptionCommandMsgPtr cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);
    LOGGER(INFO, ACTION_TASK) << "receive card detect command : cmd:"
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
    
    subscribeTopic_("TIMER_50MS", CALLBACK(&CardDetectModule::checkDetectTime));
    subscribeTopic_("IFM3D_IMG", CALLBACK(&CardDetectModule::onSensorMsg));

    float current_camera_height = g_state.fork_height_encoder + camera_to_fork_height_;
    
    LOG(INFO) << "current_camera_height" << current_camera_height;
    LOG(INFO) << "g_state.fork_height_encoder" << g_state.fork_height_encoder;
    LOG(INFO) << "camera_to_fork_height_" << camera_to_fork_height_;

    // this->param_.target_value_z_ = cmd->obj_center_ground_clearance;
    this->param_.target_value_z_ = 90;
    this->param_.camera_to_agv_transf = Eigen::Vector3f(0, 0, 0);
    this->param_.camera_to_agv_rotate = Eigen::Vector3f(camera_to_agv_roll_, camera_to_agv_pitch_, 0.); //roll, pitch, yaw   

    // 获取当前位姿
    auto curr_pose = src_sdk->getCurPose();
    
    // 计算栈板到相机距离: 理论位姿到当前位姿距离 + 标定距离X 
    if(std::fabs(curr_pose.x() - this->theory_pose_[0]) > std::fabs(curr_pose.y() - this->theory_pose_[1])){
        param_.detect_range.min_x = camera_to_base_pose_[0] + std::fabs(curr_pose.x() - this->theory_pose_[0]) - 0.8;
        param_.detect_range.max_x = camera_to_base_pose_[0] + std::fabs(curr_pose.x() - this->theory_pose_[0]) - 0.3;
    }else{
        param_.detect_range.min_x = camera_to_base_pose_[0] + std::fabs(curr_pose.y() - this->theory_pose_[1]) - 0.8;
        param_.detect_range.max_x = camera_to_base_pose_[0] + std::fabs(curr_pose.y() - this->theory_pose_[1]) - 0.3;
    }
    
    LOG(INFO) << "param_.detect_range.min_x: " << param_.detect_range.min_x 
              << "   param_.detect_range.max_x: " << param_.detect_range.max_x;

    card_detector_.setParam(this->param_);


    if (0 == this->goal_id_) {
        LOG(WARNING) << "cmd->goal_id = 0 (zero is invalid value)" << this->goal_id_;
    } else if (-1 == this->goal_id_ || this->cards_map_.find(this->goal_id_) != cards_map_.end()) {
        startDetector();
    } else {
        LOG(WARNING) << "Command parameter error: cmd->goal_id_=" << cmd->goal_id;
        stopDetector();
    }


}

void CardDetectModule::onSensorMsg(const sros::core::base_msg_ptr &msg) {
    //    auto begin_time = sros::core::util::get_time_in_us();
    //    LOG(ERROR) << "receive 03d3xx("<<msg->time_<<") frame time point: " << sros::core::util::get_time_in_us();

    //    enable_process_ = true; // 调试状态,确保每次都识别展板
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
    
    //should transform pointcloud before processing
    PointCloudPtr transform_pointCloud(new PointCloud);
    common_func::transformPointCloud(o3d3xx_frame.cloud, transform_pointCloud, param_.camera_to_agv_transf, param_.camera_to_agv_rotate);
    o3d3xx_frame.cloud = transform_pointCloud;


    if (this->is_record_) {
        recordSensorData("/sros/log/", o3d3xx_frame);
    }
    
    int64_t start = common_func::get_time_in_ms();
    // DetectResult result = card_detector_.detect_test(o3d3xx_frame);
    DetectResult result = card_detector_.detect(o3d3xx_frame);
    int64_t finish = common_func::get_time_in_ms();
    showResult(result, finish - start);

    if (result.is_available) {
        Eigen::Vector3f card_pose(result.x, result.y, result.angle);
        if(!isnan(result.x) && !isnan(result.y) && !isnan(result.angle)){
            detected_poses_.push_back(card_pose);
            this->goal_id_ = result.id;
            LOGGER(INFO, ACTION_TASK) << "#Decards Info: Current frame detect result cam_z:"<<result.z<<" fork_encoder:"<< img->getForkHeightEncoder();
        }else{
            LOG(INFO) << "results have nan value, ignore it.";
        }
        
    }

    if (detected_count_ < DETECTED_COUNT_THRESH) {
        return;
    }

    // if (this->is_record_ && out_put_.is_open()) {
    //     LOG(INFO) << "close sensor data file: " << this->sensor_data_file_path_;
    //     out_put_.close();
    // }

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
    // if (this->is_record_) {
    //     LOG(INFO) << "remove sensor data file: " << this->sensor_data_file_path_;
    //     remove(this->sensor_data_file_path_.c_str());
    // }

    // Judge that the target angle and Y-direction offset are greater than the set boundary.
    auto mean_pose = computeTargetMeanPose(detected_poses_);

    for (const auto &pose : detected_poses_) {
        LOG(INFO) << "poses: " << pose[0] << " " << pose[1] << " " << pose[2];
    }
    LOG(INFO) << "mean_pose: " << mean_pose[0] << " " << mean_pose[1] << " " << mean_pose[2];

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
    LOG(INFO) << "     theory in world pose: " << theory_pose_[0] << " ," << theory_pose_[1] << " ," << theory_pose_[2];

    stopDetector();
    sendResultMsg(result_msg);
    o3d3xx_frame.reset();
    //    auto end_time = sros::core::util::get_time_in_us();
    //    LOG(ERROR) << "process 03d3xx("<<msg->time_<<") frame consume time : " << (end_time-begin_time) / 1.0e6;
    //    LOG(ERROR) << "--------------------------------------------------------------------";
}

void CardDetectModule::loadDetectParam() {
    using namespace perception;

    // Load detect common parameter
    sros::core::Settings &s = sros::core::Settings::getInstance();
    common_func::loadCommonParam<CardDetectParam>(param_);
    is_record_ = (s.getValue<std::string>("perception.is_record", "True") == "True");

    // load detect range parameter.
    std::vector<float> range3d_str_vct;
    const int range3d_param_num = sizeof(param_.detect_range) / sizeof(float);
    const int decode_param_num = common_func::decodeStringParam("perception.card_range3d", range3d_str_vct);
    if (decode_param_num == range3d_param_num) {
        param_.detect_range.min_x = range3d_str_vct[0];
        param_.detect_range.max_x = range3d_str_vct[1];
        param_.detect_range.min_y = range3d_str_vct[2];
        param_.detect_range.max_y = range3d_str_vct[3];
        param_.detect_range.min_z = range3d_str_vct[4];
        param_.detect_range.max_z = range3d_str_vct[5];
    } else {
        param_.detect_range.min_x = CARD_DEFAULT_MIN_X;
        param_.detect_range.max_x = CARD_DEFAULT_MAX_X;
        param_.detect_range.min_y = CARD_DEFAULT_MIN_Y;
        param_.detect_range.max_y = CARD_DEFAULT_MAX_Y;
        param_.detect_range.min_z = CARD_DEFAULT_MIN_Z;
        param_.detect_range.max_z = CARD_DEFAULT_MAX_Z;
    }

    param_.min_cluster_size = s.getValue<int>("perception.card_min_cluster_size", CARD_DEFAULT_MIN_CLUSTER_SIZE);
    param_.max_cluster_size = s.getValue<int>("perception.card_max_cluster_size", CARD_DEFAULT_MAX_CLUSTER_SIZE);
    param_.point_dist_tolerance =
        s.getValue<float>("perception.card_point_dist_tolerance", CARD_DEFAULT_POINT_DIST_TOLERANCE);
    param_.threshold = s.getValue<int>("perception.card_threshold", CARD_DEFAULT_THRESHOLD);
    param_.normal_angle_tolerance =
        s.getValue<float>("perception.card_normal_angle_tolerance", CARD_DEFAULT_NORMAL_ANGLE_TOLERANCE);
    param_.both_pallet_dist_allow =
        s.getValue<float>("perception.card_both_pallet_dist_allow", CARD_DEFAULT_BOTH_PALLET_DIST_ALLOW);
    param_.up_extend_pixel = s.getValue<int>("perception.card_up_extend_pixel", CARD_DEFAULT_UP_EXTEND_PIXEL);
    param_.down_extend_pixel = s.getValue<int>("perception.card_down_extend_pixel", CARD_DEFAULT_DOWN_EXTEND_PIXEL);
    param_.min_area_rate = s.getValue<float>("perception.card_min_area_rate", CARD_DEFAULT_MIN_AREA_RATE);
    param_.max_area_rate = s.getValue<float>("perception.card_max_area_rate", CARD_DEFAULT_MAX_AREA_RATE);
    param_.hole_height_rate_diff_allow =
        s.getValue<float>("perception.card_hole_height_rate_diff_allow", CARD_DEFAULT_HOLE_HEIGHT_RATE_DIFF_ALLOW);
    param_.pallet_normal_diff_angle_allow = s.getValue<float>("perception.card_pallet_normal_diff_angle_allow",
                                                              CARD_DEFAULT_PALLET_NORMAL_DIFF_ANGLE_ALLOW);
    param_.enable_record_debug_image = (s.getValue<std::string>("perception.enable_record_debug_img_data", "False") == "True");

    param_.normal_groud_angle_tolerance = s.getValue<float>("perception.normal_ground_angle_tolerance", 90);
}

void CardDetectModule::loadTargetParam() {
    auto &s = sros::core::Settings::getInstance();
    camera_to_base_pose_[0] = s.getValue<double>("camera.o3d303_install_x_offset", 0) / 1000.0;
    camera_to_base_pose_[1] = s.getValue<double>("camera.o3d303_install_y_offset", 0) / 1000.0;
    camera_to_base_pose_[2] = s.getValue<double>("camera.o3d303_install_yaw", 0) / 180.0 * M_PI;
    camera_to_agv_roll_ = s.getValue<double>("camera.o3d303_install_roll", 0) / 180.0 * M_PI;
    camera_to_agv_pitch_ = s.getValue<double>("camera.o3d303_install_pitch", 0) / 180.0 * M_PI;
    camera_to_fork_height_ = s.getValue<double>("camera.o3d303_install_z_offset", 0) / 1000.0;

    auto get_card_param = [&](const std::string &card_name, const std::string &default_value) {
        auto first_pallet_info_str = s.getValue<std::string>(card_name, default_value);
        perception::Card card;
        LOG(INFO) << first_pallet_info_str;
        if (buildCardInfo(first_pallet_info_str, card)) {
            cards_map_.insert(std::make_pair(card.getId(), std::move(card)));
        }
    };

    get_card_param("perception.card_first_info", "0");
    get_card_param("perception.card_second_info", "0;");
    get_card_param("perception.card_third_info", "0;");
    get_card_param("perception.card_fourth_info", "0;");
    get_card_param("perception.card_fifth_info", "0;");
    get_card_param("perception.card_sixth_info", "0;");

    LOG(INFO) << "load cards size: " << cards_map_.size();
}

bool CardDetectModule::buildCardInfo(const std::string &info_strs, perception::Card &card_info) {
    auto infos = common_func::splitStringToVector<std::string>(info_strs, ';');
    if (infos.size() == 6) {
        card_info.setId(std::stoi(infos[0]));
        card_info.setLength(std::stof(infos[1]));
        card_info.setPalletHeight(std::stof(infos[2]));
        card_info.setHoleHeight(std::stof(infos[3]));
        std::vector<std::string> holes_width_str = common_func::splitStringToVector<std::string>(infos[4], ',');
        std::vector<std::string> pallets_width_str = common_func::splitStringToVector<std::string>(infos[5], ',');
        std::vector<float> holes_width;
        float width = .0f, temp;
        for (auto const &iter : holes_width_str) {
            temp = std::stof(iter);
            width += temp;
            holes_width.push_back(temp);
        }
        std::vector<float> pallets_width;
        for (auto const &iter : pallets_width_str) {
            temp = std::stof(iter);
            width += temp;
            pallets_width.push_back(temp);
        }
        card_info.setWidth(width);
        card_info.setHoleWidth(holes_width);
        card_info.setPalletWidth(pallets_width);
        LOG(INFO) << "card id=" << infos[0] << " length=" << infos[1] << " width=" << card_info.getWidth()
                  << " pallet_height=" << infos[2] << " hole_height=" << infos[3] << " holes_width[" << infos[4]
                  << "] pallets_width[" << infos[5] << "]";
        return true;
    } else {
        LOG(WARNING) << "pallet info is wrong!" << info_strs << "," << infos.size();
    }
    return false;
}

}  // namespace object_detector
