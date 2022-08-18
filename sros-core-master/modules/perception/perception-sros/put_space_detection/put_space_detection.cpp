/**
 * @file put_space_detector_module.cpp
 * @brief The class responsible for the card awareness module.
 *
 * If the parameter of main.enable_card_detect is set to true, the card sensing module will
 * be turned on and waiting for the collect to be DETECT_COMMAND and IFM3D_IMG message,
 * once received DETECT_COMMAND message, that is enable_img_process_ will be set to
 * true, the next IFM3D_IMG received will be processed by run(), than the card detection
 * program will be executed and the result will be sent to DETECT_RESULT topic.
 *
 * @author lijunhong@standard-robots.com
 * @date create date：2022/01/15
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "put_space_detection.h"
#include "../perception-sros/detector_base/unit.hpp"

#include "core/msg/image_msg.hpp"

#include "core/msg/common_msg.hpp"
#include "core/msg/perception_state_msg.hpp"
#include "core/msg/perception_command_msg.hpp"

#include "core/settings.h"
#include "core/module.h"
#include "../lib/object_detectors/card_detector/card.hpp"
#include "../lib/object_detectors/circle_detector/circle.hpp"
#include "../lib/object_detectors/handcart_detector/handcart.hpp"
#include "modules/perception/perception-sros/card_detection/card_detection.h"
#include "modules/perception/perception-sros/circle_detection/circle_detection.h"
#include "modules/perception/perception-sros/handcart_detection/handcart_detection.h"

// CODE
namespace object_detector {

    using PerceptionCommandMsg = sros::core::PerceptionCommandMsg;
    using PerceptionCommandMsgPtr = sros::core::PerceptionCommandMsgPtr;
    using PerceptionStateMsg = sros::core::PerceptionStateMsg;
    using PerceptionStateMsgPtr = sros::core::PerceptionStateMsgPtr;
    using Command = sros::core::PerceptionCommandMsg::Command;
    using DetectStage = sros::core::PerceptionCommandMsg::DetectStage;
    using ObjectType = sros::core::PerceptionCommandMsg::ObjectType;

    PutspaceDetectModule::PutspaceDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic)
            : DetectorModuleBase("PutspaceDetect", sendMsg, subscribeTopic) {

    }

    bool PutspaceDetectModule::isEnable() {
        auto &s = sros::core::Settings::getInstance();
        bool enable_unload_detect = (s.getValue<std::string>("main.enable_unload_detect", "False") == "True");
        return enable_unload_detect;
    }

    bool PutspaceDetectModule::init() {
        if (!isEnable()) {
            LOG(WARNING) << "PutspaceDetect module init fail running(disable)";
            return false;
        }

        loadDetectParam();
        loadTargetParam();

        if (!put_space_detector_.init(this->param_)) {
            LOG(ERROR) << "Put space Detector init fail";
            return false;
        }


        LOG(INFO) << "PutspaceDetect module start running";
        //subscribeTopic_("TIMER_50MS", CALLBACK(&PutspaceDetectModule::checkDetectTime));

        auto &s = sros::core::Settings::getInstance();
        is_mid70_laser_ = (s.getValue<std::string>("device.enable_livox_device", "False") == "True");
        if (is_mid70_laser_) {
            subscribeTopic_("LIVOX_MID70", CALLBACK(&PutspaceDetectModule::onHeadupMid70SensorMsg));
        } else {
            subscribeTopic_("IFM3D_IMG", CALLBACK(&PutspaceDetectModule::onSensorMsg));
        }


        is_init_detector_ = true;
        return true;
    }

    void PutspaceDetectModule::onDetectCommandMsg(const sros::core::base_msg_ptr &msg) {
        if (!is_init_detector_) {
            LOG(WARNING) << "put space detector is disable!";
            return;
        }

        // Analyse detect command and parameter.
        PerceptionCommandMsgPtr cmd = std::dynamic_pointer_cast<PerceptionCommandMsg>(msg);

        if (is_mid70_laser_) {
            if (cmd->command.object_type == ObjectType::OBJECT_TYPE_HEAPUP_CUBE) {
                LOG(INFO) << "subscribeTopic  LIVOX_MID70  onHeadupMid70SensorMsg";
                subscribeTopic_("LIVOX_MID70", CALLBACK(&PutspaceDetectModule::onHeadupMid70SensorMsg));
            } else if (cmd->command.object_type == ObjectType::OBJECT_TYPE_PUT_SPACE) {
                LOG(INFO) << "subscribeTopic  LIVOX_MID70  onPutSpaceMid70SensorMsg";
                subscribeTopic_("LIVOX_MID70", CALLBACK(&PutspaceDetectModule::onPutSpaceMid70SensorMsg));
            } else {
                LOG(INFO) << "subscribeTopic  LIVOX_MID70  onHeadupMid70SensorMsg";
                subscribeTopic_("LIVOX_MID70", CALLBACK(&PutspaceDetectModule::onHeadupMid70SensorMsg));
            }
//            float min_z = ((cmd->obj_center_ground_clearance - cmd->obj_self_height/2.0f) - cmd->current_camera_height)/1000.0f;
//            float max_z = ((cmd->obj_center_ground_clearance + cmd->obj_self_height/2.0f) - cmd->current_camera_height)/1000.0f;
//            this->param_.detect_range.min_z = min_z;
//            this->param_.detect_range.max_z = max_z;
//            put_space_detector_.setParam(this->param_);
        }

        LOGGER(INFO, ACTION_TASK) << "receive put space detect command : cmd:"
                                  << cmd->command.detect_stage * 10 + cmd->command.object_type
                                  << " pose:" << cmd->theory_pose.x() << "," << cmd->theory_pose.y() << ","
                                  << cmd->theory_pose.yaw() << " goal_id," << cmd->goal_id
                                  << " obj_center_ground_clearance:" << cmd->obj_center_ground_clearance
                                  << " obj_self_height:" << cmd->obj_self_height
                                  << " current_camera_height:" << cmd->current_camera_height
                                  << " detect_range.min_z:" << this->param_.detect_range.min_z
                                  << " detect_range.max_z:" << this->param_.detect_range.max_z;

//        this->put_space_type_ = cmd->command.object_type;
        this->theory_pose_[0] = cmd->theory_pose.x();
        this->theory_pose_[1] = cmd->theory_pose.y();
        this->theory_pose_[2] = cmd->theory_pose.yaw();
        this->command_ = cmd->command;
        this->goal_id_ = cmd->goal_id;
        this->is_receive_cmd_ = true;
        this->receive_cmd_time_ = sros::core::util::get_time_in_ms();

        if (!loadObjectArea(1, this->command_)) {
            LOGGER(INFO, ACTION_TASK) << "load object detect area fail!";
        }

        if (0 == this->goal_id_) {
            LOG(WARNING) << "cmd->goal_id = 0 (zero is invalid value)" << this->goal_id_;
        } else if (-1 == this->goal_id_) {
            startDetector();
            frame_queue_.clear();
        } else {
            LOG(WARNING) << "Command parameter error: cmd->goal_id_=" << cmd->goal_id;
            stopDetector();
            frame_queue_.clear();
        }
    }

    bool PutspaceDetectModule::loadObjectArea(const int object_index, const Command &cmd) {
        auto &s = sros::core::Settings::getInstance();

        if (ObjectType::OBJECT_TYPE_CARD == cmd.object_type) {
            std::map<int, perception::Card> cards_map;
            auto get_card_param = [&](const std::string &card_name, const std::string &default_value) {
                auto first_pallet_info_str = s.getValue<std::string>(card_name, default_value);
                perception::Card card;
                LOG(INFO) << first_pallet_info_str;
                if (CardDetectModule::buildCardInfo(first_pallet_info_str, card)) {
                    cards_map.insert(std::make_pair(card.getId(), std::move(card)));
                }
            };

            get_card_param("perception.card_first_info", "0");
            get_card_param("perception.card_second_info", "0;");
            get_card_param("perception.card_third_info", "0;");
            get_card_param("perception.card_fourth_info", "0;");
            get_card_param("perception.card_fifth_info", "0;");
            get_card_param("perception.card_sixth_info", "0;");
            LOG(INFO) << "load cards size: " << cards_map.size();

            if (-1 == object_index) {
                for (auto const &c: cards_map) {
                    object_length_ = object_length_ < c.second.getLength() ? c.second.getLength() : object_length_;
                    object_width_ = object_width_ < c.second.getWidth() ? c.second.getWidth() : object_width_;
                }
            } else if (object_index > 0 && cards_map.find(object_index) != cards_map.end()) {
                object_length_ = cards_map[object_index].getLength();
                object_width_ = cards_map[object_index].getWidth();
            } else {
                LOG(WARNING) << "The card target index was not found in the target list";
                return false;
            }
            LOG(INFO) << "object_index: " << object_index << " length " << object_length_ << ", width "
                      << object_width_;
            return true;
        } else if (ObjectType::OBJECT_TYPE_CIRCLE == cmd.object_type) {
            std::map<int, perception::Circle> circles_map;
            auto get_circle_param = [&](const std::string &name, const std::string &default_value) {
                auto first_circle_info_str = s.getValue<std::string>(name, default_value);
                perception::Circle circle;
                LOG(INFO) << first_circle_info_str;
                if (CircleDetectModule::buildCircleInfo(first_circle_info_str, circle)) {
                    circles_map.insert(std::make_pair(circle.getId(), std::move(circle)));
                }
            };
            get_circle_param("perception.circle_disk_info", "0");
            LOG(INFO) << "load circles size: " << circles_map.size();

            if (-1 == object_index) {
                for (auto const &c: circles_map) {
                    object_length_ = object_length_ < c.second.getLength() ? c.second.getLength() : object_length_;
                    object_width_ = object_width_ < c.second.getWidth() ? c.second.getWidth() : object_width_;
                }
            } else if (object_index > 0 && circles_map.find(object_index) != circles_map.end()) {
                object_length_ = circles_map[object_index].getLength();
                object_width_ = circles_map[object_index].getWidth();
            } else {
                LOG(WARNING) << "The circle target index was not found in the target list";
                return false;
            }
            LOG(INFO) << "object_index: " << object_index << " length " << object_length_ << ", width "
                      << object_width_;
            return true;
        } else if (ObjectType::OBJECT_TYPE_HANDCART == cmd.object_type) {
            std::map<int, perception::Handcart> handcarts_map;
            auto get_handcart_param = [&](const std::string &name, const std::string &default_value) {
                auto handcart_info_str = s.getValue<std::string>(name, default_value);
                perception::Handcart handcart;
                LOG(INFO) << handcart_info_str;
                if (HandcartDetectModule::buildHandcartInfo(handcart_info_str, handcart)) {
                    handcarts_map.insert(std::make_pair(handcart.getId(), std::move(handcart)));
                }
            };
            get_handcart_param("perception.handcart_info", "0");
            LOG(INFO) << "load handcarts size: " << handcarts_map.size();

            if (-1 == object_index) {
                for (auto const &c: handcarts_map) {
                    object_length_ = object_length_ < c.second.getLength() ? c.second.getLength() : object_length_;
                    object_width_ = object_width_ < c.second.getWidth() ? c.second.getWidth() : object_width_;
                }
            } else if (object_index > 0 && handcarts_map.find(object_index) != handcarts_map.end()) {
                object_length_ = handcarts_map[object_index].getLength();
                object_width_ = handcarts_map[object_index].getWidth();
            } else {
                LOG(WARNING) << "The handcart target index was not found in the target list";
                return false;
            }
            LOG(INFO) << "object_index: " << object_index << " length " << object_length_ << ", width "
                      << object_width_;
            return true;
        }

        return false;
    }

    void PutspaceDetectModule::onSensorMsg(const sros::core::base_msg_ptr &msg) {
        //    auto begin_time = sros::core::util::get_time_in_us();
        LOG(ERROR) << "@001 receive 03d3xx(" << msg->time_ << ") frame time point: "
                   << sros::core::util::get_time_in_us();

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

        if (this->is_record_) {
            recordSensorData("/sros/log/", o3d3xx_frame);
        }

        int64_t start = common_func::get_time_in_ms();
        DetectResult result = put_space_detector_.detect(o3d3xx_frame);
        int64_t finish = common_func::get_time_in_ms();
        showResult(result, finish - start);

        if (result.is_available) {
            Eigen::Vector3f card_pose(result.x, result.y, result.angle);
            detected_poses_.push_back(card_pose);
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
        LOG(INFO) << "     theory in world pose: " << theory_pose_[0] << " ," << theory_pose_[1] << " ,"
                  << theory_pose_[2];

        stopDetector();
        sendResultMsg(result_msg);

        //    auto end_time = sros::core::util::get_time_in_us();
        //    LOG(ERROR) << "process 03d3xx("<<msg->time_<<") frame consume time : " << (end_time-begin_time) / 1.0e6;
        //    LOG(ERROR) << "--------------------------------------------------------------------";
    }

    static void convert2PointCloud(const sros::core::PointCloudMsg &frame, const PointCloudPtr &cloud) {
        cloud->reserve(frame.cloud.size());
        int index = 0;
        for (auto const &point: frame.cloud) {
            cloud->points.emplace_back(point.x(), point.y(), point.z());
            cloud->image_indices[index] = index;
            ++index;
        }
        cloud->time = frame.seq;
    }

    void
    PutspaceDetectModule::onPutSpaceMid70SensorMsg(const sros::core::base_msg_ptr &msg) {

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
        if (QUEUE_SIZE < ACCUMULATE_QUEUE_SIZE_COUNT) {
            if (0 == QUEUE_SIZE % 100) {
                LOG(WARNING) << "Put space receive frame_queue_.size=" << std::setw(4) << std::setfill(' ')
                             << QUEUE_SIZE;
            }
            return;
        }

        // The accumulated multi frame data is merged together.
        int index = 0;
        PointCloudPtr multi_cloud(new PointCloud());
        multi_cloud->resize(frame_queue_.size() * 96);
        for (auto const &f: frame_queue_) {
            for (auto const &point: f->points) {
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


        // get current pose of agv
        slam::tf::TransForm curr_pose;
        if (!tf_base_to_world_->lookUpTransForm(msg->time_, curr_pose, 50000)) {
            LOG(INFO) << "delta time is wrong! cannot get real time pose!";
            return;
        }

        int64_t start = common_func::get_time_in_ms();
        putSpaceDetect(curr_pose, multi_cloud);
        int64_t finish = common_func::get_time_in_ms();

    }


    void
    PutspaceDetectModule::onHeadupMid70SensorMsg(const sros::core::base_msg_ptr &msg) {
        //    auto begin_time = sros::core::util::get_time_in_us();
        //    LOG(ERROR) << "@001 receive livox("<<msg->time_<<") frame time point: " << sros::core::util::get_time_in_us();
        //    enable_process_ = true; // 调试状态,确保每次都识别展板
        if (!enable_process_) {
            return;
        }

        detected_count_++;
        auto frame = std::dynamic_pointer_cast<sros::core::PointCloudMsg>(msg);

        if (frame->seq == last_frame_time_) {
//        LOG(INFO) << "current frame time == last frame_time.";
            return;
        }

        PointCloudPtr cloud(new PointCloud());
        convert2PointCloud(*frame, cloud);

        if (cloud->empty()) {
            LOG(WARNING) << "receive cloud.size=0";
        } else {
            frame_queue_.push_back(cloud);
            last_frame_time_ = frame->seq;
        }

        // accumulated frame.
        const int QUEUE_SIZE = frame_queue_.size();
        if (QUEUE_SIZE < ACCUMULATE_QUEUE_SIZE_COUNT) {
            if (0 == QUEUE_SIZE % 100) {
                LOG(WARNING) << "receive frame_queue_.size=" << std::setw(4) << std::setfill(' ') << QUEUE_SIZE;
            }
            return;
        }

        // The accumulated multi frame data is merged together.
        int index = 0;
        PointCloudPtr multi_cloud(new PointCloud());
        multi_cloud->resize(frame_queue_.size() * 96);
        for (auto const &f: frame_queue_) {
            for (auto const &point: f->points) {
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


        int64_t start = common_func::get_time_in_ms();
        DetectResult result = put_space_detector_.detect(multi_cloud, 1);
        int64_t finish = common_func::get_time_in_ms();
        showResult(result, finish - start);

        if (result.is_available) {
            result.z -= tan(laser_install_pitch_) * result.x; // 修正传感器pitch角度误差
            Eigen::Vector4f card_pose(result.x, result.y, result.z, result.angle);
            detected_xyzyaw_.push_back(card_pose);
            this->goal_id_ = result.id;
        }

//        if (detected_count_ < DETECTED_COUNT_THRESH) {
//            return;
//        }

        if (this->is_record_ && out_put_.is_open()) {
            LOG(INFO) << "close sensor data file: " << this->sensor_data_file_path_;
            out_put_.close();
        }

        // If no target is detected, send (0,0,0) and return.
        auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
        result_msg->command = this->command_;
        result_msg->goal_in_global_pose.x() = .0f;
        result_msg->goal_in_global_pose.y() = .0f;
        result_msg->goal_in_global_pose.z() = .0f;
        result_msg->goal_in_global_pose.yaw() = .0f;
        result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
        result_msg->error_code = PerceptionStateMsg::ERROR_CODE_NOT_FIND_GOAL;
        if (detected_xyzyaw_.empty()) {
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
        auto mean_xyzyaw = computeTargetMeanPose(detected_xyzyaw_);

        // else init the detect result message and send to topic:DETECT_RESULT
        slam::tf::TransForm curr_pose;
        if (!tf_base_to_world_->lookUpTransForm(msg->time_, curr_pose, 50000)) {
            LOG(INFO) << "delta time is wrong! cannot get real time pose!";
            stopDetector();
            sendResultMsg(result_msg);
            return;
        }
        Eigen::Vector3f mean_pose{mean_xyzyaw.x(), mean_xyzyaw.y(), mean_xyzyaw.w()};
        Eigen::Vector3f agv_in_world_tf(curr_pose.position.x(), curr_pose.position.y(), curr_pose.rotation.yaw());
        auto camera_in_world_pose = common_func::cvtToWorldPose(agv_in_world_tf, camera_to_base_pose_);
        auto goal_in_world_pose = common_func::cvtToWorldPose(camera_in_world_pose, mean_pose);
        auto goal_in_agv_pose = common_func::cvtToWorldPose(camera_to_base_pose_, mean_pose);
        result_msg->goal_in_global_pose.x() = goal_in_world_pose[0];
        result_msg->goal_in_global_pose.y() = goal_in_world_pose[1];
        result_msg->goal_in_global_pose.yaw() = goal_in_world_pose[2];
        result_msg->goal_in_global_pose.z() =
                mean_xyzyaw.z() + g_state.fork_height_encoder + laser_install_fork_z_offset_;

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
                  << ", " << goal_in_agv_pose[2] << ", " << result_msg->goal_in_global_pose.z();
        LOG(INFO) << "     theory in world pose: " << theory_pose_[0] << " ," << theory_pose_[1] << " ,"
                  << theory_pose_[2];

        stopDetector();
        sendResultMsg(result_msg);

        //    auto end_time = sros::core::util::get_time_in_us();
        //    LOG(ERROR) << "process 03d3xx("<<msg->time_<<") frame consume time : " << (end_time-begin_time) / 1.0e6;
        //    LOG(ERROR) << "--------------------------------------------------------------------";
    }

    void PutspaceDetectModule::loadDetectParam() {
        using namespace perception;

        // Load detect common parameter
        sros::core::Settings &s = sros::core::Settings::getInstance();
        common_func::loadCommonParam<PutspaceDetectParam>(param_);
        is_record_ = (s.getValue<std::string>("perception.is_record", "True") == "True");

        // load detect range parameter.
        std::vector<float> range3d_str_vct;
        const int range3d_param_num = sizeof(param_.detect_range) / sizeof(float);
        const int decode_param_num = common_func::decodeStringParam("perception.headup_range3d", range3d_str_vct);
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
                s.getValue<float>("perception.card_hole_height_rate_diff_allow",
                                  CARD_DEFAULT_HOLE_HEIGHT_RATE_DIFF_ALLOW);
        param_.pallet_normal_diff_angle_allow = s.getValue<float>("perception.card_pallet_normal_diff_angle_allow",
                                                                  CARD_DEFAULT_PALLET_NORMAL_DIFF_ANGLE_ALLOW);

        param_.min_cloud_size = s.getValue<int>("perception.headup_min_cloud_size",
                                                PUTSPACE_HEADUP_DEFAULT_MIN_CLOUD_SIZE);
        param_.max_cloud_size = s.getValue<int>("perception.headup_max_cloud_size",
                                                PUTSPACE_HEADUP_DEFAULT_MAX_CLOUD_SIZE);
        param_.front_normal_angle_tolerance = s.getValue<float>("perception.headup_front_normal_angle_tolerance",
                                                                PUTSPACE_HEADUP_DEFAULT_FRONT_NORMAL_ANGLE_TOLERANCE);
        param_.top_normal_angle_tolerance = s.getValue<float>("perception.headup_top_normal_angle_tolerance",
                                                              PUTSPACE_HEADUP_DEFAULT_TOP_NORMAL_ANGLE_TOLERANCE);
        param_.enable_record_debug_image = (
                s.getValue<std::string>("perception.enable_record_debug_img_data", "False") == "True");
        param_.occupy_size_threshold = s.getValue<int>("occupy_size_threshold", 100);

        // load detector common parameter.
        auto laser_install_x_offset = s.getValue<float>("perception.livox_install_x_offset", -50) * 0.001;
        auto laser_install_y_offset = s.getValue<float>("perception.livox_install_y_offset", 0) * 0.001;
        auto laser_install_fork_z_offset = s.getValue<float>("perception.livox_install_fork_z_offset", -90) * 0.001;
        auto laser_install_yaw = s.getValue<float>("perception.livox_install_yaw", 180) * M_PI / 180;
        auto laser_install_pitch = s.getValue<float>("perception.livox_install_pitch", 0) * M_PI / 180;

        Eigen::Translation2d translation2D(laser_install_x_offset, laser_install_y_offset);
        Eigen::Rotation2Dd rotation2Dd(laser_install_yaw);
        scan_to_agv_tf_ = translation2D * rotation2Dd;
    }

    void PutspaceDetectModule::startDetector() {
        detected_poses_.clear();
        detected_xyzyaw_.clear();
        enable_process_ = true;
        detected_count_ = 0;
        enableSensor();
    }

    void PutspaceDetectModule::stopDetector() {
        detected_poses_.clear();
        detected_xyzyaw_.clear();
        enable_process_ = false;
        detected_count_ = 0;
        disableSensor();
    }

    void PutspaceDetectModule::loadTargetParam() {
        auto &s = sros::core::Settings::getInstance();
//        camera_to_base_pose_[0] = s.getValue<double>("camera.o3d303_install_x_offset", 0) / 1000.0;
//        camera_to_base_pose_[1] = s.getValue<double>("camera.o3d303_install_y_offset", 0) / 1000.0;
//        camera_to_base_pose_[2] = s.getValue<double>("camera.o3d303_install_yaw", 0) / 180.0 * M_PI;

        camera_to_base_pose_[0] = s.getValue<float>("perception.livox_install_x_offset", -50) * 0.001;
        camera_to_base_pose_[1] = s.getValue<float>("perception.livox_install_y_offset", 0) * 0.001;
        camera_to_base_pose_[2] = s.getValue<float>("perception.livox_install_yaw", 180) * M_PI / 180;
        laser_install_fork_z_offset_ = s.getValue<float>("perception.livox_install_fork_z_offset", -90) * 0.001;
        laser_install_pitch_ = s.getValue<float>("perception.livox_install_pitch", 0) * M_PI / 180;


    }


    void PutspaceDetectModule::enableSensor() {

        if (is_mid70_laser_) {
            LOG(INFO) << "@001 send TOPIC_LIVOX_ENABLE_PUBLISH msg";
            std::shared_ptr <sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_LIVOX_ENABLE_PUBLISH"));
            cmd_msg->str_0_ = sros::device::DEVICE_LIDAR_LIVOX;
            cmd_msg->flag = true;
            sendMsg_(cmd_msg);
        } else {
            LOG(INFO) << "@001 send TOPIC_SVC100_ENABLE_PUBLISH msg";
            std::shared_ptr <sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
            cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_O3D303;
            cmd_msg->flag = true;
            sendMsg_(cmd_msg);
        }

    }

    void PutspaceDetectModule::disableSensor() {

        if (is_mid70_laser_) {
            LOG(INFO) << "send TOPIC_LIVOX_ENABLE_PUBLISH msg";
            std::shared_ptr <sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_LIVOX_ENABLE_PUBLISH"));
            cmd_msg->str_0_ = sros::device::DEVICE_LIDAR_LIVOX;
            cmd_msg->flag = false;
            sendMsg_(cmd_msg);
        } else {
            LOG(INFO) << "send TOPIC_SVC100_ENABLE_PUBLISH msg";
            std::shared_ptr <sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
            cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_O3D303;
            cmd_msg->flag = false;
            sendMsg_(cmd_msg);
        }
    }

    bool PutspaceDetectModule::putSpaceDetect(slam::tf::TransForm &curr_pose, const PointCloudPtr &cloud) {

        Eigen::Affine2d world_tf = Eigen::Translation2d(curr_pose.position.x(), curr_pose.position.y()) *
                                   Eigen::Rotation2Dd(curr_pose.rotation.yaw());
        auto scan_to_world_tf = world_tf * scan_to_agv_tf_;

        std::vector <Eigen::Vector3d> laser_points;
        convertScanToWorld(scan_to_world_tf, param_.detect_range, cloud, laser_points);

        this->cur_pose_[0] = curr_pose.position.x();
        this->cur_pose_[1] = curr_pose.position.y();
        this->cur_pose_[2] = curr_pose.rotation.yaw();
        calculateDetectCubeRange(this->cur_pose_, cube_range_);

        auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
        result_msg->goal_id = this->goal_id_;
        result_msg->command = this->command_;
        if (isRectangleRegionOccupy(this->cube_range_, laser_points)) {
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_EXISTENCE_OBSTACLE;
            LOG(INFO) << "put space detect is exit Occupy";
        } else {
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_SUCCESS;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_INVALID;
            LOG(INFO) << "put space detect is not exit Occupy";
        }
        sendResultMsg(result_msg);
        return true;
    }

    bool PutspaceDetectModule::isRectangleRegionOccupy(const Range3D<float> &cube_range,
                                                       const std::vector <Eigen::Vector3d> &laser_points) {
        int occupy_size = 0;
        for (const auto &point: laser_points) {
            if (IS_WITHIN_SCOPE(point.x(), cube_range.min_x, cube_range.max_x)
                && IS_WITHIN_SCOPE(point.y(), cube_range.min_y, cube_range.max_y)) {
                ++occupy_size;
            }
        }
        LOG(INFO) << "put space detect occupy size :" << occupy_size;
        if (occupy_size < param_.occupy_size_threshold) {
            return false;
        } else {
            return true;
        }
    }


    void PutspaceDetectModule::calculateDetectCubeRange(const Eigen::Vector3d &cur_pose, Range3D<float> &cube_range) {
        auto &s = sros::core::Settings::getInstance();
        auto lateral_expansion_distance = s.getValue<float>("perception.lateral_expansion_distance", 0.05);
        auto longitudinal_expansion_distance = s.getValue<float>("perception.longitudinal_expansion_distance", 0.1);
        auto updown_expansion_distance = s.getValue<float>("perception.updown_expansion_distance", 0.01);

        auto total_length = (cur_pose - theory_pose_).norm();
        auto m1_rate = 1 - ((this->object_length_ / 2) + longitudinal_expansion_distance) / total_length;
        auto m2_rate = 1 + ((this->object_length_ / 2) + longitudinal_expansion_distance) / total_length;
        LOG(INFO) << "cur_pose(" << cur_pose.x() << ", " << cur_pose.y() << ", " << cur_pose[2]
                  << ") theory_pose(" << theory_pose_.x() << ", " << theory_pose_.y() << ", " << theory_pose_[2]
                  << ") total_length=" << total_length;

        // 区分近处的点与远处的点,方便后期扩大检测区域的范围
        Eigen::Vector3d near, far;
        near.x() = cur_pose.x() + (theory_pose_.x() - cur_pose.x()) * m1_rate;
        near.y() = cur_pose.y() + (theory_pose_.y() - cur_pose.y()) * m1_rate;
        far.x() = cur_pose.x() + (theory_pose_.x() - cur_pose.x()) * m2_rate;
        far.y() = cur_pose.y() + (theory_pose_.y() - cur_pose.y()) * m2_rate;
        LOG(INFO) << "near(" << near.x() << ", " << near.y() << ") far(" << far.x() << ", " << far.y() << ")";

        // 区域延展到车体下面
        m1_rate = (total_length + param_base_.fork_end.x()) / total_length;
        near.x() = theory_pose_.x() + (cur_pose.x() - theory_pose_.x()) * m1_rate;
        near.y() = theory_pose_.y() + (cur_pose.y() - theory_pose_.y()) * m1_rate;
        LOG(INFO) << "after extend: near(" << near.x() << ", " << near.y() << ") far(" << far.x() << ", " << far.y()
                  << ")";

        auto width_x_weight = ((this->object_width_ / 2) + lateral_expansion_distance) * sin(theory_pose_[2]);
        auto width_y_weight = ((this->object_width_ / 2) + lateral_expansion_distance) * cos(theory_pose_[2]);


        float min_x = near.x() < far.x() ? near.x() : far.x();
        float max_x = near.x() < far.x() ? far.x() : near.x();
        float min_y = near.y() < far.y() ? near.y() : far.y();
        float max_y = near.y() < far.y() ? far.y() : near.y();
        cube_range.min_x = min_x - width_x_weight;
        cube_range.max_x = max_x + width_x_weight;
        cube_range.min_y = min_y - width_y_weight;
        cube_range.max_y = max_y + width_y_weight;
        cube_range.min_z = -updown_expansion_distance;
        cube_range.max_z = +updown_expansion_distance;


        LOGGER(INFO, ACTION_TASK) << " min_x:" << cube_range.min_x << " max_x:" << cube_range.max_x
                                  << " min_y:" << cube_range.min_y << " max_y:" << cube_range.max_y
                                  << " min_z:" << cube_range.min_z << " max_z:" << cube_range.max_z;
    }


    void PutspaceDetectModule::convertScanToWorld(const Eigen::Affine2d &world_tf,
                                                  const Range3D<float> &range,
                                                  const PointCloudPtr &cloud,
                                                  std::vector <Eigen::Vector3d> &laser_points) {
        laser_points.reserve(cloud->points.size());
        for (const auto &point: cloud->points) {
            if (IS_WITHIN_SCOPE(point.x, range.min_x, range.max_x)
                && IS_WITHIN_SCOPE(point.y, range.min_y, range.max_y)
                && IS_WITHIN_SCOPE(point.z, range.min_z, range.max_z)) {
                Eigen::Vector2d curr_point(point.x, point.y);
                auto world_point = world_tf * curr_point;
                laser_points.emplace_back(world_point[0], world_point[1], point.z);
            }
        }
        LOG(INFO) << "Mid70 Laser filtered pint sizie:" << laser_points.size();
    }


}  // namespace object_detector
