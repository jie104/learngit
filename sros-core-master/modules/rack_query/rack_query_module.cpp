//
// Created by getup on 18-12-13.
//

#include "rack_query_module.h"
#include <core/settings.h>
#include <core/src.h>
#include <core/msg/data_matrix_code_msg.hpp>
#include <core/msg/posture_correct_command_msg.hpp>
#include "core/tf/TFOperator.h"
#include "core/state.h"

using namespace sros::core;

namespace rack_query {
static void convertDmToStandardHandSystem(const sros::core::DataMatrixCodeMsg_ptr& dm_pose,sros::core::DataMatrixCodeMsg_ptr& right_pose){
    //目前这种坐标系变换只针对下视摄像头，为了兼容之前其他传感器数据。以后，内循环系统内的摄像头全部都是标准右手系，不需要变换。
    if (dm_pose == right_pose) {
        LOG(INFO) << "cannot convert same ptr!";
    }
    *right_pose = *dm_pose;
    if (dm_pose->right_handed_system) {
        right_pose->x_ = dm_pose->y_;
        right_pose->y_ = -dm_pose->x_;
        right_pose->angle_ = (dm_pose->angle_ - M_PI_2);
    }else{
        right_pose->x_ = dm_pose->x_;
        right_pose->y_ = -dm_pose->y_;
        right_pose->angle_ = -dm_pose->angle_;
    }
}

RackQueryModule::RackQueryModule() : Module("RackQuery") {
    process_state_ = STATE_IDLE_PROCESS;
    slam::tf::FrameToFrame base_to_world_frame;
    base_to_world_frame.parent_frame = "world";
    base_to_world_frame.child_frame = "base_link";
    tf_base_to_world_.reset(new slam::tf::TFOperator(base_to_world_frame));
}

void RackQueryModule::run() {
    auto &s = sros::core::Settings::getInstance();
    enable_rack_query_ = (s.getValue<std::string>("main.enable_rack_query", "False") == "True");
//    if (!enable_rack_query_) {
//        LOG(WARNING) << "RackQuery module stop running(disable)";
//        stop();
//        return;
//    }
    updatePara();
    LOG(INFO) << "RackQuery module start running";
    subscribeTopic("RACK_QUERY_COMMAND", CALLBACK(&RackQueryModule::onNavigationCommandMsg));
    subscribeTopic("TOPIC_LASER", CALLBACK(&RackQueryModule::onLaserScanMsg));
    subscribeTopic("TOPIC_POSTURE_CORRECT_CMD", CALLBACK(&RackQueryModule::onPostureReceive));
    subscribeTopic("DM_CODE_INFO", CALLBACK(&RackQueryModule::onDmCodeMsg));
    subscribeTopic("TIMER_100MS", CALLBACK(&RackQueryModule::onTimer_100ms));
    dispatch();
}

void RackQueryModule::onNavigationCommandMsg(sros::core::base_msg_ptr msg) {
    std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd =
        std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::string>>(msg);
    LOG(INFO) << "cmd:" << cmd->command;
    if (cmd->command == "RACK_DETECT") {
        LOG(INFO) << "begin to detect!";
        resetDetectPara(cmd);
        process_state_ = STATE_DETECT_PROCESS;
    } else if (cmd->command == "RACK_QUERY") {
        resetQueryPara(cmd);
        process_state_ = STATE_QUERY_PROCESS;
    } else {
        process_state_ = STATE_IDLE_PROCESS;
    }
}

void RackQueryModule::onLaserScanMsg(sros::core::base_msg_ptr msg) {
    switch (process_state_) {
        case STATE_IDLE_PROCESS:
            break;
        case STATE_DETECT_PROCESS:
            processDetect(msg);
            break;
        case STATE_QUERY_PROCESS:
            processQuery(msg);
            break;
        default:
            break;
    }
    return;
}

void RackQueryModule::processDetect(sros::core::base_msg_ptr msg) {
    auto scan_ptr = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(msg);
    if (detect_count_ < detect_count_thresh_) {
        slam::tf::TransForm curr_pose;
        if (!tf_base_to_world_->lookUpTransForm(scan_ptr->time_, curr_pose, 50000)) {
            LOG(INFO) << "delta time is wrong! cannot get real time pose!";
            return;
        }
        Eigen::Vector3d curr_eigen_pose(curr_pose.position.x(), curr_pose.position.y(), curr_pose.rotation.yaw());

        detect_count_++;
        std::vector<RackInfo> detect_racks;
        std::vector<Eigen::Vector3d> detect_center_poses;
        detector_.detectRack(scan_ptr, detect_racks, detect_center_poses);
        RackInfoWithPose rack_info_with_pose;
        LOG(INFO) << "detect!";
        if (findRightRackInfo(cvtToWorldPose(curr_eigen_pose, scan_pose_), detect_racks, detect_center_poses,
                              true_rack_pose_, rack_info_with_pose)) {
            LOG(INFO) << "the leg:" << rack_infos_[0].leg_d << "," << rack_infos_[0].leg_groups[0].length << ","
                      << rack_infos_[0].leg_groups[0].width;
            //            LOG(INFO) << "pose: angle:" << rack_info_with_pose.pose[2] << "center pose:" <<
            //            detect_center_poses[0][2];
            world_rack_infos_.push_back(rack_info_with_pose);
        } else {
            LOG(INFO) << "cannot detect!";
            if (rack_infos_.size()) {
                for (auto &rack : rack_infos_) {
                    if (!rack.leg_groups.size()) {
                        LOG(WARNING) << "the leg is wrong!";
                    } else {
                        LOG(INFO) << "the leg:" << rack.leg_d << "," << rack.leg_groups[0].length << ","
                                  << rack.leg_groups[0].width;
                        LOG(INFO) << "not detect";
                    }
                }
            }
        }

    } else {
        auto station_msg = std::make_shared<sros::core::CommonCommandMsg<std::vector<RackInfo>>>("RACK_INFO");
        if (world_rack_infos_.size()) {
            auto mean_rack_info = computeMeanRack(world_rack_infos_);
            double rack_direction = mean_rack_info.pose[2] + M_PI_2;
            double delta_angle = true_rack_pose_[2] - rack_direction;
            normalizeAngle(delta_angle);
            if (fabs(delta_angle) > M_PI_2) {
                rack_direction -= M_PI;
            }
            LOG(INFO) << "rack direction:" << rack_direction << "," << mean_rack_info.pose[2] << ","
                      << true_rack_pose_[2];
            station_msg->pose.x() = mean_rack_info.pose[0];
            station_msg->pose.y() = mean_rack_info.pose[1];
            //            station_msg->pose.yaw() = rack_direction;
            station_msg->pose.yaw() = mean_rack_info.pose[2];
            station_msg->command.push_back(mean_rack_info.rack_info);
            LOG(INFO) << "send back!";
        } else {
            LOG(INFO) << "cannot detect rack!";
        }
        sendMsg(station_msg);
        world_rack_infos_.clear();  //在清空之前,需要计算当前中心位姿
        detect_count_ = 0;
        process_state_ = STATE_IDLE_PROCESS;
    }
}

void RackQueryModule::processQuery(sros::core::base_msg_ptr msg) {
    auto scan_ptr = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(msg);
}

std::vector<double> RackQueryModule::splitStrsToDoubles(const std::string &s, const char seperator) {
    std::vector<double> result;
    std::string::size_type i = 0;
    std::string::size_type j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                std::string item_s = s.substr(j, len);
                if (item_s == "\"") break;
                try {
                    if (item_s.size()) {
                        double item = stod(item_s);
                        result.push_back(item);
                    }
                } catch (std::exception &e) {
                    LOG(ERROR) << "throw error:" << e.what() << item_s;
                }
            }
            j = i + 1;
        }
        i++;
    }

    return result;
}

std::vector<std::string> RackQueryModule::splitStrsToStrs(const std::string &s, const char seperator) {
    std::vector<std::string> result;
    std::string::size_type i = 0;
    std::string::size_type j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                std::string item_s = s.substr(j, len);
                if (item_s == "\"") break;
                result.push_back(item_s);
            }
            j = i + 1;
        }
        i++;
    }

    return result;
}

void RackQueryModule::updatePara() {
    auto &s = sros::core::Settings::getInstance();
    RackInfo curr_rack;
    rack_infos_.clear();

    scan_pose_[0] = s.getValue("posefilter.laser_coordx", 0.29);
    scan_pose_[1] = s.getValue("posefilter.laser_coordy", 0.0);
    scan_pose_[2] = s.getValue("posefilter.laser_coordyaw", 0.0);
    LOG(INFO) << "scan info:" << scan_pose_[0] << "," << scan_pose_[1] << "," << scan_pose_[2];
    auto rack_info_strs = (s.getValue<std::string>("rack.rack_infos", "0"));
    auto racks = splitStrsToStrs(rack_info_strs, ';');
    for (auto &rack : racks) {
        auto rack_values = splitStrsToDoubles(rack, ',');
        int value_size = rack_values.size();
        if (value_size % 2 == 0) {  // value size不能为偶数,否则无法完成赋值
            LOG(INFO) << "the value format is wrong:" << rack_info_strs;
            continue;
        }
        if (value_size <= 2) {
            LOG(INFO) << "rack para is wrong!" << rack;
            continue;
        }
        int first_leg_index = 3;
        if (value_size == 3) {
            LOG(INFO) << "value size is smaller than three:" << value_size;
            first_leg_index = 1;  //这里记录该货架的避障尺寸
        }
        rack_infos_.emplace_back();
        const double MM_TO_M = 0.001;
        rack_infos_.back().leg_d = rack_values[0] * MM_TO_M;
        auto leg_radius = rack_infos_.back().leg_d;
        for (int i = first_leg_index; i < value_size; i += 2) {
            rack_infos_.back().leg_groups.emplace_back();
            rack_infos_.back().leg_groups.back().length = rack_values[i] * MM_TO_M;
            if (i + 1 < value_size) {
                rack_infos_.back().leg_groups.back().width = rack_values[i + 1] * MM_TO_M;
            } else {
                LOG(INFO) << "rack info is wrong!";
            }
        }
    }
    //    if (rack_infos_.empty()) {
    LOG(INFO) << "will use default laser rack info!";
    double rack_leg_center_length = s.getValue<double>("rack.rack_leg_center_length", 1060) * sros::core::MM_TO_M;
    double rack_leg_center_width = s.getValue<double>("rack.rack_leg_center_width", 600) * sros::core::MM_TO_M;
    double rack_leg_diameter = s.getValue<double>("rack.rack_leg_diameter", 100) * sros::core::MM_TO_M;
    rack_infos_.emplace_back();
    rack_infos_.back().leg_d = rack_leg_diameter;
    rack_infos_.back().leg_groups.emplace_back();
    rack_infos_.back().leg_groups.back().length = rack_leg_center_length;
    rack_infos_.back().leg_groups.back().width = rack_leg_center_width;
    //    }
    for (auto &rack : rack_infos_) {
        LOG(INFO) << "rack info:" << rack.leg_d << "," << rack.leg_groups.back().length << ","
                  << rack.leg_groups.back().width;
        detector_.addRackInfo(rack);
    }
    detect_count_ = 0;
}

bool RackQueryModule::findRightRackInfo(const Eigen::Vector3d &world_pose, const std::vector<RackInfo> &rack,
                                        const std::vector<Eigen::Vector3d> &rack_poses, Eigen::Vector3d &right_pose,
                                        RackQueryModule::RackInfoWithPose &rack_with_pose) {
    int size = rack.size();
    double min_dist = std::numeric_limits<double>::max();
    int index = 0;
    for (int i = 0; i < size; ++i) {
        auto curr_angle = rack_poses[i][2];
        curr_angle += M_PI;
        normalizeAngle(curr_angle);
        if (fabs(curr_angle) > rack_angle_thresh) {
            LOG(INFO) << "rack angle large than 20°! rack pose:" << rack_poses[i][0] << "," << rack_poses[i][1] << ","
                      << rack_poses[i][2] << "delta angle:" << curr_angle * 180 / M_PI << "°";
            continue;
        }

        auto curr_rack_pose = cvtToWorldPose(world_pose, rack_poses[i]);
        double dist = distance(right_pose, curr_rack_pose);
        if (dist < min_dist) {
            min_dist = dist;
            index = i;
        }
    }
    if (min_dist < rack_dist_thresh_) {
        rack_with_pose.pose = cvtToWorldPose(world_pose, rack_poses[index]);
        rack_with_pose.rack_info = rack[index];

        auto curr_angle = rack_poses[index][2];
        curr_angle += M_PI;
        normalizeAngle(curr_angle);
        LOG(INFO) << "true rack pose:" << rack_poses[index][0] << "," << rack_poses[index][1] << ","
                  << rack_poses[index][2] << ",direction:" << curr_angle * 180 / M_PI << "°"
                  << "pose," << rack_with_pose.pose[0] << "," << rack_with_pose.pose[1] << ","
                  << rack_with_pose.pose[2];

        return true;
    }
    return false;
}

void RackQueryModule::resetDetectPara(std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd) {
    world_rack_infos_.clear();
    true_rack_pose_[0] = cmd->pose.x();
    true_rack_pose_[1] = cmd->pose.y();
    true_rack_pose_[2] = cmd->pose.yaw();
    detect_count_ = 0;
}

void RackQueryModule::resetQueryPara(std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd) {}

RackQueryModule::RackInfoWithPose RackQueryModule::computeMeanRack(
    std::vector<RackQueryModule::RackInfoWithPose> infos) {
    RackInfoWithPose rack_info;
    if (infos.empty()) {
        LOG(INFO) << "cannot compute rack info! will return!";
        return rack_info;
    }
    std::vector<double> xs, ys;
    std::vector<std::pair<double, double>> cos_yaws;
    for (const auto &info : infos) {
        xs.push_back(info.pose[0]);
        ys.push_back(info.pose[1]);
        cos_yaws.emplace_back(std::cos(info.pose[2]), info.pose[2]);
    }
    std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
    std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
    std::nth_element(cos_yaws.begin(), cos_yaws.begin() + cos_yaws.size() / 2, cos_yaws.end());
    rack_info.pose[0] = xs[xs.size() / 2];
    rack_info.pose[1] = ys[xs.size() / 2];
    rack_info.pose[2] = cos_yaws[cos_yaws.size() / 2].second;
    rack_info.rack_info = infos[0].rack_info;
    double min_delta_dist = (infos[0].pose - rack_info.pose).head<2>().norm();
    for (auto &info : infos) {
        double delta_dist = (info.pose - rack_info.pose).head<2>().norm();
        if (min_delta_dist > delta_dist) {
            min_delta_dist = delta_dist;
            rack_info.rack_info = info.rack_info;
        }
    }
    return rack_info;
}

bool getMeanCodeMsg(std::vector<sros::core::base_msg_ptr> &dm_code_msgs,Eigen::Vector3d& center_pose, Eigen::Vector3d &pose_in_code) {
    bool get_code_msg = false;
    double code_weight = 0.0f;
    pose_in_code.setZero();
    for (int i = 0; i < dm_code_msgs.size(); ++i) {
        if (dm_code_msgs[i]) {
            auto code = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(dm_code_msgs[i]);
            if (code->state_ == DM_CODE_DETECTED) {
                get_code_msg = true;
                pose_in_code[0] = (code->x_ + pose_in_code[0] * code_weight) / (1.0 + code_weight);
                pose_in_code[1] = (code->y_ + pose_in_code[1] * code_weight) / (1.0 + code_weight);
                pose_in_code[2] = (code->angle_ + pose_in_code[2] * code_weight) / (1.0 + code_weight);
                code_weight += 1.0;
            }
        }
    }
    if (get_code_msg) {
        Eigen::Affine2d center_tf(Eigen::Translation2d(center_pose.head<2>()) * Eigen::Rotation2Dd(center_pose[2]));
        pose_in_code.head<2>() = center_tf.inverse() * pose_in_code.head<2>();
        pose_in_code[2] = pose_in_code[2] - center_pose[2];
        LOG(INFO) << "pose in code:" << pose_in_code[0] << "," << pose_in_code[1] << "," << pose_in_code[2];
    }
    return get_code_msg;
}

float findMaxRackLength(std::vector<RackInfo> &racks) {
    float length = 0.0f;
    for (auto &rack : racks) {
        auto real_size_length = rack.leg_groups.back().length + rack.leg_d;
        auto real_size_width = rack.leg_groups.back().width + rack.leg_d;
        length = length > real_size_length ? length : real_size_length;
        length = length > real_size_width ? length : real_size_width;
    }
    return length;
}

void RackQueryModule::onPostureCorrectCmd(sros::core::base_msg_ptr msg) {
    auto sendFailedState = [&](int fail_code) {
        auto result = std::make_shared<sros::core::PostureCorrectCommandMsg>("TOPIC_POSTURE_CORRECT_RESULT");
        result->topic_ = "TOPIC_POSTURE_CORRECT_RESULT";
        result->result.correct_result = sros::core::PostureCorrectCommandMsg::CORRECT_RESULT_FAIL;
        result->result.error_code = fail_code;
        LOG(INFO) << "fail!";
        sendMsg(result);
        g_state.station_no = 0;
        cur_station_no = 0;//矫正完成，清id
    };
    auto sendSuccessState = [&]() {
        auto result = std::make_shared<sros::core::PostureCorrectCommandMsg>("TOPIC_POSTURE_CORRECT_RESULT");
        result->topic_ = "TOPIC_POSTURE_CORRECT_RESULT";
        result->result.correct_result = sros::core::PostureCorrectCommandMsg::CORRECT_RESULT_SUCCESS;
        LOG(INFO) << "success!";
        sendMsg(result);
        g_state.station_no = 0;
        cur_station_no = 0;//矫正完成，清id
    };
    auto sendPathState = [&](const Eigen::Vector3d mid_pose, const Eigen::Vector3d dst_pose) {
        auto result = std::make_shared<sros::core::PostureCorrectCommandMsg>("TOPIC_POSTURE_CORRECT_RESULT");
        result->topic_ = "TOPIC_POSTURE_CORRECT_RESULT";
        result->result.correct_result = sros::core::PostureCorrectCommandMsg::CORRECT_RESULT_REPLAN_PATHS;
        result->result.pose.mid_pose.x() = mid_pose[0];
        result->result.pose.mid_pose.y() = mid_pose[1];
        result->result.pose.mid_pose.yaw() = mid_pose[2];
        result->result.pose.dst_pose.x() = dst_pose[0];
        result->result.pose.dst_pose.y() = dst_pose[1];
        result->result.pose.dst_pose.yaw() = dst_pose[2];
        LOG(INFO) << "mid_pose:" << mid_pose[0] << "," << mid_pose[1] << "," << mid_pose[2];
        LOG(INFO) << "dst_pose:" << dst_pose[0] << "," << dst_pose[1] << "," << dst_pose[2];
        sendMsg(result);
    };
    auto sendCorrectPoseState = [&](const Eigen::Vector3d offset) {
        auto result = std::make_shared<sros::core::PostureCorrectCommandMsg>("TOPIC_POSTURE_CORRECT_RESULT");
        result->topic_ = "TOPIC_POSTURE_CORRECT_RESULT";
        result->result.correct_result = sros::core::PostureCorrectCommandMsg::CORRECT_RESULT_POSE_CORRECT;
        result->result.offset.offset_x = offset[0]*10000;
        result->result.offset.offset_y = offset[1]*10000;
        result->result.offset.offset_angle = offset[2] / M_PI * 1800;

        LOG(INFO) << "offset:" << offset[0] << "," << offset[1] << "," << offset[2];
        LOG(INFO) << "offset:" << result->result.offset.offset_x << "," << result->result.offset.offset_y << ","
                  << result->result.offset.offset_angle;
        sendMsg(result);
    };

    auto pc_cmd_msg = std::dynamic_pointer_cast<sros::core::PostureCorrectCommandMsg>(msg);
    if (pc_cmd_msg->command.first_send_cmd) {//只有第一次的时候，才配置
        curr_zero_center_pose_.setZero();
        max_height_err = sros::core::Settings::getInstance().getValue<double>("rack.cmd_136_up_resolution", 8) * sros::core::MM_TO_M;

        if(pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_FORWARD||
           pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_BACK){
            if(!updateDownCameraOffsetInfo(curr_zero_center_pose_)){
                LOG(INFO) << "will use zero center pose!";
                curr_zero_center_pose_.setZero();
            }
            max_height_err = sros::core::Settings::getInstance().getValue<double>("rack.cmd_136_down_resolution", 5) * sros::core::MM_TO_M;
        }else if(pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_CHARGE_FORWARD||
                 pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_CHARGE_BACK){
            if(!updateDownCameraOffsetInfo(curr_zero_center_pose_)){
                LOG(INFO) << "will use zero center pose!";
                curr_zero_center_pose_.setZero();
            }
            max_height_err = sros::core::Settings::getInstance().getValue<double>("rack.cmd_136_down_resolution", 5) * sros::core::MM_TO_M;
        }
    }
    g_state.station_no = cur_station_no;//重新赋值给站点id号，避免matrix上站点偏移数据被清空
    auto station = MapManager::getInstance()->getStation(cur_station_no);
    DMCodeOffset offset(station.dmcode_id, station.dmcode_offset.x / 100, station.dmcode_offset.y / 100,
                station.dmcode_offset.yaw);
    g_state.station_camera_offset.set(offset);//把站点偏移数据发给matrix
    LOG(INFO) << "cur_station_no:" << cur_station_no;

    if (pc_cmd_msg->command.correct_cmd == sros::core::PostureCorrectCommandMsg::CORRECT_CMD_START) {
        Eigen::Vector3d center_in_code;
        if (getMeanCodeMsg(dm_code_msgs_,curr_zero_center_pose_,center_in_code)) {  //获取当前二维码值，这里返回的是相机中心在二维码坐标系位置偏差
            auto angle = atan2(sin(center_in_code[2]), cos(center_in_code[2]));  //归一化到-180到180之间
            auto normal_angle = round(angle / M_PI_2) * M_PI_2;  //将其设置成90°的倍数，用以更好的确定货架实际朝向
            LOG(INFO) << "normal angle:" << normal_angle;
            Eigen::Vector2d normal_vector(cos(normal_angle), sin(normal_angle));  //当前货架实际朝向
            auto height = normal_vector[0] * center_in_code[1] -
                          normal_vector[1] * center_in_code[0];  //基于朝向，计算小车与货架中心的横向偏差
            auto delta_angle = normal_angle - center_in_code[2];  //计算小车朝向和车体朝向偏差
            delta_angle = atan2(sin(delta_angle), cos(delta_angle));
            auto delta_x = normal_vector[0] * center_in_code[0] +
                           normal_vector[1] * center_in_code[1];  //基于朝向，计算小车与货架中心的纵向偏差
            LOG(INFO) << "delta info:" << delta_x << "," << height << "," << delta_angle;
            if(fabs(delta_angle) > 0.174533f){
                LOG(INFO) << "angle:" << delta_angle << ", angle > 10!";
                sendFailedState(13606);
            }else{
                if (fabs(height) < max_height_err) {

                    //局部微调,考虑到差速轮横向偏差调整不了，所以，不调节横向偏差。若横向偏差在一定范围内，则进行局部微调
                    if (have_small_correct&&fabs(delta_x) < 0.002f &&
                        fabs(delta_angle) < 0.0087f) {  //如果误差在该范围内，则不继续调整，直接返回正确
                        have_small_correct = false;
                        if(pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_CHARGE_FORWARD||
                        pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_CHARGE_BACK){
                            float offset_x = 0.1;
                            if(pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_CHARGE_FORWARD){
                                offset_x -= delta_x;
                            }else{
                                offset_x = -offset_x - delta_x;
                            }
                            sendCorrectPoseState(
                                Eigen::Vector3d(offset_x, 0, delta_angle));  //如果误差在该范围内，则发送微调信息
                            usleep(5e5);
                        }
                        sendSuccessState();
                    } else {
                        have_small_correct = true;
                        sendCorrectPoseState(
                            Eigen::Vector3d(-delta_x, 0, delta_angle));  //如果误差在该范围内，则发送微调信息
                    }
                } else {  //全局路径规划
                    LOG(INFO) << "will global plan!";
                    Eigen::Affine2d center_in_code_tf(Eigen::Translation2d(center_in_code[0], center_in_code[1]) *
                                                    Eigen::Rotation2Dd(center_in_code[2]));
                    auto code_center_point =
                        center_in_code_tf.inverse() * Eigen::Vector2d(0, 0);  //将坐标求逆，计算code在当前车体中心位置信息

                    auto cur_pos = src_sdk->getCurPose();                     //获取当前小车中心位姿
                    Eigen::Affine2d world_tf(Eigen::Translation2d(cur_pos.x(), cur_pos.y()) *
                                            Eigen::Rotation2Dd(cur_pos.yaw()));  //转换成矩阵的形式
                    Eigen::Vector3d code_in_world;
                    code_in_world.head<2>() = world_tf * code_center_point;  //计算世界坐标系下，需要到达的目标位置
                    float err_offset = 0.005f;//添加5mm调节偏差
                    if (height > 0) {
                        err_offset = -0.005f;
                    }
                    auto max_length =
                        findMaxRackLength(rack_infos_) / 2.0f;  //获取最长的货架长度，目的是小车后退调整，具有调整空间
                    max_length = max_length > 0.5 ? max_length : 0.5;
                    if(pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_FORWARD||
                    pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_BACK){
                        max_length = 0.3;
                        err_offset *= 0.8;
                    }
                    code_in_world[2] = cur_pos.yaw() + delta_angle;  //计算世界坐标系下，需要到达的目标姿态
                    Eigen::Affine2d code_in_world_tf(Eigen::Translation2d(code_in_world.x(), code_in_world.y()) *
                                            Eigen::Rotation2Dd(code_in_world.z()));  //转换成矩阵的形式
                    code_in_world.head<2>() = code_in_world_tf * Eigen::Vector2d(0, err_offset);

                    Eigen::Vector2d delta_point = Eigen::Vector2d::Zero();
                    if (pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DIR_FORWARD ||
                        pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_FORWARD||
                        pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_CHARGE_FORWARD) {
                        //如果是前进的，需要进行后退调整
                        delta_point[0] = -max_length;
                    } else if (pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DIR_BACK ||
                            pc_cmd_msg->command.correct_dir == sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_BACK ||
                            pc_cmd_msg->command.correct_dir ==
                                sros::core::PostureCorrectCommandMsg::CORRECT_DOWN_CHARGE_BACK) {
                        //如果是后退的，需要进行前进调整
                        delta_point[0] = max_length;
                    } else {
                        LOG(INFO) << "wrong direction!";
                        //sendFailedState(13607);
                        return;
                    }
                    Eigen::Affine2d dst_tf(Eigen::Translation2d(code_in_world.x(), code_in_world.y()) *
                                        Eigen::Rotation2Dd(code_in_world[2]));
                    Eigen::Vector3d mid_pose;
                    mid_pose.head<2>() = dst_tf * delta_point;  //计算调整中间点的位置
                    mid_pose[2] = code_in_world[2];
                    sendPathState(mid_pose, code_in_world);  //生成调整路径
                    //                sendPathState()
                }
            //            auto delta
            //            auto curr_pose
            }
        } else {
            LOG(INFO) << "cannot get right code!";
            sendFailedState(13601);
        }
    } else {
        LOG(INFO) << "cannot get right command!" << pc_cmd_msg->command.correct_cmd;
        sendFailedState(13605);
    }
}

void RackQueryModule::onDmCodeMsg(sros::core::base_msg_ptr msg) {
    auto sendFailedState = [&](int fail_code) {
        auto result = std::make_shared<sros::core::PostureCorrectCommandMsg>("TOPIC_POSTURE_CORRECT_RESULT");
        result->topic_ = "TOPIC_POSTURE_CORRECT_RESULT";
        result->result.correct_result = sros::core::PostureCorrectCommandMsg::CORRECT_RESULT_FAIL;
        result->result.error_code = fail_code;
        LOG(INFO) << "fail!";
        sendMsg(result);
    };
    if (in_posture_correct_state) {
        const int max_dm_code_count = 2;
        auto code = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(msg);

        if (sensor_name_ == sros::device::DEVICE_SVC100_DOWN) {
            std::shared_ptr<sros::core::DataMatrixCodeMsg> right_code(new sros::core::DataMatrixCodeMsg);
            convertDmToStandardHandSystem(code, right_code);
            *code = *right_code;
        }
        LOG(INFO) << "name:" << code->camera_name_ << "," << sensor_name_ << ", state:" << code->state_ << ", id:" \
        << code->code_str_ << ", x:" << code->x_ << ", y:" << code->y_ << ", yaw:" << code->angle_;

        if (cur_station_no != 0) {
            auto station = sros::core::MapManager::getInstance()->getStation(cur_station_no);
            if (!station.dmcode_id.empty() && code->state_ == 1) {
                if(station.dmcode_id != code->code_str_)
                {
                    LOG(INFO) << "id diff, station_id:" << station.dmcode_id << ", curr_id:" << code->code_int_;
                    sendFailedState(13604);//id no diff
                    return ;
                }
            }
        }

        if (code->camera_name_ == sensor_name_) {
            if (code->state_ == DM_CODE_DETECTED) {
                dm_code_msgs_.push_back(msg);
            }
            all_dm_code_msgs_.push_back(msg);
            if ((dm_code_msgs_.size() >= 1&&all_dm_code_msgs_.size()>=2) || all_dm_code_msgs_.size() >= 20) {//如果连续20帧都得不到数据，就说明扫码失败
                if(dm_code_msgs_.empty()){
                    un_detect_id_++;
                    LOG(INFO) << "un_detect_id:" << un_detect_id_;
                    std::shared_ptr<sros::core::CommonCommandMsg<std::string>> cmd_msg(new sros::core::CommonCommandMsg<std::string>("SAVE_IMG"));
                    cmd_msg->seq = un_detect_id_;
                    cmd_msg->command = "SAVE";
                    cmd_msg->str0 = sros::device::DEVICE_SVC100_UP;
                    sendMsg(cmd_msg);
                }
                in_posture_correct_state = false;
                onPostureCorrectCmd(last_posture_corr_cmd_);
                dm_code_msgs_.clear();
                all_dm_code_msgs_.clear();
            }
        }
    }
}
void RackQueryModule::onTimer_100ms(sros::core::base_msg_ptr msg) {
    auto sendFailedState = [&](int fail_code) {
        auto result = std::make_shared<sros::core::PostureCorrectCommandMsg>("TOPIC_POSTURE_CORRECT_RESULT");
        result->topic_ = "TOPIC_POSTURE_CORRECT_RESULT";
        result->result.correct_result = sros::core::PostureCorrectCommandMsg::CORRECT_RESULT_FAIL;
        result->result.error_code = fail_code;
        sendMsg(result);
    };

    if (in_posture_correct_state) {
        watch_dog_time++;
        if (watch_dog_time > watch_dog_timeout_1s) {
            LOG(INFO) << "watch time out!";
            sendFailedState(13601);//no dm_code data
            watch_dog_time = 0;
            in_posture_correct_state = false;
        }
    }
}
void RackQueryModule::onPostureReceive(sros::core::base_msg_ptr msg) {
    auto pc_cmd_msg = std::dynamic_pointer_cast<sros::core::PostureCorrectCommandMsg>(msg);
    if (pc_cmd_msg->command.correct_cmd == sros::core::PostureCorrectCommandMsg::CORRECT_CMD_START) {
        LOG(INFO) << "will correct!";
        watch_dog_time = 0;
        in_posture_correct_state = true;
        sensor_name_ = pc_cmd_msg->command.sensor_name;
        last_posture_corr_cmd_ = msg;
    }
}
bool RackQueryModule::updateDownCameraOffsetInfo(Eigen::Vector3d &center_offset) {
    cur_station_no = g_state.station_no;
    if (cur_station_no != 0) {
        auto station = sros::core::MapManager::getInstance()->getStation(cur_station_no);
        if (!station.dmcode_id.empty()) {
            center_offset[0] = station.dmcode_offset.x / 100;
            center_offset[1] = station.dmcode_offset.y / 100;
            center_offset[2] =  station.dmcode_offset.yaw;
            if (sensor_name_ == sros::device::DEVICE_SVC100_DOWN) {
                auto right_hand = sros::core::Settings::getInstance().getValue<std::string>(
                                        "vision.down_dm_code_right_handed_system", "True") == "True";
                std::shared_ptr<sros::core::DataMatrixCodeMsg> origin_code(new sros::core::DataMatrixCodeMsg);
                std::shared_ptr<sros::core::DataMatrixCodeMsg> right_origin_code(new sros::core::DataMatrixCodeMsg);
                origin_code->x_ = center_offset[0];
                origin_code->y_ = center_offset[1];
                origin_code->angle_ = center_offset[2];
                origin_code->right_handed_system = right_hand;
                convertDmToStandardHandSystem(origin_code, right_origin_code);
                center_offset[0] = right_origin_code->x_;
                center_offset[1] = right_origin_code->y_;
                center_offset[2] = right_origin_code->angle_;
                LOG(INFO) << "center:" << center_offset[0] << "," << center_offset[1] << "," << center_offset[2];
            }
            return true;
        }
    }
    return false;
}
}