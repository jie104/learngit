//
// Created by lfc on 17-9-5.
//

#include <core/msg/PoseStampedMsg.h>
#include <core/msg/parameter_msg.hpp>
#include <core/settings.h>

#include "slam_map_processor.h"
namespace mapping{
bool SlamMapProcessor::handleStartMapping(sros::core::SlamCommandMsg &syscommand) {
    map_name = syscommand.map_name;
    map_path = syscommand.map_path;
    slam::MappingCmd_Ptr cmd(new slam::MappingCmdSlamMsg);
    cmd->map_name = map_name;
    cmd->map_path = map_path;
    cmd->cmd = slam::START_MAPPING_SLAMCMD;
    if (mapping_processor->handleMapCmd(cmd)) {
        LOG(INFO) << "successfully to start map!";
    }
    return true;
}

bool SlamMapProcessor::handleStopMapping(sros::core::SlamCommandMsg &syscommand) {
    map_name = syscommand.map_name;
    map_path = syscommand.map_path;
    slam::MappingCmd_Ptr cmd(new slam::MappingCmdSlamMsg);
    cmd->map_name = map_name;
    cmd->map_path = map_path;
    cmd->cmd = slam::STOP_MAPPING_SLAMCMD;
    LOG(INFO) << "stop map!";
    if (mapping_processor->handleMapCmd(cmd)) {
        LOG(INFO) << "successfully to stop map!";
    }
    return true;
}

bool SlamMapProcessor::handleCancelMapping(sros::core::SlamCommandMsg &syscommand) {
    map_name = syscommand.map_name;
    map_path = syscommand.map_path;
    slam::MappingCmd_Ptr cmd(new slam::MappingCmdSlamMsg);
    cmd->map_name = map_name;
    cmd->map_path = map_path;
    cmd->cmd = slam::CANCEL_MAPPING_SLAMCMD;
    if (mapping_processor->handleMapCmd(cmd)) {

    }
    return true;
}

bool SlamMapProcessor::processScan(sros::core::base_msg_ptr base_ptr) {
    if (getMappingState()) {
        sros::core::LaserScan_ptr scan_msg = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);
        slam::ScanSlam_Ptr scan(new slam::ScanSlamMsg);
        scan->angle_increment = scan_msg->angle_increment;
        scan->angle_min = scan_msg->angle_min;
        scan->angle_max = scan_msg->angle_max;
        scan->range_min = scan_msg->range_min;
        scan->range_max = scan_msg->range_max;
        scan->stamp = scan_msg->time_;
        scan->time_increment = scan_msg->time_increment;
        scan->ranges = scan_msg->undistorted_ranges;
        scan->intensities = scan_msg->undistorted_intensities;
        mapping_processor->processMsg(scan);
    }
    return true;
}

bool SlamMapProcessor::processPara(sros::core::base_msg_ptr base_ptr) {
    auto msg = std::dynamic_pointer_cast<sros::core::ParameterMsg>(base_ptr);
    std::string name = msg->name;
    std::string value = msg->value;
//
// LOG(INFO) << "StandardMappingRos::onParameterMsg -> " << name << ": " << value;

    // TODO 修改参数
    if (name == "slam.map_laser_min_dist") {
       mapping_info->laser_min_dist = atof(value.c_str());
    } else if (name == "slam.map_laser_max_dist") {
        mapping_info->laser_max_dist = atof(value.c_str());
    } else if (name == "slam.tilt_angle_thresh") {
        mapping_info->tilt_angle_thresh = atof(value.c_str());
    } else if (name == "slam.laser_angle_min") {
        mapping_info->laser_angle_min = atof(value.c_str());
    } else if (name == "slam.laser_angle_max") {
        mapping_info->laser_angle_max = atof(value.c_str());
    } else if (name == "slam.pose_percentage_thresh") {
    } else if (name == "slam.laser_z_min") {
        mapping_info->laser_z_min = atof(value.c_str());
    } else if (name == "slam.laser_z_max") {
        mapping_info->laser_z_max = atof(value.c_str());
    } else if (name == "slam.map_resolution") {
        mapping_info->map_resolution = atof(value.c_str());
    } else if (name == "slam.map_multi_level") {
        mapping_info->map_multi_level = atoi(value.c_str());
    } else if (name == "slam.laser_publish_freq") {
    }  else if (name == "slam.stamp_match_thresh") {
        mapping_info->stamp_match_thresh = static_cast<int64_t>(atoi(value.c_str()));
    } else if (name == "slam.ini_angle_count") {
    } else if (name == "slam.ini_distri_size") {
    } else if (name == "slam.is_allow_cout") {
    } else if (name == "slam.usekslam") {
        mapping_info->use_kslam = (bool) atoi(value.c_str());
    } else if (name == "slam.userecordedbag") {
//        mapping_info->userecordedbag = (bool) atoi(value.c_str());
    } else if (name == "slam.uselmkslam") {
        mapping_info->use_lmkslam = (bool) atoi(value.c_str());
    } else if (name == "slam.loop_search_maximum_distance") {
//        if (slam_karto)
//            slam_karto->loop_search_maximum_distance = atof(value.c_str());
    } else if (name == "slam.length_double_lmk_thresh") {
        mapping_info->length_double_lmk_thresh = atof(value.c_str());
    } else if (name == "slam.lmk_intensity_thresh") {
        mapping_info->lmk_intensity_thresh = atof(value.c_str());
    } else if (name == "slam.length_single_lmk_thresh") {
//        LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!length:" <<
//        lmk::intensitylmk.lengthSingleLmkThres;
    } else if (name == "slam.use_pitchroll_correct") {
        mapping_info->use_pitchroll_correct = (bool) atoi(value.c_str());
    } else if (name == "slam.consistent_min_error") {
//        constraint_consistent->min_error = atof(value.c_str());
    } else if (name == "slam.consistent_count_thresh") {
//        constraint_consistent->pose_error_count_thresh = atoi(value.c_str());
    } else if (name == "slam.use_record_pose_info") {
//        use_record_pose_info = atoi(value.c_str());
    }else if (name == "slam.use_h_optimize") {
//        pose_map_para->use_H_optimize = (bool) atoi(value.c_str());
    }else if (name == "slam.enable_large_map_mode") {
//        enable_large_map_mode_ = (value == "True");
    } else if (name == "slam.pre_map_size_x") {
//        pre_map_size_x_ = atoi(value.c_str());
    } else if (name == "slam.pre_map_size_y") {
//        pre_map_size_y_ = atoi(value.c_str());
    } else if (name == "slam.pre_map_zero_offset_x") {
//        pre_map_zero_offset_x_ = atoi(value.c_str());
    } else if (name == "slam.pre_map_zero_offset_y") {
//        pre_map_zero_offset_y_ = atoi(value.c_str());
    }else if (name == "slam.lmk_extract_type") {
        mapping_info->lmk_type = value;
    }else if(name == "slam.need_endstart_loop"){
        mapping_info->need_endstart_loop = (bool) atoi(value.c_str());
    }
    return true;
}

void SlamMapProcessor::saveMap(std::string map_name) {
//    MappingProcessor::saveMap(map_name);
}

SlamMapProcessor::SlamMapProcessor():MappingProcessor(SLAM_MAP_TYPE) {
    mapping_info.reset(new slam::MappingInfoSlamMsg);
    updatePara(mapping_info);

    mapping_processor = slam::NormalMappingSingleton::getInstance(mapping_info);
    mapping_processor->setMapOutputCallback(boost::bind(&SlamMapProcessor::mapOutputCallback, this, _1));
}

SlamMapProcessor::~SlamMapProcessor() {

}

void SlamMapProcessor::mapOutputCallback(slam::MapOutputSlam_Ptr map_output) {
    using namespace slam;
    switch (map_output->state) {
        case IDLE_STATE: {
            changeStateMsg(sros::core::STATE_SLAM_IDLE);
            resetMappingState();
            break;
        }
        case START_MAP_SLAMSTATE: {
            changeStateMsg(sros::core::STATE_SLAM_DRAWING);
            usleep(1e5);
            setMappingState();
            break;
        }
        case SAVING_MAP_SLAMSTATE: {
            double delta_time = map_output->stop_time - map_output->start_time;
            double exe_time = map_output->stamp - map_output->start_time;

            sros::core::Progress_t progress = 0;
            if (delta_time > 0 && exe_time > 0) {
                progress = floor(exe_time / delta_time * 100.0);
            }
            if (map_output->pose) {
                sendPose(map_output->pose);
            }
            changeStateMsg(sros::core::STATE_SLAM_SAVING_MAP, progress);
            resetMappingState();
            break;
        }
        case NORMAL_MAPPING_SLAMSTATE: {
            LOG(INFO) << "slam drawing!";
            changeStateMsg(sros::core::STATE_SLAM_DRAWING);
            setMappingState();
            break;
        }
        case WARN_MAPPING_SLAMSTATE: {
            LOG(INFO) << "warnning!";
            break;
        }
        case ERR_MAPPING_SLAMSTATE: {
            LOG(INFO) << "err!";
            processStopMapping();
            break;
        }
        case FALT_MAPPING_SLAMSTATE: {
            LOG(INFO) << "falt!";
            processStopMapping();
            break;
        }
        case NOT_GET_LMK_SLAMSTATE: {
            LOG(INFO) << "cannot get the lmk!";
            changeStateMsg(sros::core::STATE_SLAM_ERROR);
//            changeStateMsg(sros::core::STATE_SLAM_DRAWING);
        }

    }

}

bool SlamMapProcessor::processPose(sros::core::base_msg_ptr base_ptr) {
    if (getMappingState()) {
        sros::core::PoseStamped_ptr pose = std::dynamic_pointer_cast<sros::core::PoseStampedMsg>(base_ptr);
        slam::PoseSlam_Ptr pose_msg(new slam::PoseSlamMsg);
        pose_msg->x = pose->pose.x();
        pose_msg->y = pose->pose.y();
        pose_msg->z = pose->pose.z();
        pose_msg->yaw = pose->pose.yaw();
        pose_msg->roll = pose->pose.roll();
        pose_msg->pitch = pose->pose.pitch();
        pose_msg->stamp = pose->time_;
        mapping_processor->processMsg(pose_msg);
    }
    return true;
}

void SlamMapProcessor::sendPose(slam::PoseSlam_Ptr pose_msg) {
    sros::core::PoseStamped_ptr pose(new sros::core::PoseStampedMsg("TOPIC_MATCHPOSE"));
    pose->pose.x() = pose_msg->x;
    pose->pose.y() = pose_msg->y;
    pose->pose.yaw() = pose_msg->yaw;
    pose->pose.z() = pose_msg->z;
    pose->pose.roll() = pose_msg->roll;
    pose->pose.pitch() = pose_msg->pitch;
    pose->time_ = sros::core::util::get_time_in_us();
    if (sendMsg) {
        sendMsg(pose);
    }
}

void SlamMapProcessor::processStopMapping() {
    changeStateMsg(sros::core::STATE_SLAM_ERROR);
    usleep(1e5);
    changeStateMsg(sros::core::STATE_SLAM_IDLE);
    resetMappingState();

}
void SlamMapProcessor::updatePara(std::shared_ptr<slam::MappingInfoSlamMsg> mapping_info) {
    auto &s = sros::core::Settings::getInstance();
    mapping_info->laser_min_dist = s.getValue("slam.map_laser_min_dist", 0.1);
    mapping_info->laser_max_dist = s.getValue("slam.map_laser_max_dist", 30.0);
    mapping_info->tilt_angle_thresh = s.getValue("slam.tilt_angle_thresh", 0.5);
    mapping_info->laser_angle_min = s.getValue("slam.laser_angle_min", -2.37);
    mapping_info->laser_angle_max = s.getValue("slam.laser_angle_max", 2.37);
    mapping_info->use_kslam = s.getValue("slam.use_kslam", 1);
    mapping_info->laser_z_min = s.getValue("slam.laser_z_min", 0.1);
    mapping_info->laser_z_max = s.getValue("slam.laser_z_max", 0.5);
    mapping_info->map_resolution = s.getValue("slam.map_resolution", 0.02);
    mapping_info->map_multi_level = s.getValue("slam.map_multi_level", 3);
    mapping_info->stamp_match_thresh = s.getValue("slam.stamp_match_thresh", 60000);
    mapping_info->use_lmkslam = s.getValue("slam.use_lmkslam", 1);
    mapping_info->length_double_lmk_thresh = s.getValue("slam.length_double_lmk_thresh", 0.34);
    mapping_info->lmk_intensity_thresh = s.getValue<int>("slam.lmk_intensity_thresh", 500);
    mapping_info->use_pitchroll_correct = s.getValue("slam.use_pitchroll_correct", 1);
    mapping_info->need_endstart_loop = s.getValue("slam.need_endstart_loop", 1);
    mapping_info->lmk_type = s.getValue<std::string>("slam.lmk_extract_type", "CYLINDER");

    mapping_info->laser_coordx = s.getValue("posefilter.laser_coordx", 0.29);
    mapping_info->laser_coordy = s.getValue("posefilter.laser_coordy", 0.0);
    mapping_info->laser_coordyaw = s.getValue("posefilter.laser_coordyaw", 0.0);

    LOG(INFO)<<"para info:"<<mapping_info->laser_min_dist<<","<<
                          mapping_info->laser_max_dist<<","<<
                          mapping_info->tilt_angle_thresh<<","<<
                          mapping_info->laser_angle_min<<","<<
                          mapping_info->laser_angle_max<<","<<
                          mapping_info->use_kslam<<","<<
                          mapping_info->laser_z_min<<","<<
                          mapping_info->laser_z_max<<","<<
                          mapping_info->map_resolution<<","<<
                          mapping_info->map_multi_level<<","<<
                          mapping_info->stamp_match_thresh<<",\n"<<
                          mapping_info->use_lmkslam<<","<<
                          mapping_info->length_double_lmk_thresh<<","<<
                          mapping_info->lmk_intensity_thresh<<","<<
                          mapping_info->use_pitchroll_correct<<","<<
                          mapping_info->need_endstart_loop<<","<<
                          mapping_info->lmk_type<<","<<
                          mapping_info->laser_coordx<<","<<
                          mapping_info->laser_coordy<<","<<
                          mapping_info->laser_coordyaw<<",";
}
}
