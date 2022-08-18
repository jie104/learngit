//
// Created by lfc on 17-9-11.
//

#include "slam_loc_processor.h"
#include <core/msg/PoseStampedMsg.h>
#include <core/settings.h>
#include <core/msg/ObstacleMsg.hpp>
#include <core/msg/common_command_msg.hpp>
#include <core/msg/laser_scan_msg.hpp>
#include <core/msg/parameter_msg.hpp>
#include <core/scan_laser_filter_alg.hpp>
#include "../../slam/include/msg/location_code_slammsg.hpp"
#include "../../slam/include/msg/location_input_slammsg.hpp"
#include "core/msg/data_matrix_code_msg.hpp"
#include "core/msg/lmk_match_info_msg.hpp"
#include <core/util/record_file_manager.hpp>

namespace location{

SlamLocProcessor::SlamLocProcessor():LocationProcessor(SLAM_LOC_TYPE) {
    reset();
}

bool SlamLocProcessor::handleStartLocation(sros::core::SlamCommandMsg &syscommand) {
    boost::thread(boost::bind(&SlamLocProcessor::initialLocThread, this, syscommand));
    return true;
}

bool SlamLocProcessor::handleStopLocation(sros::core::SlamCommandMsg &syscommand) {
    slam::LocationCmd_Ptr cmd(new slam::LocationCmdSlamMsg);
    cmd->cmd = slam::STOP_LOCATION_SLAMCMD;
    LOG(INFO) << "stop loc!";
    cmd->map_name = syscommand.map_name;
    cmd->map_path = syscommand.map_path;
    loc_processor->handleLocCmd(cmd);
    return true;
}

std::vector<std::string> SlamLocProcessor::splitStr(const std::string &str, const char delim)
{
    std::vector<std::string> splited;
    std::stringstream sstr(str);
    std::string splited_str;
    while (getline(sstr, splited_str, delim))
    {
        splited.emplace_back(splited_str);
    }
    return splited;
}

bool SlamLocProcessor::processPara(sros::core::base_msg_ptr base_ptr) {
    return true;
    auto msg = std::dynamic_pointer_cast<sros::core::ParameterMsg>(base_ptr);
    std::string name = msg->name;
    std::string value = msg->value;
//
// LOG(INFO) << "StandardMappingRos::onParameterMsg -> " << name << ": " << value;

    // TODO 修改参数
    if (name == "slam.map_laser_min_dist") {
        loc_info->laser_min_dist = atof(value.c_str());
    } else if (name == "slam.map_laser_max_dist") {
        loc_info->laser_max_dist = atof(value.c_str());
    } else if (name == "slam.tilt_angle_thresh") {
        loc_info->tilt_angle_thresh = atof(value.c_str());
    } else if (name == "slam.laser_angle_min") {
        loc_info->laser_angle_min = atof(value.c_str());
    } else if (name == "slam.laser_angle_max") {
        loc_info->laser_angle_max = atof(value.c_str());
    } else if (name == "slam.pose_percentage_thresh") {
        loc_info->pose_percentage_thresh = atof(value.c_str());
    } else if (name == "slam.laser_z_min") {
        loc_info->laser_z_min = atof(value.c_str());
    } else if (name == "slam.laser_z_max") {
        loc_info->laser_z_max = atof(value.c_str());
    } else if (name == "slam.map_resolution") {
        loc_info->map_resolution = atof(value.c_str());
    } else if (name == "slam.map_multi_level") {
        loc_info->map_multi_level = atoi(value.c_str());
    } else if (name == "slam.laser_publish_freq") {
    }  else if (name == "slam.stamp_match_thresh") {
        loc_info->stamp_match_thresh = static_cast<int64_t>(atoi(value.c_str()));
    } else if (name == "slam.ini_angle_count") {
        loc_info->ini_angle_count = atoi(value.c_str());
    } else if (name == "slam.ini_distri_size") {
        loc_info->ini_distri_size = atof(value.c_str());
    } else if (name == "slam.is_allow_cout") {
    } else if (name == "slam.usekslam") {
    } else if (name == "slam.userecordedbag") {
//        mapping_info->userecordedbag = (bool) atoi(value.c_str());
    } else if (name == "slam.uselmkslam") {
//        mapping_info->use_lmkslam = (bool) atoi(value.c_str());
    } else if (name == "slam.loop_search_maximum_distance") {

//        if (slam_karto)
//            slam_karto->loop_search_maximum_distance = atof(value.c_str());
    } else if (name == "slam.length_double_lmk_thresh") {
//        lmk::intensitylmk.lengthDoubleLmkThres = atof(value.c_str());
    } else if (name == "slam.lmk_intensity_thresh") {
//        lmk::intensitylmk.intensThres = atof(value.c_str());
//        mapping_info->lmk_intensity_thresh = atof(value.c_str());
    } else if (name == "slam.length_single_lmk_thresh") {
//        lmk::intensitylmk.lengthSingleLmkThres = atof(value.c_str());
    } else if (name == "slam.use_pitchroll_correct") {
        loc_info->use_pitch_roll_correct = (bool) atoi(value.c_str());
    } else if (name == "slam.consistent_min_err") {
        loc_info->consistent_min_err = atof(value.c_str());
//        constraint_consistent->min_error = atof(value.c_str());
    } else if (name == "slam.consistent_count_thresh") {
        loc_info->consistent_count_thresh = atoi(value.c_str());
//        constraint_consistent->pose_error_count_thresh = atoi(value.c_str());
    } else if(name == "slam.consistent_pose_size") {
        loc_info->consistent_pose_size = atoi(value.c_str());
    }else if (name == "slam.use_record_pose_info") {
//        use_record_pose_info = atoi(value.c_str());
    }else if (name == "slam.use_h_optimize") {
        loc_info->use_h_optimize = (bool) atoi(value.c_str());
//        pose_map_para->use_H_optimize = (bool) atoi(value.c_str());
    } else if(name == "slam.use_pose_record") {
        loc_info->use_pose_record = (bool) atoi(value.c_str());
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
    } else if(name == "slam.use_rt_slam_method") {
        loc_info->use_rt_slam_method = (bool) atoi(value.c_str());
    } else if(name == "slam.rt_slam_angle_min") {
        loc_info->rt_slam_range_min = atof(value.c_str());
    } else if (name == "posefilter.laser_coordx") {
         loc_info->laser_coordx = atof(value.c_str());
    } else if (name == "posefilter.laser_coordy") {
        loc_info->laser_coordy = atof(value.c_str());
    } else if (name == "posefilter.laser_coordyaw") {
        loc_info->laser_coordyaw = atof(value.c_str());
    } else if(name == "posefilter.y_middle_err_thresh"){
        loc_info->y_middlet_err_thresh = atof(value.c_str());
        printf("y_middle_err_thresh:%f\n", loc_info->y_middlet_err_thresh);
    } else if(name == "posefilter.y_small_err_thresh"){
        loc_info->y_small_err_thresh = atof(value.c_str());
        printf("y_small_err_thresh:%f\n", loc_info->y_small_err_thresh);
    } else if(name == "posefilter.k_theta_small_err"){
        loc_info->k_theta_small_err= atof(value.c_str());
        printf("k_theta_small_err:%f\n", loc_info->k_theta_small_err);
    } else if(name == "posefilter.k_theta_big_err"){
        loc_info->k_theta_big_err= atof(value.c_str());
        printf("k_theta_big_err:%f\n", loc_info->k_theta_big_err);
    } else if(name == "posefilter.k_y_small_err"){
        loc_info->k_y_small_err= atof(value.c_str());
        printf("k_y_small_err:%f\n", loc_info->k_y_small_err);
    } else if(name == "posefilter.k_y_middle_err"){
        loc_info->k_y_middle_err= atof(value.c_str());
        printf("k_y_middle_err:%f\n", loc_info->k_y_middle_err);
    } else if(name == "posefilter.k_y_large_err"){
        loc_info->k_y_large_err= atof(value.c_str());
        printf("k_y_large_err:%f\n", loc_info->k_y_large_err);
    }else if(name == "posefilter.x_small_err_thresh") {
        loc_info->x_small_err_thresh = atof(value.c_str());
    }else if(name == "posefilter.x_middle_err_thresh") {
        loc_info->x_middle_err_thresh = atof(value.c_str());
    }else if(name == "posefilter.k_x_small_err") {
        loc_info->k_x_small_err = atof(value.c_str());
    }else if(name == "posefilter.k_x_middle_err") {
        loc_info->k_x_middle_err = atof(value.c_str());
    }else if(name == "posefilter.k_x_large_err") {
        loc_info->k_x_large_err = atof(value.c_str());
    }else if(name == "posefilter.theta_small_err_thresh"){
        loc_info->theta_small_err_thresh= atof(value.c_str());
        printf("theta_small_err_thresh:%f\n", loc_info->theta_small_err_thresh);
    }else if(name == "posefilter.debug_info_print_flag"){
        loc_info->debug_info_print_flag= (bool)atoi(value.c_str());
        printf("debug_info_print_flag:%d\n", loc_info->debug_info_print_flag);
    } else if(name == "posefilter.pose_dist_error_thresh"){
        loc_info->pose_dist_err_thresh= atof(value.c_str());
        printf("pose_dist_error_thresh:%f\n", loc_info->pose_dist_err_thresh);
    } else if(name == "posefilter.pose_angle_error_thresh"){
        loc_info->pose_angle_err_thresh= atof(value.c_str());
        printf("pose_angle_error_thresh:%f\n", loc_info->pose_angle_err_thresh);
    } else if(name == "posefilter.enable_pose_fusion_flag") {
        loc_info->enable_pose_fusion = (bool)atoi(value.c_str());
        printf("enable_pose_fusion_flag:%d\n", loc_info->enable_pose_fusion);
    } else if (name == "slam.lmk_extract_type") {
        loc_info->lmk_type = value;
    } else if(name == "slam.always_update_pose") {
        loc_info->always_update_pose = (bool) atoi(value.c_str());
    } else if(name == "slam.realtime_update_map"){
        loc_info->realtime_update_map = (bool) atoi(value.c_str());
        LOG(INFO) << "real time update map:" << loc_info->realtime_update_map;
    } else if(name == "slam.realtime_update_max_dist"){
        loc_info->realtime_update_map_max_dist = atof(value.c_str());
        LOG(INFO) << "real time update max dist:" << loc_info->realtime_update_map_max_dist;
    } else if(name == "slam.lidar_type"){
        int lidar_type = atoi(value.c_str());
        if (lidar_type == 0x03 || lidar_type == 0x07) {
            need_filter_noise_points = true;
        }else{
            need_filter_noise_points = false;
        }
        LOG(INFO) << "slam.lidar type:" << lidar_type;
    } else if(name == "slam.location_layer"){
        loc_info->location_layer = atoi(value.c_str());
    }
    return true;
}

/**
 * @brief 构造scan和pose的定位输入，进行定位处理
 * 
 * @param base_ptr 
 * @return true 
 * @return false 
 */
bool SlamLocProcessor::processScan(sros::core::base_msg_ptr base_ptr) {
    sros::core::LaserScan_ptr scan_msg = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);
    scan_backup = scan_msg;
    if (getLocationState()) {
        slam::tf::TransForm tf;
        auto scan = getSlamScan(scan_msg);
        //step1 从tf中获取scan的pose
        if (tf_base_to_odo->lookUpTransForm(scan_msg->time_, tf, loc_info->stamp_match_thresh)){
            slam::PoseSlam_Ptr pose(new slam::PoseSlamMsg);
            pose->x = tf.position.x();
            pose->y = tf.position.y();
            pose->z = tf.position.z();
            pose->yaw = tf.rotation.yaw();
            pose->roll = tf.rotation.roll();
            pose->pitch = tf.rotation.pitch();
            slam::LocationInput_Ptr input_msg(new slam::LocationInputSlamMsg);
            input_msg->scan = scan;
            input_msg->pose = pose;
            //step2 进行slam处理
            loc_processor->processMsg(input_msg);
            last_odo_ = pose;
        }else if(last_odo_){
            LOG(INFO) << "will use last odo to match!";
            slam::LocationInput_Ptr input_msg(new slam::LocationInputSlamMsg);
            input_msg->scan = scan;
            input_msg->pose = last_odo_;
            loc_processor->processMsg(input_msg);
        }
        //step3 过滤scan，传给避障模块
        auto oba_scan = getSlamScan(scan_msg);
        if (need_filter_noise_points) {
            laser::filterLowIntenPoints(oba_scan, 0.8);
        }
        laser::filterScanTraildPoint(oba_scan, 0.8);
        filtScans(oba_scan);
        sendObstacle(oba_scan);
    }
    return LocationProcessor::processScan(base_ptr);
}

bool SlamLocProcessor::loadMap(std::string map_name) {
    return LocationProcessor::loadMap(map_name);
}

void SlamLocProcessor::locOutputCallback(slam::LocationOutput_Ptr loc_output) {
    using namespace slam;
    if (!loc_output) {
        LOG(WARNING) << "err to get loc ptr!";
        return;
    }
    switch (loc_output->state) {
        case INITIAL_LOCATION_SLAMSTATE:
            if(getState()!=sros::core::STATE_SLAM_LOCATING_AMCL){
                LOG(INFO) << "initial location!";
                changeStateMsg(sros::core::STATE_SLAM_LOCATING_AMCL);
            }
            if (loc_output->pose) {
                LOG(INFO) << "will send initial loc pose!";
                sendMatchMsg(loc_output);
            }
            break;
        case INITIAL_SUCESS_SLAMSTATE:
            LOG(INFO) << "successfully to locate!";
            changeStateMsg(sros::core::STATE_SLAM_LOCATING);
            setLocationState();
            break;
        case NORMAL_LOCATION_SLAMSTATE:
        case LOCAL_SLAMSTATE:
        case LOCAL_REALTIME_SLAM_SLAMSTATE:
            if (getLocationState()) {
                if (loc_output->pose) {
                    sendMatchMsg(loc_output);
                }
            }else{
                LOG(INFO) << "slam state is wrong!";
            }
            break;
        case ERR_LOCATION_SLAMSTATE:
        case WARN_LOCATION_SLAMSTATE:
            LOG(INFO) << "err!";
            processStopLocation();
            break;
        case IDLE_STATE:{
        	//            LOG(INFO) << "!err state!";
            LOG(INFO) << "will stop!";
            resetLocationState();
            changeStateMsg(sros::core::STATE_SLAM_IDLE);
            auto last_match_pose = getLastPose();
            if (last_match_pose) {
                sendPose(last_match_pose, 0);
            }
        }

            break;
        case FALT_LOCATION_SLAMSTATE:
            LOG(INFO) << "falt!";
            processStopLocation();
            break;
        case RELOC_SLAMSTATE:
            LOG(INFO) << "reloc state!";
            changeStateMsg(sros::core::STATE_SLAM_LOCATING_AMCL);
            if (loc_output->pose) {
                sendMatchMsg(loc_output);//确定导航时候，已经发送过位姿了。
            }
            break;
        case SUCCESS_RELOC_SLAMSTATE:
            LOG(INFO) << "success_reloc!";
            if (loc_output->pose) {
                sendMatchMsg(loc_output);
            }
            changeStateMsg(sros::core::STATE_SLAM_LOCATING);
            break;
    }
}

void SlamLocProcessor::sendPose(slam::PoseSlam_Ptr pose_msg,float score) {
    if (!pose_msg) {
        LOG(INFO) << "err to get the pose!";
    }
    setLastPose(pose_msg);
    sros::core::PoseStamped_ptr pose(new sros::core::PoseStampedMsg("TOPIC_MATCHPOSE"));
    pose->pose.x() = pose_msg->x;
    pose->pose.y() = pose_msg->y;
    pose->pose.yaw() = pose_msg->yaw;
    pose->pose.z() = pose_msg->z;
    pose->pose.roll() = pose_msg->roll;
    pose->pose.pitch() = pose_msg->pitch;
    pose->time_ = pose_msg->stamp;
    pose->pose.confidence() = score;

//    Eigen::Vector3f curr_pose(pose_msg->x, pose_msg->y, pose_msg->yaw);
//    static Eigen::Vector3f last_pose = curr_pose;
//    auto delta_pose = curr_pose - last_pose;
//    bool need_update = false;
//    if (delta_pose.norm() > 0.015) {
//        need_update = true;
//        last_pose = curr_pose;
//    }
//    pose->session_id = 0;
//    static std::fstream record_loc_stream;
//    if (!record_loc_stream.is_open()) {
//        auto file_name = record::RecordFileManager::creatImgName(record::RecordFileManager::getCurrSaveTime());
//        std::string map_path = "/sros/record/";
//        std::string suffix = ".txt";
//        record::RecordFileManager::manageRecordFile(map_path, suffix, 5);
//        LOG(INFO) << "will reopen record file:" << map_path << "," << file_name;
//        record_loc_stream.open(map_path + file_name + suffix, std::ios_base::out);
//        record_loc_stream << "time,pose_x,pose_y,pose_yaw,score,delta_delay_time" << std::endl;
//        record_loc_stream.precision(5);
//    }else{
//        if (need_update) {
//            pose->session_id = 1;
//            auto curr_sstamp_time = sros::core::util::get_time_in_us();
//            auto curr_time = record::RecordFileManager::getCurrSaveTime();
//            record_loc_stream << curr_time->tm_mday<<"-"<<curr_time->tm_hour << ":" << curr_time->tm_min << ":" << curr_time->tm_sec<<"."<<record::RecordFileManager::timeinus()<<",";
//            record_loc_stream << curr_pose[0] << "," << curr_pose[1] << "," <<atan2(sin(curr_pose[2]),cos(curr_pose[2])) << "," << score
//                              << ","<<(curr_sstamp_time - pose->time_ )/1.0e6<<std::endl;
//        }
//    }

    //    LOG(INFO) << "the pose is:" << pose->pose.x() << "," << pose->pose.y() << "," << pose->pose.yaw();
    if (sendMsg) {
        sendMsg(pose);
    }
}

slam::ScanSlam_Ptr SlamLocProcessor::getSlamScan(sros::core::LaserScan_ptr scan_msg) {
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
    scan->topic = scan_msg->topic_;
    return scan;
}

void SlamLocProcessor::initialLocThread(sros::core::SlamCommandMsg &syscommand) {
    int count = 0;
    int count_thresh = 10;
    while (!scan_backup) {
        LOG(INFO) << "wait for getting the scan!";
        usleep(1e5);
        count++;
        if (count > count_thresh) {
            LOG(WARNING) << "cannot locat! will return!";
            changeStateMsg(sros::core::STATE_SLAM_IDLE);
            return;
        }
    }
    if(!curr_pgv_pose_){
        LOG(INFO)<<"will wait for pgv pose****";
        usleep(5e5);
    }
    if(curr_pgv_pose_){
        auto delta_stamp = abs(scan_backup->time_ - curr_pgv_pose_->time_);
        LOG(INFO)<<"delta stamp:"<<delta_stamp;
        if (delta_stamp > 1e6) {
            LOG(WARNING) << "the delta time between scan angd pgv is too long!";
        }else {
            LOG(INFO) << "will use pgv to realize initial locationg";
            syscommand.pose = curr_pgv_pose_->pose;
            syscommand.use_curr_pose = true;
        }
    }

    auto scan = getSlamScan(scan_backup);
    slam::LocationCmd_Ptr cmd(new slam::LocationCmdSlamMsg);
    cmd->use_curr_pose = syscommand.use_curr_pose;
    if(syscommand.slam_command == sros::core::COMMAND_START_LOCATION_MANUAL){
        cmd->cmd = slam::START_LOCATION_SLAMCMD;
    }else if(syscommand.slam_command == sros::core::COMMAND_START_RELOCATION){
        cmd->cmd = slam::START_RELOCATION_SLAMCMD;
    }
    cmd->map_name = syscommand.map_name;
    cmd->map_path = syscommand.map_path;
    map_name_ = syscommand.map_name;
    map_path_ = syscommand.map_path;
    cmd->layer_suffix = "";
    if (loc_info->location_layer != 0) {
        LOG(INFO) << "will use layer:" << loc_info->location_layer;
        cmd->layer_suffix = "_" + std::to_string(loc_info->location_layer);
    }
    cmd->pose[0] = syscommand.pose.x();
    cmd->pose[1] = syscommand.pose.y();
    cmd->pose[2] = syscommand.pose.yaw();
    cmd->scan = scan;
    cmd->stamp = scan->stamp;
    loc_processor->handleLocCmd(cmd);
}

void SlamLocProcessor::reset() {

    slam::tf::FrameToFrame base_to_odo_frame;
    base_to_odo_frame.parent_frame = "odom";
    base_to_odo_frame.child_frame = "base_link";
    tf_base_to_odo.reset(new slam::tf::TFOperator(base_to_odo_frame));
    slam::tf::FrameToFrame base_to_world_frame;
    base_to_world_frame.parent_frame = "world";
    base_to_world_frame.child_frame = "base_link";
    tf_base_to_world.reset(new slam::tf::TFOperator(base_to_world_frame));
    if (!loc_info) {
        loc_info.reset(new slam::LocationInfoSlamMsg);
        updatePara(loc_info);
    }
    fusion_code_state_ = FusionCodeState::STATE_FUSION_IDLE;
    loc_processor = slam::NormalLocationSingleton::getInstance(loc_info);//全局共享一个数据
    loc_processor->setMapOutputCallback(boost::bind(&SlamLocProcessor::locOutputCallback, this, _1));
    resetLocationState();


}

void SlamLocProcessor::processStopLocation() {
	auto last_match_pose = getLastPose();
    if (last_match_pose) {
        sendPose(last_match_pose, 0.0);
    }

    resetLocationState();
    changeStateMsg(sros::core::STATE_SLAM_ERROR);
    usleep(1e5);
    changeStateMsg(sros::core::STATE_SLAM_IDLE);
}

void SlamLocProcessor::sendMatchMsg(slam::LocationOutput_Ptr loc_output) {
//    auto& scan = loc_output->scan;

    sendPose(loc_output->pose,loc_output->score);
    if (sendMsg&&loc_output->lmk_match_info) {
        sendMsg(loc_output->lmk_match_info);
    }
}

void SlamLocProcessor::sendObstacle(slam::ScanSlam_Ptr scan) {
    return;
    slam::tf::TransForm tf;
    if (!tf_base_to_world->lookUpTransForm(scan->stamp, tf, loc_info->stamp_match_thresh)) {
        LOG(INFO) << "err to get the tf! will return!";
        return;
    }
    sros::core::ObstacleMsg_ptr m(new sros::core::ObstacleMsg("TOPIC_OBSTACLE"));
    m->oba_name = "LASER_OBA";
    auto &location_vec = m->point_cloud;
    m->car_pose.location() = tf.position;
    m->car_pose.rotation() = tf.rotation;
    if(std::isnan(tf.rotation.yaw()||!finite(tf.rotation.yaw()))) {
        LOG(WARNING) << "yaw is nan value!" << tf.rotation.yaw();
        return;
    }
    if(std::isnan(tf.position.x()||!finite(tf.position.x()))) {
        LOG(WARNING) << "x is nan value!" << tf.position.x();
        return;
    }
    if(std::isnan(tf.position.y()||!finite(tf.position.y()))) {
        LOG(WARNING) << "y is nan value!" << tf.position.y();
        return;
    }

    Eigen::Affine2f scan_to_base(Eigen::Translation2f(loc_info->laser_coordx, loc_info->laser_coordy) *
                                 Eigen::Rotation2Df(loc_info->laser_coordyaw));//获取将雷达点变换到base坐标系的坐标变换
    Eigen::Affine2f base_to_world(Eigen::Translation2f(tf.position.x(), tf.position.y()) *
                                 Eigen::Rotation2Df(tf.rotation.yaw()));//获取将雷达点变换到world坐标系的坐标变换
    Eigen::Affine2f scan_to_world = base_to_world * scan_to_base;

    auto &ranges = scan->ranges;
    float angle = scan->angle_min;
    float increment = scan->angle_increment;
    float maxrange = loc_info->laser_max_dist;
    float minrange = loc_info->laser_min_dist;
    auto &laser_min_angle = loc_info->laser_angle_min;
    auto &laser_max_angle = loc_info->laser_angle_max;

    location_vec.clear();
    location_vec.reserve(ranges.size());
    for (auto &dist:ranges) {
        if (angle > laser_min_angle && angle < laser_max_angle) {
            if ((dist > minrange) && (dist < maxrange)) {
                Eigen::Vector2f tmp_point_2d(dist * cos(angle), dist * sin(angle));
                auto world_point_2d = scan_to_world * tmp_point_2d;
                sros::core::Location point;
                point.x() = world_point_2d[0];
                point.y() = world_point_2d[1];
                point.z() = angle;
                location_vec.push_back(point);//转换到base坐标系
            }
        }
        angle += increment;
    }

    m->is_real_time_ = true;//需要主动设置为true
    m->time_ = scan->stamp;
    if (sendMsg) {
        sendMsg(m);
    }
}

bool SlamLocProcessor::handleStopLocalLocation(sros::core::SlamCommandMsg &syscommand) {
    int count = 0;
    int count_thresh = 10;
    while (!scan_backup) {
        LOG(INFO) << "wait for getting the scan!";
        usleep(1e5);
        count++;
        if (count > count_thresh) {
            LOG(INFO) << "cannot locat! will return!";
            changeStateMsg(sros::core::STATE_SLAM_IDLE);
            return false;
        }
    }
    LOG(INFO) << "will stop locallocation!";
    auto scan = getSlamScan(scan_backup);
    slam::LocationCmd_Ptr cmd(new slam::LocationCmdSlamMsg);
    cmd->cmd = slam::STOP_LOCAL_LOCATION_SLAMCMD;
    cmd->map_name = syscommand.map_name;
    cmd->map_path = syscommand.map_path;
    cmd->pose[0] = syscommand.pose.x();
    cmd->pose[1] = syscommand.pose.y();
    cmd->pose[2] = syscommand.pose.yaw();
    cmd->scan = scan;
    cmd->stamp = scan->stamp;
    loc_processor->handleLocCmd(cmd);
    return true;
}

bool SlamLocProcessor::handleStartLocalLocation(sros::core::SlamCommandMsg &syscommand) {
    int count = 0;
    int count_thresh = 10;
    while (!scan_backup) {
        LOG(INFO) << "wait for getting the scan!";
        usleep(1e5);
        count++;
        if (count > count_thresh) {
            LOG(INFO) << "cannot locat! will return!";
            changeStateMsg(sros::core::STATE_SLAM_IDLE);
            return false;
        }
    }
    LOG(INFO) << "will start local location!";
    auto scan = getSlamScan(scan_backup);
    slam::LocationCmd_Ptr cmd(new slam::LocationCmdSlamMsg);
    cmd->cmd = slam::START_LOCAL_LOCATION_SLAMCMD;
    cmd->map_name = syscommand.map_name;
    cmd->map_path = syscommand.map_path;
    cmd->pose[0] = syscommand.pose.x();
    cmd->pose[1] = syscommand.pose.y();
    cmd->pose[2] = syscommand.pose.yaw();
    cmd->scan = scan;
    cmd->stamp = scan->stamp;
    loc_processor->handleLocCmd(cmd);
    return true;
}
bool SlamLocProcessor::handleStartLocalRealTimeSlamLoc(sros::core::SlamCommandMsg &syscommand) {
    int count = 0;
    int count_thresh = 10;
    while (!scan_backup) {
        LOG(INFO) << "wait for getting the scan!";
        usleep(1e5);
        count++;
        if (count > count_thresh) {
            LOG(INFO) << "cannot locat! will return!";
            changeStateMsg(sros::core::STATE_SLAM_IDLE);
            return false;
        }
    }
    LOG(INFO) << "handleStartLocalRealTimeSlamLoc!";
    auto scan = getSlamScan(scan_backup);
    slam::LocationCmd_Ptr cmd(new slam::LocationCmdSlamMsg);
    cmd->cmd = slam::START_LOCAL_REALTIME_SLAM_SLAMCMD;
    cmd->map_name = syscommand.map_name;
    cmd->map_path = syscommand.map_path;
    cmd->pose[0] = syscommand.pose.x();
    cmd->pose[1] = syscommand.pose.y();
    cmd->pose[2] = syscommand.pose.yaw();
    cmd->scan = scan;
    cmd->stamp = scan->stamp;
    loc_processor->handleLocCmd(cmd);
    return true;
}
bool SlamLocProcessor::handleStopLocalRealTimeSlamLoc(sros::core::SlamCommandMsg &syscommand) {
    int count = 0;
    int count_thresh = 10;
    while (!scan_backup) {
        LOG(INFO) << "wait for getting the scan!";
        usleep(1e5);
        count++;
        if (count > count_thresh) {
            LOG(INFO) << "cannot locat! will return!";
            changeStateMsg(sros::core::STATE_SLAM_IDLE);
            return false;
        }
    }
    LOG(INFO) << "will stop locallocation!";
    auto scan = getSlamScan(scan_backup);
    slam::LocationCmd_Ptr cmd(new slam::LocationCmdSlamMsg);
    cmd->cmd = slam::STOP_LOCAL_REALTIME_SLAM_SLAMCMD;
    cmd->map_name = syscommand.map_name;
    cmd->map_path = syscommand.map_path;
    cmd->pose[0] = syscommand.pose.x();
    cmd->pose[1] = syscommand.pose.y();
    cmd->pose[2] = syscommand.pose.yaw();
    cmd->scan = scan;
    cmd->stamp = scan->stamp;
    loc_processor->handleLocCmd(cmd);
    return true;
}

/**
 * @brief 过滤scan断点
 * 
 * @param scan 
 */
void SlamLocProcessor::filtScans(slam::ScanSlam_Ptr scan) {
    auto& ranges = scan->ranges;
    double theta_thresh=sin((double)scan->angle_increment)/sin(0.170);//临界值,用于识别断点
    double min_thresh = 0.03f;
    int scan_size = ranges.size() - 1;
    std::vector<int> indexes;
    for (int i = 1; i < scan_size; i++) {
        if (ranges[i] == 100 || ranges[i] == 0) {
            continue;
        }
        float dist_1 = fabs(ranges[i] - ranges[i - 1]);
        float dist_2 = fabs(ranges[i + 1] - ranges[i]);
        float range_thresh_1 = ranges[i - 1] * theta_thresh + min_thresh;
        float range_thresh_2 = ranges[i + 1] * theta_thresh + min_thresh;
        if (dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
            indexes.push_back(i);
        }
    }
    for (auto &index:indexes) {
        ranges[index] = 100;
    }
}

void SlamLocProcessor::processPgvPose(sros::core::base_msg_ptr base_ptr) {
    curr_pgv_pose_ = std::dynamic_pointer_cast<sros::core::PoseStampedMsg>(base_ptr);
}
bool SlamLocProcessor::handleRelocation(sros::core::SlamCommandMsg &syscommand) {
    resetLocationState();
    return handleStartLocation(syscommand);
}

void SlamLocProcessor::updatePara(std::shared_ptr<slam::LocationInfoSlamMsg> &loc_info) {
    auto &s = sros::core::Settings::getInstance();
    loc_info->laser_min_dist = s.getValue("slam.map_laser_min_dist", 0.1);
    loc_info->laser_max_dist = s.getValue("slam.map_laser_max_dist", 30.0);
    loc_info->tilt_angle_thresh = s.getValue("slam.tilt_angle_thresh", 0.5);
    loc_info->laser_angle_min = s.getValue("slam.laser_angle_min", -2.37);
    loc_info->laser_angle_max = s.getValue("slam.laser_angle_max", 2.37);
    loc_info->pose_percentage_thresh = s.getValue("slam.pose_percentage_thresh", 0.5);
    loc_info->laser_z_min = s.getValue("slam.laser_z_min", 0.1);
    loc_info->laser_z_max = s.getValue("slam.laser_z_max", 0.5);
    loc_info->map_resolution = s.getValue("slam.map_resolution", 0.02);
    loc_info->map_multi_level = s.getValue("slam.map_multi_level", 3);
    loc_info->stamp_match_thresh = s.getValue("slam.stamp_match_thresh", 60000);
    loc_info->ini_angle_count = s.getValue("slam.ini_angle_count", 40);
    loc_info->ini_distri_size = s.getValue("slam.ini_distri_size", 2.0);
    loc_info->use_pitch_roll_correct = s.getValue<int>("slam.use_pitchroll_correct", 0);
    loc_info->consistent_min_err = s.getValue("slam.consistent_min_err", 0.1);
    loc_info->consistent_count_thresh = s.getValue("slam.consistent_count_thresh", 3);
    loc_info->consistent_pose_size = s.getValue("slam.consistent_pose_size", 20);
    loc_info->use_h_optimize = s.getValue("slam.use_h_optimize", 1);
    loc_info->use_pose_record = s.getValue("slam.use_pose_record", 0);
    loc_info->use_rt_slam_method = s.getValue("slam.use_rt_slam_method", 1);
    loc_info->rt_slam_range_min = s.getValue("slam.rt_slam_range_min", 1.0);
    loc_info->lmk_type = s.getValue<std::string>("slam.lmk_extract_type", "CYLINDER");
    loc_info->always_update_pose = s.getValue("slam.always_update_pose", 0);
    loc_info->realtime_update_map = s.getValue("slam.realtime_update_map", 1);
    loc_info->realtime_update_map_max_dist = s.getValue("slam.realtime_update_max_dist", 15.0);

    loc_info->laser_coordx = s.getValue("posefilter.laser_coordx", 0.29);
    loc_info->laser_coordy = s.getValue("posefilter.laser_coordy", 0.0);
    loc_info->laser_coordyaw = s.getValue("posefilter.laser_coordyaw", 0.0);
    loc_info->y_middlet_err_thresh = s.getValue("posefilter.y_middle_err_thresh", 0.08);
    loc_info->y_small_err_thresh = s.getValue("posefilter.y_small_err_thresh", 0.03);
    loc_info->k_theta_small_err = s.getValue("posefilter.k_theta_small_err", 0.1);
    loc_info->k_theta_big_err = s.getValue("posefilter.k_theta_big_err", 0.6);
    loc_info->k_y_small_err = s.getValue("posefilter.k_y_small_err", 0.4);
    loc_info->k_y_middle_err = s.getValue("posefilter.k_y_middle_err", 0.2);
    loc_info->k_y_large_err = s.getValue("posefilter.k_y_large_err", 0.1);
    loc_info->x_small_err_thresh = s.getValue("posefilter.x_small_err_thresh", 0.05);
    loc_info->x_middle_err_thresh = s.getValue("posefilter.x_middle_err_thresh", 0.2);
    loc_info->k_x_middle_err = s.getValue("posefilter.k_x_middle_err", 0.2);
    loc_info->k_x_small_err = s.getValue("posefilter.k_x_small_err", 1.0);
    loc_info->k_x_large_err = s.getValue("posefilter.k_x_large_err", 0.1);
    loc_info->theta_small_err_thresh = s.getValue("posefilter.theta_small_err_thresh", 0.087);
    loc_info->debug_info_print_flag = s.getValue("posefilter.debug_info_print_flag", 0);
    loc_info->pose_dist_err_thresh = s.getValue("posefilter.pose_dist_error_thresh", 0.08);
    loc_info->pose_angle_err_thresh = s.getValue("posefilter.pose_angle_error_thresh", 0.1);
    loc_info->enable_pose_fusion = s.getValue("posefilter.enable_pose_fusion_flag", 1);

    loc_info->location_layer = s.getValue<int>("slam.location_layer", 0);

    loc_info->breakPt_dist_err = s.getValue<float>("slam.corridor_breakPt_dist_err", 0.05);
    loc_info->beacon_feature_err = s.getValue<float>("slam.corridor_beacon_feature_err", 0.03);
    loc_info->beacon_lenght_err = s.getValue<float>("slam.corridor_beacon_lenght_err", 0.2);

    auto code_string = s.getValue<std::string>("slam.corridor_landmark_code_feature", "0.1,0.53,0.1;");
    auto code_str = splitStr(code_string,';');
    std::vector<std::vector<float>> lmk_code;
    for(const auto& code : code_str)
    {
        auto dist_str = splitStr(code,',');
        std::vector<float> dist;
        for(const auto& d : dist_str)
        {
            dist.emplace_back(std::stof(d));
        }
        lmk_code.emplace_back(std::move(dist));
    }
    loc_info->corridor_endpoint_feature_value = lmk_code;

    auto beacon_str = s.getValue<std::string>("slam.corridor_single_beacon_feature", "0.1,0.05;");
    auto beacon_code_str = splitStr(beacon_str, ';');
    std::vector<std::vector<float>> single_lmk_code;
    for (const auto &code : beacon_code_str)
    {
        auto dist_str = splitStr(code, ',');
        std::vector<float> dist;
        for (const auto &d : dist_str)
        {
            dist.emplace_back(std::stof(d));
        }
        single_lmk_code.emplace_back(std::move(dist));
    }
    loc_info->single_beacon_feature_value = single_lmk_code;

    auto lidar_type = s.getValue<int>("slam.lidar_type", 0x03);
    if (lidar_type == 0x03 || lidar_type == 0x07) {
        need_filter_noise_points = true;
    }else{
        need_filter_noise_points = false;
    }

    computeAligenSensorTF(loc_info);
    LOG(INFO) << "slam.lidar type:" << lidar_type;

    LOG(INFO) << "para info:" << loc_info->laser_min_dist << "," << loc_info->laser_max_dist << ","
              << loc_info->tilt_angle_thresh << "," << loc_info->laser_angle_min << "," << loc_info->laser_angle_max
              << "," << loc_info->pose_percentage_thresh << "," << loc_info->laser_z_min << "," << loc_info->laser_z_max
              << "," << loc_info->map_resolution << "," << loc_info->map_multi_level << ","
              << loc_info->stamp_match_thresh << ",\n" <<
              loc_info->ini_angle_count << ","<<
              loc_info->ini_distri_size << ","<<
              loc_info->use_pitch_roll_correct << ","<<
              loc_info->consistent_min_err << ","<<
              loc_info->consistent_count_thresh << ","<<
              loc_info->consistent_pose_size << ","<<
              loc_info->use_h_optimize << ","<<
              loc_info->use_pose_record << ","<<
              loc_info->use_rt_slam_method << ","<<
              loc_info->rt_slam_range_min << ","<<
              loc_info->lmk_type << ","<<
              loc_info->always_update_pose << ","<<
              loc_info->realtime_update_map << ","<<
              loc_info->realtime_update_map_max_dist << ",\n"<<
              loc_info->laser_coordx << ","<<
              loc_info->laser_coordy << ","<<
              loc_info->laser_coordyaw << ","<<
              loc_info->y_middlet_err_thresh << ","<<
              loc_info->y_small_err_thresh << ","<<
              loc_info->k_theta_small_err << ","<<
              loc_info->k_theta_big_err << ","<<
              loc_info->k_y_small_err << ","<<
              loc_info->k_y_middle_err << ","<<
              loc_info->k_y_large_err << ",\n"<<
              loc_info->x_small_err_thresh << ","<<
              loc_info->x_middle_err_thresh << ","<<
              loc_info->k_x_middle_err << ","<<
              loc_info->k_x_small_err << ","<<
              loc_info->k_x_large_err << ","<<
              loc_info->theta_small_err_thresh << ","<<
              loc_info->debug_info_print_flag << ","<<
              loc_info->pose_dist_err_thresh << ","<<
              loc_info->pose_angle_err_thresh << ","<<
              loc_info->location_layer << ","<<
              loc_info->enable_pose_fusion;
}

static Eigen::Vector3d computeTFVel(const slam::tf::TransForm &old_tf, const slam::tf::TransForm &new_tf){
    auto delta_yaw = new_tf.rotation.yaw() - old_tf.rotation.yaw();
    delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));
    Eigen::Vector2d delta_point, back_point(new_tf.position.x(), new_tf.position.y());
    old_tf.transform2DPointByInverse(back_point, delta_point);
    auto delta_stamp = new_tf.pose_time - old_tf.pose_time;
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    if (delta_stamp != 0) {
        vel[0] = delta_point[0] / delta_stamp;
        vel[1] = delta_point[1] / delta_stamp;
        vel[2] = delta_yaw / delta_stamp;
    }
    return vel;
}

static Eigen::Vector3d computeTFVelWithWeight(const slam::tf::TransForm &old_tf, const slam::tf::TransForm &new_tf,const int64_t& cand_stamp,
                                              double &weight) {
    auto delta_yaw = new_tf.rotation.yaw() - old_tf.rotation.yaw();
    delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));
    Eigen::Vector2d delta_point, back_point(new_tf.position.x(), new_tf.position.y());
    old_tf.transform2DPointByInverse(back_point, delta_point);
    auto delta_stamp = new_tf.pose_time - old_tf.pose_time;
    auto weight_stamp = new_tf.pose_time - cand_stamp;
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    if (delta_stamp != 0) {
        vel[0] = delta_point[0] / delta_stamp;
        vel[1] = delta_point[1] / delta_stamp;
        vel[2] = delta_yaw / delta_stamp;
        weight = 5.0e3 / ((double)abs(weight_stamp) + 5.0e3);
    }else{
        weight = 1.0;
    }
    return vel;
}

static Eigen::Vector3d computeTFVelByInterpolate(const slam::tf::TransForm &old_tf, const slam::tf::TransForm &new_tf){
    auto delta_yaw = new_tf.rotation.yaw() - old_tf.rotation.yaw();
    delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));
    Eigen::Vector2d delta_point, back_point(new_tf.position.x(), new_tf.position.y());
    old_tf.transform2DPointByInverse(back_point, delta_point);
    auto delta_stamp = new_tf.pose_time - old_tf.pose_time;
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    if (delta_stamp != 0) {
        vel[2] = delta_yaw / delta_stamp;
        if (fabs(delta_yaw) > 0.00002) {
            vel.head<2>() = delta_point*(0.5*vel[2]/sin(delta_yaw*0.5));
        }else{
            vel[0] = delta_point[0] / delta_stamp;
            vel[1] = delta_point[1] / delta_stamp;
        }
    }
    return vel;
}

static const Eigen::Vector3d addPose(const Eigen::Vector3d& base_pose,const Eigen::Vector3d& delta_pose){
    auto cos_theta = cos(base_pose[2]);
    auto sin_theta = sin(base_pose[2]);
    Eigen::Vector3d tf_pose;
    tf_pose[0] = cos_theta * delta_pose[0] - sin_theta * delta_pose[1] + base_pose[0];
    tf_pose[1] = sin_theta * delta_pose[0] + cos_theta * delta_pose[1] + base_pose[1];
    tf_pose[2] = base_pose[2] + delta_pose[2];
    tf_pose[2] = fmod(tf_pose[2], M_PI * 2);
    return tf_pose;
}

static Eigen::Vector3d computeTFVelByWeight(const std::vector<slam::tf::TransForm> &tfs, const int64_t &cand_time){
    int tfs_size = tfs.size();
    int pair_size = tfs_size / 2;
    Eigen::Vector3d sum_vel;
    sum_vel.setZero();
    double sum_weight = 0.0;
    for (int i = 0; i < pair_size; ++i) {
        double weight = 1.0;
        sum_vel += computeTFVelWithWeight(tfs[i], tfs[i + pair_size], cand_time, weight) * weight;
        sum_weight += weight;
    }
    if (sum_weight > 0.0) {
        sum_vel /= sum_weight;
        if(std::isnan(sum_vel[0])||std::isnan(sum_vel[2])){
            LOG(ERROR) << "delta pose is wrong!" << sum_vel[0] << "," << sum_vel[2] << "," << cand_time << ","
                       << sum_weight;
        }
        return sum_vel;
    }
    return Eigen::Vector3d::Zero();
}

static Eigen::Vector3d computeDeltaPoseByInterpolate(const Eigen::Vector3d &vel,const int64_t delta_stamp){
    auto delta_yaw = vel[2] * delta_stamp;
    Eigen::Vector3d delta_pose;
    delta_pose[2] = delta_yaw;
    if(fabs(vel[2])>0.00002) {
        delta_pose.head<2>() = 2 * vel.head<2>() / vel[2] * sin(delta_yaw / 2.0);
    }else{
        delta_pose.head<2>() = vel.head<2>() * delta_stamp;
    }
    if(std::isnan(delta_pose[0])||std::isnan(delta_pose[2])){
        LOG(ERROR) << "delta pose is wrong!" << delta_pose[0] << "," << delta_pose[2] << "," << vel[2];
    }
    return delta_pose;
}

Eigen::Vector3d SlamLocProcessor::computeDeltaOdoPose(const int64_t &old_time,const int64_t &new_time,int range_index) {
    slam::tf::TransForm odo_tf_1,odo_tf_2;
    std::vector<slam::tf::TransForm> tfs;
    static int64_t curr_stamp = sros::core::util::get_time_in_us();
    if (abs(new_time - old_time) > 1e5) {
//        LOG(WARNING) << "delta old time too large!" << abs(new_time - old_time);
    }
    if (!tf_base_to_odo->lookUpMeanTransForm(old_time, odo_tf_1,range_index, loc_info->stamp_match_thresh)) {
        LOG(INFO) << "err to get the tf! will return!" << old_time << "," << scan_backup->time_;
        return Eigen::Vector3d::Zero();
    }
    if (!tf_base_to_odo->lookUpMeanTransForm(new_time, odo_tf_2, range_index,loc_info->stamp_match_thresh)) {
        LOG(INFO) << "err to get the tf! will return!" << new_time << "," << scan_backup->time_;
        return Eigen::Vector3d::Zero();
    }
    int64_t newest_time = new_time > old_time ? new_time : old_time;
    const int tfs_size = 10;
    if (!tf_base_to_odo->lookUpTransForms(newest_time, tfs, tfs_size, loc_info->stamp_match_thresh)) {
        LOG(INFO) << "err to get the tf! will return!" << new_time << "," << scan_backup->time_;
        return Eigen::Vector3d::Zero();
    }
    if (tfs.size() == tfs_size) {
        auto old_vel = computeTFVelByWeight(tfs, old_time);
        auto new_vel = computeTFVelByWeight(tfs, new_time);
        Eigen::Vector3d old_time_delta_pose,new_time_delta_pose;
        int64_t old_delta_time =  odo_tf_1.pose_time - old_time;
        int64_t new_delta_time = new_time - odo_tf_2.pose_time;
        int64_t delta_time = odo_tf_2.pose_time - odo_tf_1.pose_time;
        old_time_delta_pose = computeDeltaPoseByInterpolate(old_vel, old_delta_time);
        new_time_delta_pose = computeDeltaPoseByInterpolate(new_vel, new_delta_time);
        auto vel_delta_pose = computeDeltaPoseByInterpolate((old_vel + new_vel) / 2.0, delta_time);

        Eigen::Vector2d delta_point;
        odo_tf_1.transform2DPointByInverse(Eigen::Vector2d(odo_tf_2.position.x(), odo_tf_2.position.y()), delta_point);
        double vel_delta_yaw = vel_delta_pose[2] ;
        double odo_delta_yaw = odo_tf_2.rotation.yaw() - odo_tf_1.rotation.yaw();
        auto delta_yaw = odo_delta_yaw;
        if (fabs(delta_yaw) > 0.01) {
            delta_yaw = fabs(vel_delta_yaw) < fabs(odo_delta_yaw) ? vel_delta_yaw : odo_delta_yaw;
        }

        auto delta_pose = addPose(old_time_delta_pose, Eigen::Vector3d(delta_point[0], delta_point[1], delta_yaw));
        delta_pose = addPose(delta_pose,new_time_delta_pose);
        if(std::isnan(delta_pose[0])||std::isnan(delta_pose[2])){
            LOG(ERROR) << "delta pose is wrong!" << delta_pose[0] << "," << delta_pose[1] << "," << delta_pose[2] << ","
                       << old_time_delta_pose[0] << "," << old_time_delta_pose[1] << "," << old_time_delta_pose[2]
                       << "," << new_time_delta_pose[0] << "," << new_time_delta_pose[1] << ","
                       << new_time_delta_pose[2];
        }
        if (fabs(delta_pose[2]) > 0.008) {
            if (delta_pose.head<2>().norm() > 0.02) {
                LOG(INFO) << "detect move large delta yaw： " << delta_pose[2]*180/3.14 ;
                // delta_pose[2] *= 0.7;
            }else{
//                LOG(INFO) << "detect rotate large delta yaw!";
            }
        }
        return delta_pose;
    }else{
        Eigen::Vector2d delta_point;
        odo_tf_1.transform2DPointByInverse(Eigen::Vector2d(odo_tf_2.position.x(), odo_tf_2.position.y()), delta_point);
        double delta_yaw = odo_tf_2.rotation.yaw() - odo_tf_1.rotation.yaw();
        return Eigen::Vector3d(delta_point[0], delta_point[1], delta_yaw);
    }
}


void SlamLocProcessor::processDmCode(sros::core::base_msg_ptr base_ptr) {
    auto code_info = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(base_ptr);
    if (code_info->state_ == DM_CODE_DETECTED) {
        if (getLocationState()) {
            int64_t last_scan_stamp = 0;
            Eigen::Vector3f last_scan_pose;
            if (loc_processor->getLastMatchedPoseStamp(last_scan_stamp, last_scan_pose)) {
                auto delta_pose = computeDeltaOdoPose(code_info->time_, last_scan_stamp, 4);
                slam::LocationCodeSlammsg_Ptr code_msg(new slam::LocationCodeSlammsg);
                code_msg->matched_scan_time = last_scan_stamp;
                code_msg->code_time = code_info->time_;
                code_msg->code_id = code_info->code_str_;
                code_msg->code_type = code_info->code_type_;
                code_msg->device_name = code_info->camera_name_;

                Eigen::Vector3f match_world_pose;
                auto realtime_world_pose_ptr = getLastPose();
                Eigen::Vector3d realtime_world_pose(realtime_world_pose_ptr->x, realtime_world_pose_ptr->y,
                                                    realtime_world_pose_ptr->yaw);
                if (last_scan_stamp != realtime_world_pose_ptr->stamp) {
                    auto delta_last_to_code = computeDeltaOdoPose(realtime_world_pose_ptr->stamp, last_scan_stamp, 4);
                    //变换到scan_stamp下，整个体系里，时间戳统一是scan_stamp
                    match_world_pose = addPose(realtime_world_pose, delta_last_to_code).cast<float>();
                } else {
                    match_world_pose = realtime_world_pose.cast<float>();
                }

                code_msg->match_world_pose = match_world_pose;

                //当前dmcode 保存的是二维码相对于相机的位姿
                slam::tf::TransForm code_in_center_tf;
                code_in_center_tf.position.x() = code_info->x_;  // TODO:注意单位的影响
                code_in_center_tf.position.y() = code_info->y_;
                code_in_center_tf.position.z() = code_info->z_;
                code_in_center_tf.rotation.yaw() = code_info->angle_;
                code_in_center_tf.rotation.roll() = code_info->roll_;
                code_in_center_tf.rotation.pitch() = code_info->pitch_;

                code_msg->scan_pose_in_world.buildTFfrom2DPose(last_scan_pose);
                code_msg->delta_pose_to_scan.buildTFfrom2DPose(delta_pose.cast<float>());
                code_msg->code_pose_in_center = code_in_center_tf;

                // TODO:判断是录制过程，还是融合过程
                if (fusion_code_state_ == STATE_FUSION_CODE || fusion_code_state_ == STATE_ALIGNMENT) {
                    loc_processor->fusionWithCodeMsg(code_msg);
                } else if (fusion_code_state_ == STATE_FUSION_RECORD_CODE) {
                    loc_processor->recordCodeMsg(code_msg);
                }

                computeLocationError(base_ptr, code_msg); 
            }
        } else if (location_system_state == sros::core::STATE_SLAM_IDLE) {
            bool enable_code_fusion = sros::core::Settings::getInstance().getValue<std::string>(
                                          "slam.enable_dm_code_location", "False") == "True";
            if (enable_code_fusion) {
                if (!(map_path_ + map_name_).empty()) {
                    if (loc_processor) {
                        Eigen::Vector3f origin_code, world_pose;
                        origin_code[0] = code_info->x_;
                        origin_code[1] = code_info->y_;
                        origin_code[2] = code_info->angle_;
                        Eigen::Affine2f tf_inv =
                            Eigen::Translation2f(origin_code[0], origin_code[1]) * Eigen::Rotation2Df(origin_code[2]);
                        auto curr_code = tf_inv.inverse()*Eigen::Vector2f(0,0);
                        origin_code.head<2>() = curr_code;
                        origin_code[2] = -origin_code[2];
                        if (loc_processor->convertDMCodeToWorld(map_path_, map_name_, origin_code,
                                                                code_info->code_str_, world_pose)) {
                            LOG(INFO) << "get dm code initial pose:" << world_pose[0] << "," << world_pose[1] << ","
                                      << world_pose[2];
                            sros::core::slam_command_msg_ptr slam_cmd(new sros::core::SlamCommandMsg);
                            slam_cmd->map_path = map_path_;
                            slam_cmd->map_name = map_name_;
                            slam_cmd->pose.x() = world_pose[0];
                            slam_cmd->pose.y() = world_pose[1];
                            slam_cmd->pose.yaw() = world_pose[2];
                            slam_cmd->use_curr_pose = true;
                            slam_cmd->slam_command = sros::core::COMMAND_START_LOCATION_MANUAL;
                            processSystemCmd(slam_cmd);
                        }
                    }
                }
            }
        }
    }
}

void SlamLocProcessor::computeLocationError(sros::core::base_msg_ptr base_ptr, slam::LocationCodeSlammsg_Ptr code_msg) {
    auto code_info = std::dynamic_pointer_cast<sros::core::DataMatrixCodeMsg>(base_ptr);
    auto realtime_world_pose_ptr = getLastPose();
    Eigen::Vector3d realtime_world_pose(realtime_world_pose_ptr->x, realtime_world_pose_ptr->y,
                                                    realtime_world_pose_ptr->yaw);
    slam::tf::TransForm pick_time_cam_pose, pick_time_cen_pose_inv;
    // code_msg->code_pose_in_world.transformTF(code_msg->code_pose_in_center, pick_time_cam_pose);

    code_msg->code_pose_in_center.transformTFByInverse(pick_time_cen_pose_inv, pick_time_cen_pose_inv);
    code_msg->code_pose_in_world.transformTF(pick_time_cen_pose_inv, pick_time_cam_pose);

    Eigen::Vector3f err_pose;

    err_pose[0] = realtime_world_pose[0] - pick_time_cam_pose.position.x();
    err_pose[1] = realtime_world_pose[1] - pick_time_cam_pose.position.y();   
    err_pose[2] = realtime_world_pose[2] - pick_time_cam_pose.rotation.yaw();
      
    code_info->loc_x_err_ = err_pose[0];
    code_info->loc_y_err_ = err_pose[1];    
    code_info->loc_yaw_err_ = err_pose[2];

    // LOG(INFO) << "判断定位正确性: x: " << err_pose[0] << " y: " << err_pose[1] << " yaw: " << err_pose[2]*180/3.14;               
    // LOG(INFO) << "实时定位： x： " << realtime_world_pose[0] << " y: " << realtime_world_pose[1] << " yaw: " << realtime_world_pose[2]*180/3.14;
    // LOG(INFO) << "二维码位置： x: " << code_msg->code_pose_in_world.position.x() << " y: " << code_msg->code_pose_in_world.position.y() << " yaw:"
    //                               << code_msg->code_pose_in_world.rotation.yaw()*180/3.14;
    // LOG(INFO) << "二维码中心位置： x: " << code_msg->code_pose_in_center.position.x() << " y: " << code_msg->code_pose_in_center.position.y() << " yaw: "
    //                                 << code_msg->code_pose_in_center.rotation.yaw()*180/3.14;  
    // LOG(INFO) << "二维码给出定位： x: " << pick_time_cam_pose.position.x() << " y: " << pick_time_cam_pose.position.y() << " yaw: " 
    //                                  << pick_time_cam_pose.rotation.yaw()*180/3.14;

}

std::vector<double> splitStrsToDoubles(const std::string &s, const char seperator) {
    std::vector<double> result;
    typedef std::string::size_type string_size;

    string_size i = 0;
    string_size j = 0;
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
                    double item = stof(item_s);
                    result.push_back(item);
                } catch (std::exception &e) {
                    LOG(INFO) << "throw error:" << e.what() << item_s;
                }
            }
            j = i + 1;
        }
        i++;
    }
    return result;
}

void SlamLocProcessor::computeAligenSensorTF(std::shared_ptr<slam::LocationInfoSlamMsg> &loc_info) {
    auto resolveTF = [&](const std::string& tf_para){
        slam::tf::TransForm tf;
        auto paras = splitStrsToDoubles(tf_para,';');
        int para_size = paras.size();
        int curr_index = 0;
        if (para_size > curr_index) {
            tf.position.x() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.position.y() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.position.z() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.rotation.yaw() = paras[curr_index++];//先解算yaw，后解算roll
        }
        if (para_size > curr_index) {
            tf.rotation.pitch() = paras[curr_index++];
        }
        if (para_size > curr_index) {
            tf.rotation.roll() = paras[curr_index++];
        }
        return tf;
    };

    auto &s = sros::core::Settings::getInstance();
    loc_info->use_scan_to_alignment = (s.getValue<std::string>("slam.use_scan_to_alignment", "True") == "True");
    if (loc_info->use_scan_to_alignment) {
        auto device_name = sros::device::DEVICE_LIDAR;
        loc_info->laser_coordx = s.getValue("posefilter.laser_coordx", 0.29);
        loc_info->laser_coordy = s.getValue("posefilter.laser_coordy", 0.0);
        loc_info->laser_coordyaw = s.getValue("posefilter.laser_coordyaw", 0.0);
        slam::tf::TransForm laser_tf;
        laser_tf.position.x() = loc_info->laser_coordx;
        laser_tf.position.y() = loc_info->laser_coordy;
        laser_tf.rotation.yaw() = loc_info->laser_coordyaw;
        sensor_transforms_[device_name] = laser_tf;
    }
    loc_info->use_up_camera_to_alignment = (s.getValue<std::string>("slam.use_up_camera_to_alignment", "False") == "True");
    if(loc_info->use_up_camera_to_alignment){
        auto device_name = sros::device::DEVICE_SVC100_UP;
        slam::tf::TransForm svc100_up_tf;
        sensor_transforms_[device_name] = svc100_up_tf;
    }
    loc_info->use_down_camera_to_alignment = (s.getValue<std::string>("slam.use_down_camera_to_alignment", "True") == "True");
    if(loc_info->use_down_camera_to_alignment){
        auto device_name = sros::device::DEVICE_SVC100_DOWN;
        slam::tf::TransForm svc100_down_tf;
        sensor_transforms_[device_name] = svc100_down_tf;
    }
    loc_info->use_forward_camera_to_alignment = (s.getValue<std::string>("slam.use_forward_camera_to_alignment", "True") == "True");
    if (loc_info->use_forward_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_FORWARD;
        std::string tf_str = s.getValue<std::string>("camera.forward_camera_install_pose","0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
    loc_info->use_backward_camera_to_alignment = (s.getValue<std::string>("slam.use_backward_camera_to_alignment", "True") == "True");
    if (loc_info->use_backward_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_BACKWARD;
        std::string tf_str = s.getValue<std::string>("camera.backward_camera_install_pose","0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
    loc_info->use_left_camera_to_alignment = (s.getValue<std::string>("slam.use_left_camera_to_alignment", "True") == "True");
    if (loc_info->use_left_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_LEFT;
        std::string tf_str = s.getValue<std::string>("camera.left_camera_install_pose","0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
    loc_info->use_right_camera_to_alignment = (s.getValue<std::string>("slam.use_right_camera_to_alignment", "True") == "True");
    if(loc_info->use_right_camera_to_alignment) {
        auto device_name = sros::device::DEVICE_CAMERA_RIGHT;
        std::string tf_str = s.getValue<std::string>("camera.right_camera_install_pose","0;");
        sensor_transforms_[device_name] = resolveTF(tf_str);
    }
}
void SlamLocProcessor::processAlignment(sros::core::base_msg_ptr base_ptr) {
    auto location_cmd = std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::string>>(base_ptr);
    LOG(INFO) << "cmd info:" << location_cmd->str0 << "," << location_cmd->command;
    slam::tf::TransForm feature_pose(0, location_cmd->pose);
    if(location_cmd->command == "STOP_ALIGNMENT"){
        fusion_code_state_ = STATE_FUSION_IDLE;
        loc_processor->removeCodeMsg(feature_pose, location_cmd->str0);
        LOG(INFO) << "stop alignment!";
    }else if(location_cmd->command == "START_ALIGNMENT"){
        if (fusion_code_state_ == STATE_FUSION_CODE) {
            if (last_fusion_devices_.empty()) {
                LOG(INFO) << "enter an unexpected state! will try to recover!";
                location_cmd->command = "START_EXTRACTOR";
                location_cmd->topic_ = "TOPIC_EXTRACT_COMMAND";
                sendMsg(location_cmd);
            }
        }
//        if (loc_processor) {//暂时先不考虑清除
//            loc_processor->resetFusionState();
//        }
        fusion_code_state_ = STATE_ALIGNMENT;
        loc_processor->recordCodeMsg(feature_pose, location_cmd->str0);
    }else if(location_cmd->command == "START_RECORD_DM_CODE"){
        if (loc_processor) {
            loc_processor->resetFusionState();
        }
        fusion_code_state_ = STATE_FUSION_RECORD_CODE;
    }else if(location_cmd->command == "START_FUSION_DM_CODE"){
        fusion_code_state_ = STATE_FUSION_CODE;
    }else if(location_cmd->command == "STOP_RECORD_DM_CODE"){
        loc_processor->saveCodeMsg();
        fusion_code_state_ = STATE_FUSION_IDLE;
    }else if(location_cmd->command == "STOP_FUSION_DM_CODE"){
        if (loc_processor) {
            loc_processor->resetFusionState();
        }
        fusion_code_state_ = STATE_FUSION_IDLE;
    }
}
void SlamLocProcessor::onTime50msLoop() {
    auto sendStartFusionCmd = [&](std::string device_name){
      auto cmd = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
      cmd->command = "START_FUSION_DM_CODE";
      cmd->str1 = device_name;
      sendMsg(cmd);
    };

    auto sendStartRecordCmd = [&](std::string device_name){
      auto cmd = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
      cmd->command = "START_RECORD_DM_CODE";
      cmd->str1 = device_name;
      sendMsg(cmd);
    };

    auto sendStopRecordCmd = [&](std::string device_name){
      auto cmd = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
      cmd->command = "STOP_RECORD_DM_CODE";
      cmd->str1 = device_name;
      sendMsg(cmd);
    };

    auto sendStopFusionCmd = [&](std::string device_name){
      auto cmd = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
      cmd->command = "STOP_FUSION_DM_CODE";
      cmd->str1 = device_name;
      sendMsg(cmd);
    };
    auto &s = sros::core::Settings::getInstance();
    auto use_fusion = s.getValue<std::string>("slam.enable_dm_slam_fusion","False") == "True";
    if (!getLocationState()||!use_fusion) {
        return;
    }

    auto curr_record_device = s.getValue<std::string>("slam.record_dm_code_info","False");
    bool enable_code_fusion = s.getValue<std::string>("slam.enable_dm_code_location", "False") == "True";
    if (!enable_code_fusion) {
        if (loc_processor) {
            loc_processor->resetFusionState();
        }
        fusion_code_state_ = STATE_FUSION_IDLE;
        return;
    }
    std::string device_name = curr_record_device;
    if(curr_record_device == "DOWN_CAMERA"){
        device_name = sros::device::DEVICE_SVC100_DOWN;
    }else if(curr_record_device == "LIDAR"){
        device_name = sros::device::DEVICE_LIDAR;
    }
    if (curr_record_device == "False") {
        auto last_match_pose = getLastPose();
        if (last_match_pose) {
            Eigen::Vector3f curr_pose(last_match_pose->x, last_match_pose->y, last_match_pose->yaw);
            if (loc_processor) {
                std::string device_name;
                if(loc_processor->nearFusionCode(curr_pose, 2.0,device_name)){//TODO:添加devicename存储
                    if (fusion_code_state_ == STATE_FUSION_IDLE) {
                        LOG(INFO) << "send start fusion cmd!" << device_name;
                        sendStartFusionCmd(device_name);
                        last_fusion_devices_.insert(device_name);
                    } else if (fusion_code_state_ == STATE_FUSION_CODE) {
                        if (!last_fusion_devices_.empty()) {
                            if (last_fusion_devices_.find(device_name) == last_fusion_devices_.end()) {
                                LOG(INFO) << "send start fusion cmd!" << device_name;
                                sendStartFusionCmd(device_name);
                                last_fusion_devices_.insert(device_name);
                            }
                        }
                    }
                }else{
                    if (fusion_code_state_ == STATE_FUSION_CODE) {
                        if(last_fusion_devices_.size()){
                            for (auto &fusion : last_fusion_devices_) {
                                sendStopFusionCmd(fusion);
                            }
                            last_fusion_devices_.clear();
                        }
                    }
                }
            }
        }
    }else{
        if (fusion_code_state_ == STATE_FUSION_CODE) {
            sendStopFusionCmd(device_name);
        }
    }

    if (last_record_device_ != device_name) {
        if (device_name == "False") {
            LOG(INFO) << "stop!";
            sendStopRecordCmd(last_record_device_);
        }else{
            LOG(INFO) << "record:" << device_name;
            sendStartRecordCmd(device_name);
        }
        last_record_device_ = device_name;
    }
}

void SlamLocProcessor::setLastPose(const slam::PoseSlam_Ptr &pose_msg) {
    boost::mutex::scoped_lock lock(last_match_mutex);
    last_match_pose_ = pose_msg;
}

slam::PoseSlam_Ptr SlamLocProcessor::getLastPose() {
    boost::mutex::scoped_lock lock(last_match_mutex);
    return last_match_pose_;
}
void SlamLocProcessor::reloadupdatedMap(const std::string map_path,const std::string map_name){
    if (getLocationState()) {
        std::string layer_suffix;
        if (loc_info->location_layer != 0) {
            LOG(INFO) << "will use layer:" << loc_info->location_layer;
            layer_suffix = "_" + std::to_string(loc_info->location_layer);
        }
        loc_processor->reloadMap(map_path,map_name,layer_suffix);
    }
}
}
