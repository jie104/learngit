//
// Created by lfc on 17-7-10.
//

#include <core/msg/laser_scan_msg.hpp>
#include <core/settings.h>
#include <core/msg/PoseStampedMsg.h>
#include <core/msg/ObstacleMsg.hpp>
//#include <bits/signum.h>
#include <signal.h>
#include <execinfo.h>
#include "lmk_loc_processor.h"
#include "modules/slam/include/lmkslam/lmk_module_singleton.h"
#include "boost/thread.hpp"
#include "modules/slam/include/lmkslam/debug_tool/RecordFunc.hpp"
#include "../debug_tool/time_tool.h"

namespace location {

bool LmkLocProcessor::handleStartLocation(sros::core::SlamCommandMsg &syscommand) {

    LOG(INFO) << "will creat new thread to process initial pose!";
    using namespace sros::core;
    if (!lmk_module_ptr) {
        getScanToOdoTF();
        lmk_module_ptr = slam::LmkModuleSingleton::getInstance(lmkslam_para);

        lmk_module_ptr->setLocOutputCallback(boost::bind(&LmkLocProcessor::mapOutputCallback, this, _1));

    }
    changeStateMsg(STATE_SLAM_LOCATING_AMCL);
    boost::thread(boost::bind(&LmkLocProcessor::processInitialPoseThread, this, syscommand));

    return true;
}

bool LmkLocProcessor::handleStopLocation(sros::core::SlamCommandMsg &syscommand) {
    return processStopLoc();
}

bool LmkLocProcessor::processPara(sros::core::base_msg_ptr base_ptr) {
    return LocationProcessor::processPara(base_ptr);
}

bool LmkLocProcessor::processScan(sros::core::base_msg_ptr base_ptr) {

    slam::tf::TransForm fusion_tf;
    if (tf_base_to_odo->lookUpTransForm(base_ptr->time_, fusion_tf, 50000)) {
        sros::core::Pose base_pose(fusion_tf.position, fusion_tf.rotation);
        //LOG(INFO) << "Get initial base pose： " << base_pose.x() << ", " << base_pose.y() << ", " << base_pose.yaw();
        processScan(base_ptr, base_pose);
        return true;
    } else {
        if (!checkScanState(base_ptr)) {
            return false;
        }
        convertToScan(base_ptr);//获取初始位姿时,需要用到该方法
        if (!is_location_state || !lmk_module_ptr) {
            return true;
        }
        if (!last_pose_msg || !sensor_msg) {
            LOG(INFO) << "err! cannot get the pose and the sensor!";
            return true;
        }
        sensor_msg->odo = last_pose_msg;//scan中心在世界坐标系的位姿
        last_scan = base_ptr;
        if (lmk_module_ptr->processLoc(sensor_msg)) {
            return true;
        } else {
            LOG(INFO) << "err to get enough lmk!";
            changeStateMsg(sros::core::STATE_SLAM_ERROR);
            usleep(10000);
            processStopLoc();
            return false;
        }
    }

}

bool LmkLocProcessor::processScan(sros::core::base_msg_ptr base_ptr, sros::core::Pose base_pose) {

    if (!checkScanState(base_ptr)) {
        return false;
    }
    convertToScan(base_ptr);//获取初始位姿时,需要用到该方法
    if (!is_location_state || !lmk_module_ptr) {
        return true;
    }
    if (!last_pose_msg || !sensor_msg) {
        LOG(INFO) << "err! cannot get the pose and the sensor!";
        return true;
    }
    sensor_msg->odo = last_pose_msg;//scan中心在世界坐标系的位姿

    //需要先得到运动中心在世界坐标系的位姿，然后依据deltaodo，求出运动中心在世界坐标系的初始位姿，最后求出scan中心在世界坐标系的初始位姿
    Eigen::Affine2f base_tf(
            Eigen::Translation2f(base_pose.x(), base_pose.y()) * Eigen::Rotation2Df(base_pose.yaw()));
    Eigen::Vector2f scan_point = base_tf * odo2scan_pose.head<2>();
    Eigen::Vector3f nav_pose;
    nav_pose.x() = scan_point.x();
    nav_pose.y() = scan_point.y();
    nav_pose.z() = base_pose.yaw() + odo2scan_pose.z();
    if (std::hypot(last_pose_msg->pose.x() - nav_pose.x(),
                   last_pose_msg->pose.y() - nav_pose.y()) < 0.5) {
        sensor_msg->odo->pose = nav_pose;
    }

    last_scan = base_ptr;
    if (lmk_module_ptr->processLoc(sensor_msg)) {
        return true;
    } else {
        LOG(INFO) << "err to get enough lmk!";
        changeStateMsg(sros::core::STATE_SLAM_ERROR);
        usleep(10000);
        processStopLoc();
        return false;
    }
}

bool LmkLocProcessor::loadMap(std::string map_name) {
    return LocationProcessor::loadMap(map_name);
}

void LmkLocProcessor::processInitialPoseThread(sros::core::SlamCommandMsg &syscommand) {
    map_path = syscommand.map_path;
    map_name = syscommand.map_name;

    std::string map_name_with_path = map_path + map_name;

    LOG(INFO) << "the map name is:" << map_name_with_path;
    lmk_module_ptr->loadMap(map_name_with_path);//防止在绘图过程中,不断加载地图,为了保险起见,重复加载地图

    auto &sros_pose = syscommand.pose;

    Eigen::Vector3f center_pose(sros_pose.x(), sros_pose.y(), sros_pose.yaw());
    Eigen::Vector3f base_to_scan_pose = getBaseToScanPose();
    Eigen::Affine2f center_tf(Eigen::Translation2f(center_pose[0], center_pose[1]) *
                              Eigen::Rotation2Df(center_pose[2]));

    Eigen::Vector2f initial_point = center_tf * base_to_scan_pose.head<2>();
    Eigen::Vector3f initial_pose(initial_point[0], initial_point[1], center_pose[2] + base_to_scan_pose[2]);
    slam::PoseMsg_Ptr pose_msg(new slam::PoseMsg);
    pose_msg->pose = initial_pose;
    if (processInitialPose(pose_msg)) {
//        allow_debug_info = true;
//        boost::thread(boost::bind(&LmkLocProcessor::monitorLoop, this));
    }
}

bool LmkLocProcessor::processInitialPose(std::shared_ptr<slam::PoseMsg> pose_msg) {
    bool use_pose_record = (bool) sros::core::Settings::getInstance().getValue<int>(
            "posefilter.use_pose_record", 0);
    if (use_pose_record) {
        std::stringstream record_name;
        record_name << "/sros/r2000_record";
        record_name << sros::core::util::get_time_in_ms();
        record_name << ".record";
        pose_record.reset(new record::PoseRecord(record_name.str()));
    }

    int count = 0;
    is_location_state = false;
    using namespace sros::core;
    while (count < 10) {
        if (sensor_msg) {
            auto scan_msg = sensor_msg;
            scan_msg->odo = pose_msg;
            if (lmk_module_ptr->initialPoseLoc(scan_msg, last_pose_msg)) {
                lmk_module_ptr->handleStartLocation();
                LOG(INFO) << "succ to get the initial pose! will go into the location state!";

                is_location_state = true;

                changeStateMsg(STATE_SLAM_LOCATING);
                return true;
            } else {
                LOG(INFO) << "err to locate! will return! please input the initial pose again!";
                changeStateMsg(STATE_SLAM_ERROR);
                usleep(10000);
                changeStateMsg(STATE_SLAM_IDLE);
                return false;
            }
        }
        count++;
        usleep(500000);
    }

    changeStateMsg(STATE_SCAN_NOT_PUBLISHED);
    usleep(10000);
    changeStateMsg(STATE_SLAM_IDLE);
    LOG(INFO) << "cannot get the scan data! will return";
    return false;
}

bool LmkLocProcessor::processStopLoc() {
    using namespace sros::core;
    is_location_state = false;
    last_pose_msg.reset();
    lmk_module_ptr->handleStopLocation();

    changeStateMsg(STATE_SLAM_IDLE);
    return true;
}

void LmkLocProcessor::convertToScan(sros::core::base_msg_ptr base_ptr) {

//    debug::RecordFunc::Record(name_to_str(convertToScan));

    sros::core::LaserScan_ptr scan_ptr = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);

    sensor_msg.reset(new slam::SensorInputMsg(scan_ptr->time_));

    sensor_msg->stamp = scan_ptr->time_;
    auto &scan_msg = sensor_msg->scan;
    scan_msg->angle_min = scan_ptr->angle_min;
    scan_msg->angle_max = scan_ptr->angle_max;
    scan_msg->angle_increment = scan_ptr->angle_increment;
    scan_msg->range_min = scan_ptr->range_min;
    scan_msg->range_max = scan_ptr->range_max;
    scan_msg->time_increment = scan_ptr->time_increment;
    scan_msg->stamp = scan_ptr->time_;
    scan_msg->intensities = scan_ptr->intensities;
    scan_msg->ranges = scan_ptr->ranges;
}

void LmkLocProcessor::sendMatchPose(std::shared_ptr<slam::PoseMsg> pose_msg) {
    sros::core::PoseStamped_ptr pose_msg_ptr(new sros::core::PoseStampedMsg("TOPIC_MATCHPOSE"));

    Eigen::Vector3f &nav_pose = pose_msg->pose;
    Eigen::Affine2f scan_tf(
            Eigen::Translation2f(nav_pose[0], nav_pose[1]) * Eigen::Rotation2Df(nav_pose[2]));
    Eigen::Vector2f center_point = scan_tf * scan2odo_pose.head<2>();

    Eigen::Vector3f center_pose;
    center_pose[0] = center_point[0];
    center_pose[1] = center_point[1];
    center_pose[2] = nav_pose[2] + scan2odo_pose[2];

    pose_msg_ptr->pose.x() = center_pose[0];
    pose_msg_ptr->pose.y() = center_pose[1];
    pose_msg_ptr->pose.yaw() = center_pose[2];
    pose_msg_ptr->time_ = pose_msg->stamp;

    if (pose_record) {
        record::RecordPoseInfo_Ptr pose(new record::RecordPoseInfo);
        pose->index = pose_msg->stamp;
        pose->cov = Eigen::Matrix3f::Zero();
        pose->matpose = nav_pose;
        pose->optpose = center_pose;
//        pose->numlmks = current_pose.flector_no;
//        pose->nummatlmk = current_pose.flector_no;
        pose_record->writePoseData(pose);
    }
    if (sendMsg) {
        sendMsg(pose_msg_ptr);
    }
}

void LmkLocProcessor::getScanToOdoTF() {
    odo2scan_pose[0] = sros::core::Settings::getInstance().getValue<float>(//getValue = mainSetting.ini中参数
            "posefilter.laser_coordx", 0.1);
    odo2scan_pose[1] = sros::core::Settings::getInstance().getValue<float>(//getValue = mainSetting.ini中参数
            "posefilter.laser_coordy", 0.0);
    odo2scan_pose[2] = sros::core::Settings::getInstance().getValue<float>(//getValue = mainSetting.ini中参数
            "posefilter.laser_coordyaw", 0.0);

    std::string location_type_str = sros::core::Settings::getInstance().getValue<std::string>(
            "posefilter.location_type", "NAV_350");

    Eigen::Affine2f odo2scan_tf(
            Eigen::Translation2f(odo2scan_pose[0], odo2scan_pose[1]) * Eigen::Rotation2Df(odo2scan_pose[2]));
    Eigen::Vector2f scan2odo_point = odo2scan_tf.inverse() * Eigen::Vector2f(0.0, 0.0);
    scan2odo_pose[0] = scan2odo_point[0];
    scan2odo_pose[1] = scan2odo_point[1];
    scan2odo_pose[2] = -odo2scan_pose[2];

}

void LmkLocProcessor::sendObstacleMsg(sros::core::base_msg_ptr base_ptr,
                                      std::shared_ptr<slam::PoseMsg> pose_msg) {

    sros::core::LaserScan_ptr scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);
    auto &ranges = scan->ranges;
    int size = ranges.size();
    float angle = scan->angle_min;
    float increment = scan->angle_increment;
    float maxrange = scan->range_max;
    float minrange = scan->range_min;

    sros::core::ObstacleMsg_ptr obstacle(new sros::core::ObstacleMsg("TOPIC_OBSTACLE"));

    auto &location_vec = obstacle->point_cloud;

    Eigen::Affine2f scan_to_world_tf(Eigen::Translation2f(pose_msg->pose[0], pose_msg->pose[1]) *
                                     Eigen::Rotation2Df(pose_msg->pose[2]));
    for (int i = 0; i < size; ++i) {
        float dist = scan->ranges[i];
        if ((dist > minrange) && (dist < maxrange)) {
            if (i % 50 == 0) {
                Eigen::Vector2f tmp_point_2d(dist * cos(angle), dist * sin(angle));
                Eigen::Vector2f world_point_2d = scan_to_world_tf * tmp_point_2d;
                sros::core::Location obstacle_point;
                obstacle_point.x() = world_point_2d.x();
                obstacle_point.y() = world_point_2d.y();
                location_vec.push_back(obstacle_point);
            }
        }
        angle += increment;
    }
    sendMsg(obstacle);
}

Eigen::Vector3f LmkLocProcessor::getBaseToScanPose() {
    Eigen::Vector3f base_to_scan;
    base_to_scan[2] = -scan2odo_pose[2];

    Eigen::Affine2f scan_to_base_tf(Eigen::Translation2f(scan2odo_pose[0], scan2odo_pose[1])
                                    * Eigen::Rotation2Df(scan2odo_pose[2]));

    auto tmp_point = scan_to_base_tf.inverse() * Eigen::Vector2f::Zero();
    base_to_scan[0] = tmp_point[0];
    base_to_scan[1] = tmp_point[1];
    return base_to_scan;
}


LmkLocProcessor::LmkLocProcessor() : LocationProcessor(LMK_LOC_TYPE), is_location_state(false) {
    slam::tf::FrameToFrame base_to_odo_frame;
    base_to_odo_frame.parent_frame = "odom";
    base_to_odo_frame.child_frame = "base_link";
    tf_base_to_odo.reset(new slam::tf::TFOperator(base_to_odo_frame));
    slam::tf::FrameToFrame base_to_world_frame;
    base_to_world_frame.parent_frame = "world";
    base_to_world_frame.child_frame = "base_link";
    tf_base_to_world.reset(new slam::tf::TFOperator(base_to_world_frame));

    getScanToOdoTF();

    lmkslam_para.reset(new slam::LmkParaMsg);
    std::string lmk_type = sros::core::Settings::getInstance().getValue<std::string>(//getValue = usersetting.ini中参数
            "slam.lmk_extract_type", "FLAT");
    if (lmk_type == "FLAT") {
        lmkslam_para->extract_type = slam::LmkParaMsg::FLAT_LMK;
    } else if (lmk_type == "CYLINDER") {
        lmkslam_para->extract_type = slam::LmkParaMsg::CYLINDER_LMK;
    } else {
        LOG(INFO) << "err to extract the type!" << lmk_type;
        LOG(INFO) << "please input FLAT or CYLINDER and restart!";
    }
    lmk_module_ptr = slam::LmkModuleSingleton::getInstance(lmkslam_para);
    lmk_module_ptr->setLocOutputCallback(boost::bind(&LmkLocProcessor::mapOutputCallback, this, _1));
}

void LmkLocProcessor::monitorLoop() {
//    sleep(1);
    while (is_location_state) {
        boost::unique_lock<boost::mutex> lock(wakeup_mutex);
        boost::xtime xt;
#if BOOST_VERSION >= 105000
        boost::xtime_get(&xt, boost::TIME_UTC_);
#else
        boost::xtime_get(&xt, boost::TIME_UTC);
#endif
        xt.nsec += 2e8;
        if (condition.timed_wait(lock, xt)) {
            int64_t now_time = sros::core::util::get_time_in_us();
//            if(now_time-last_scan_time_stamp>1e6) {
//                LOG(INFO) << "the lidar delay too long!";
//                abort();
//            }
            continue;
        } else {
            if (is_location_state) {
                changeStateMsg(sros::core::STATE_SLAM_ERROR);
                usleep(10000);
                changeStateMsg(sros::core::STATE_SLAM_IDLE);
                LOG(INFO) << "cannot return! will abort!";
                LOG(INFO) << "the now time is：" << sros::core::util::get_time_in_us();
                LOG(INFO) << "the scan time is:" << last_scan_time_stamp;
                int64_t now_time = sros::core::util::get_time_in_us();
                if (now_time - last_scan_time_stamp > 2e5) {
                    LOG(INFO) << "the lidar publish delay too long!";
                }
                abort();
            }
        }
    }

}

void LmkLocProcessor::wakeupInfo() {
    condition.notify_all();
}

void LmkLocProcessor::mapOutputCallback(std::shared_ptr<slam::MapOutputMsg> match_output) {

    if (match_output) {
        switch (match_output->successful_match) {
            case slam::MapOutputMsg::NORM_MATCH_STATE:
                last_pose_msg = match_output->output_pose;
                if (last_pose_msg) {
                    sendMatchPose(last_pose_msg);
                    sendObstacleMsg(last_scan, last_pose_msg);
                }
                err_loc_count = 0;
                break;
            case slam::MapOutputMsg::WARN_MATCH_STATE:
                err_loc_count++;
                if (err_loc_count > err_loc_count_thresh) {
                    err_loc_count = 0;
                    LOG(INFO) << "err to locate! will return false";
                    changeStateMsg(sros::core::STATE_SLAM_ERROR);
                    usleep(10000);
                    processStopLoc();
                }
                break;
            case slam::MapOutputMsg::ERR_MATCH_STATE:
                processStopLoc();
                LOG(INFO) << "will stop map!";
                break;
            case slam::MapOutputMsg::FALT_MATCH_STATE:
                LOG(INFO) << "will abort the prog!";
                abort();
                break;
            default:
                LOG(INFO) << "err to get the match state!" << match_output->successful_match;
                break;
        }
    } else {
        LOG(INFO) << "err to get the map_putput!";
    }
}

bool LmkLocProcessor::checkScanState(sros::core::base_msg_ptr base_ptr) {
    sros::core::LaserScan_ptr scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);
    if (!scan) {
        LOG(INFO) << "cannot get the scan! will return false";
        return false;
    }
    int com_point_size = floorf((scan->angle_max - scan->angle_min) / scan->angle_increment + 0.5 + 1.0);
    if (com_point_size == scan->ranges.size()) {
        return true;
    } else {
        LOG(INFO) << "the scan size is wrong!" << scan->ranges.size();
        return false;
    }
}
}
