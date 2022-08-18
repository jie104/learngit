//
// Created by lfc on 17-7-12.
//

#include <core/settings.h>
#include <modules/slam/include/lmkslam/lmk_module_singleton.h>
#include <core/msg/PoseStampedMsg.h>
#include <core/msg/laser_scan_msg.hpp>
#include <execinfo.h>
#include <signal.h>
#include "lmk_map_processor.h"
#include "../include/lmkslam/debug_tool/RecordFunc.hpp"
#include "../include/feature_extractor/feature_function_factory.h"

namespace mapping {
LmkMapProcessor::LmkMapProcessor() : MappingProcessor(LMK_MAP_TYPE), is_mapping_state(false) {
    map_resolution = 0.02;
    map_level = 3;
    updatemap_dist_thresh = 0.1;
    updatemap_yaw_thresh = 0.107;

    laser_min_dist = sros::core::Settings::getInstance().getValue<float>(//getValue = mainSetting.ini中参数
            "slam.map_laser_min_dist", 0.1);
    laser_max_dist = sros::core::Settings::getInstance().getValue<float>(//getValue = mainSetting.ini中参数
            "slam.map_laser_max_dist", 30.0);


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
    getScanToOdoTF();
    lmk_module_ptr = slam::LmkModuleSingleton::getInstance(lmkslam_para);
    lmk_module_ptr->setMapOutputCallback(boost::bind(&LmkMapProcessor::mapOutputCallback, this, _1));
}

bool LmkMapProcessor::handleStartMapping(sros::core::SlamCommandMsg &syscommand) {
    LOG(INFO) << "start mapping!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    map_name = syscommand.map_name;
    map_path = syscommand.map_path;
    std::string map_path_with_name = map_path + map_name;
    boost::thread(boost::bind(&LmkMapProcessor::startMapThread, this, map_path_with_name));
    return true;
}

bool LmkMapProcessor::handleStopMapping(sros::core::SlamCommandMsg &syscommand) {
    std::string map_path_with_name = map_path + map_name;
    boost::thread(boost::bind(&LmkMapProcessor::stopMapThread, this, map_path_with_name));
    return true;
}

bool LmkMapProcessor::handleCancelMapping(sros::core::SlamCommandMsg &syscommand) {
    is_mapping_state = false;
    lmk_module_ptr->clearMap();
    lmk_module_ptr.reset();
    return true;
}

bool LmkMapProcessor::processScan(sros::core::base_msg_ptr base_ptr) {

    if (!checkScanState(base_ptr)) {
        return false;
    }

    convertToScan(base_ptr);
    if (!is_mapping_state || !lmk_module_ptr) {
        return true;
    }
    if (!sensor_msg) {
        LOG(INFO) << "err! cannot get the pose and the sensor!";
        return true;
    }
    if (last_pose_msg) {
        sensor_msg->odo = last_pose_msg;
    } else {
        sensor_msg->odo.reset(new slam::PoseMsg);
        sensor_msg->odo->pose = Eigen::Vector3f::Zero();
        sensor_msg->odo->pose.z() = -slam::FeatureFunctionFactory::getScanDirection(sensor_msg->scan);
    }
    if (lmk_module_ptr->processMap(sensor_msg)) {
        return true;
    } else {
        LOG(INFO) << "err to process map! maybe the thread have not creat!";
    }
    LOG(INFO) << "err to extract enough lmk! will return!";
    std::string map_name_with_path = map_path + map_name;
    boost::thread(boost::bind(&LmkMapProcessor::stopMapThread, this, map_name_with_path));

    return false;
}

bool LmkMapProcessor::processPara(sros::core::base_msg_ptr base_ptr) {
    return true;
}

void LmkMapProcessor::saveMap(std::string map_name) {
    lmk_module_ptr->handleStopMapping(map_name);
    saveOccMap(map_name);
}

void LmkMapProcessor::getScanToOdoTF() {
    Eigen::Vector3f odo2scan_pose;
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


bool LmkMapProcessor::startMapThread(std::string map_name_with_path) {
    is_mapping_state = false;
    bool state_record = true;
    if (!lmk_module_ptr) {
        lmk_module_ptr = slam::LmkModuleSingleton::getInstance(lmkslam_para);
    }

    if (lmk_module_ptr->haveSameMap(map_name_with_path)) {//继续绘图
        auto last_pose = lmk_module_ptr->getLastPose();
        if (sensor_msg) {
//            boost::mutex::scoped_lock lock(thread_mutex);
            sensor_msg->odo = last_pose;
            auto scan_msg = sensor_msg;
            if (lmk_module_ptr->processLoc(scan_msg, last_pose_msg)) {

                lmk_module_ptr->continueMapping(map_name_with_path);//用来初始化系统

                sendMatchPose(last_pose_msg);
                state_record = true;
            } else {
                state_record = false;
            }
        } else {
            LOG(INFO) << "the msg have not published! cannot mapping!";
        }

    } else {
        last_pose_msg.reset();
        lmk_module_ptr->clearMap();
        state_record = true;
    }
    if (state_record) {
        lmk_module_ptr->handleStartMapping(map_name_with_path);
        is_mapping_state = true;
        changeStateMsg(sros::core::STATE_SLAM_DRAWING);
        LOG(INFO) << "have start darwing!";
        return true;
    } else {
        changeStateMsg(sros::core::STATE_SLAM_IDLE);
        return false;
    }

}

void LmkMapProcessor::sendMatchPose(std::shared_ptr<slam::PoseMsg> pose_msg) {
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
    if (sendMsg) {
        sendMsg(pose_msg_ptr);
    }
}

bool LmkMapProcessor::stopMapThread(std::string map_name_with_path) {
    boost::mutex::scoped_lock lock(stop_mapping_mutex);

    is_mapping_state = false;
    changeStateMsg(sros::core::STATE_SLAM_SAVING_MAP);
    saveMap(map_name_with_path);
    changeStateMsg(sros::core::STATE_SLAM_IDLE);
    lmk_module_ptr.reset();
    map_officer.reset();
    return true;
}

void LmkMapProcessor::convertToScan(sros::core::base_msg_ptr base_ptr) {

    sros::core::LaserScan_ptr scan_ptr = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);

//    boost::mutex::scoped_lock lock(thread_mutex);
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

void LmkMapProcessor::updateOccMap(std::shared_ptr<slam::PoseMsg> pose_msg,
                                   sros::core::base_msg_ptr base_ptr) {
    if (pose_msg->id == -1) {
        return;
    }
//    bool need_update_map = true;
////    if (last_updatemap_pose) {
////        auto last_pose = last_updatemap_pose->pose;
////        auto curr_pose = pose_msg->pose;
////
////        float delta_dist = (curr_pose-last_pose).head<2>().norm();
////        float delta_yaw = curr_pose[2] - last_pose[2];
////
////        if (delta_dist > updatemap_dist_thresh) {
////            need_update_map = true;
////        }
////        if (delta_yaw > updatemap_yaw_thresh) {
////            need_update_map = true;
////        }
////    }else {
////        need_update_map = true;
////    }
//    if (need_update_map) {
////        last_updatemap_pose = pose_msg;
////        sros::core::LaserScan_ptr scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);
////        mapping::PointContainer_Ptr points(new mapping::PointContainer);
////        points->origin = pose_msg->pose;
////        convertScantoPoints(scan, points);
////        if (map_officer) {
////            map_officer->updateMap(points);
////        }else {
////            LOG(INFO) << "err to get the map officer!";
////        }
//        sros::core::LaserScan_ptr scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(base_ptr);
//        SrosMapInfo map_info;
//        map_info.scan = scan;
//        map_info.pose = pose_msg;
//        map_info_list.push_back(map_info);

//    }

}

void LmkMapProcessor::convertScantoPoints(slam::ScanMsg_Ptr scan_ptr,
                                          mapping::PointContainer_Ptr points) {
    if (scan_ptr) {
        int size = scan_ptr->ranges.size();
        float angle = scan_ptr->angle_min;
        float increment = scan_ptr->angle_increment;

        auto &laser_cloud = points->points;

        laser_cloud.resize(size);

        float maxrange = scan_ptr->range_max - 0.01f;
        float minrange = scan_ptr->range_min;
        int count = 0;
        float dist_min = minrange > laser_min_dist ? minrange : laser_min_dist;
        float dist_max = maxrange < laser_max_dist ? maxrange : laser_max_dist;
        for (int i = 0; i < size; ++i) {
            float dist = scan_ptr->ranges[i];
            if ((dist > dist_min) && (dist < dist_max)) {
                laser_cloud[count].x = (cos(angle) * dist);
                laser_cloud[count].y = (sin(angle) * dist); //where we need to check the orientation of the deltayaw
                count++;
            }
            angle += increment;
        }
        laser_cloud.resize(count);
    } else {
        LOG(INFO) << "error to get the scan!";
    }
}


void LmkMapProcessor::saveOccMap(std::string map_name) {
    auto scan_list = lmk_module_ptr->getScanList();
    mapping::PointContainer_Ptr points(new mapping::PointContainer);

    map_officer.reset(new mapping::GridMapOfficer(map_resolution, map_level));

    int update_free = 0;

    for (auto &map_info:scan_list) {
        points->origin = map_info.pose->pose;
        convertScantoPoints(map_info.scan, points);
        if (map_officer) {
            if (update_free++ % update_free_step == 0) {
                map_officer->updateMap(points);
            } else {
                map_officer->updateMap(points, false);
            }

        } else {
            LOG(INFO) << "err to get the map officer!";
        }
    }
    map_officer->saveMap(map_name);
}

void LmkMapProcessor::mapOutputCallback(slam::MapOutputMsg_Ptr map_output) {
    if (map_output) {
        switch (map_output->successful_match) {
            case slam::MapOutputMsg::NORM_MATCH_STATE:
                last_pose_msg = map_output->output_pose;
                sendMatchPose(last_pose_msg);
                break;
            case slam::MapOutputMsg::WARN_MATCH_STATE:
            case slam::MapOutputMsg::ERR_MATCH_STATE: {
                std::string map_name_with_path = map_path + map_name;
                boost::thread(boost::bind(&LmkMapProcessor::stopMapThread, this, map_name_with_path));
                LOG(INFO) << "will stop map!";
                break;
            }
            case slam::MapOutputMsg::FALT_MATCH_STATE:
                LOG(INFO) << "will abort the prog!";
                abort();
                break;
            default:
                LOG(INFO) << "err to get the match state!" << map_output->successful_match;
                break;
        }
    } else {
        LOG(INFO) << "err to get the map_putput!";
    }

}

bool LmkMapProcessor::checkScanState(sros::core::base_msg_ptr base_ptr) {
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
