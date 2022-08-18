//
// Created by lfc on 18-12-13.
//

#ifndef SROS_LASER_SENSOR_MANAGER_HPP
#define SROS_LASER_SENSOR_MANAGER_HPP

#include "sensor_process_alg.h"
#include "base_sensor_manager.hpp"
#include <glog/logging.h>

namespace sensor{
class LaserSensorManager :public BaseSensorManager{
public:
    LaserSensorManager(oba::ObstacleModulePara_Ptr para,ObaMsgCallback msg):BaseSensorManager(para,msg,TYPE_LASER_SENSOR) {
        scan_tf = Eigen::Translation2d(para->laser_coord_x, para->laser_coord_y) *
                  Eigen::Rotation2Dd(para->laser_coord_yaw);
        oba_name = sros::device::DEVICE_LIDAR;
    }

    virtual void processMsg(sros::core::base_msg_ptr& m){
        sros::core::LaserScan_ptr scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(m);
        sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
        oba_msg->oba_name = scan->sensor_name;
        oba_msg->time_ = m->time_;

        if (enable_publish_obstacle_) {
            sros::core::LaserScan_ptr scan_msg(new sros::core::LaserScanMsg);
            *scan_msg = *scan;
            if (para->enable_filter_low_intensity_points) {
                sensor::filterLowIntenPoints<sros::core::LaserScan_ptr>(scan_msg, para->min_filter_length);
            }
            //
            sensor::filterScanTrailPoint<sros::core::LaserScan_ptr>(scan_msg, para->min_filter_length);

            filterScanIsolatedPoint(scan_msg);
            slam::tf::TransForm curr_pose;
            if (!base_to_world_tf->lookUpTransForm(scan_msg->time_, curr_pose, para->delta_time_thresh)) {
                //LOG(INFO) << "oba err to get the realtime msg!";
                return;
            }

            if (std::isnan(curr_pose.rotation.yaw() || !finite(curr_pose.rotation.yaw()))) {
                LOG(WARNING) << "yaw is nan value!" << curr_pose.rotation.yaw();
                return;
            }
            if (std::isnan(curr_pose.position.x() || !finite(curr_pose.position.x()))) {
                LOG(WARNING) << "x is nan value!" << curr_pose.position.x();
                return;
            }
            if (std::isnan(curr_pose.position.y() || !finite(curr_pose.position.y()))) {
                LOG(WARNING) << "y is nan value!" << curr_pose.position.y();
                return;
            }

            Eigen::Affine2d world_tf = Eigen::Translation2d(curr_pose.position.x(), curr_pose.position.y()) *
                                       Eigen::Rotation2Dd(curr_pose.rotation.yaw());
            auto scan_to_world_tf = world_tf * scan_tf;
            if (scan_msg->second_sensor_name.empty()) {
                convertToObas(scan_to_world_tf, scan_msg, oba_msg);
            }else{
                convertLidarToObasWithAngle(scan_to_world_tf, scan_msg, oba_msg, scan_msg->first_angle_min,
                                            scan_msg->first_angle_max);
                sros::core::ObstacleMsg_ptr second_oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
                second_oba_msg->oba_name = scan->second_sensor_name;
                second_oba_msg->time_ = m->time_;
                convertDualLidarToObas(scan_to_world_tf, scan_msg, oba_msg, second_oba_msg);
                if (sendObaMsg) {
                    sendObaMsg(second_oba_msg);
                }
            }
        }

        if (sendObaMsg) {
            sendObaMsg(oba_msg);
        }else {
            LOG(INFO) << "have not set the sendMsgCallback! cannot transform the obas!";
        }
    }

    virtual void setObaName(std::string name) {
        oba_name = name;
    }

    void convertToObas(Eigen::Affine2d& world_tf,const sros::core::LaserScan_ptr &scan_msg,sros::core::ObstacleMsg_ptr& oba_msg){
        double start_angle = scan_msg->angle_min;
        double angle_incre = scan_msg->angle_increment;
        oba_msg->point_cloud.reserve(scan_msg->ranges.size());
        double max_range_min =
                scan_msg->range_min > para->oba_laser_range_min ? scan_msg->range_min : para->oba_laser_range_min;
        double min_range_max =
                scan_msg->range_max < para->oba_laser_range_max ? scan_msg->range_max : para->oba_laser_range_max;

        for (auto &range:scan_msg->ranges) {
            if (start_angle > para->laser_angle_min && start_angle < para->laser_angle_max) {
                if (range < min_range_max && range > max_range_min) {
                    Eigen::Vector2d curr_point(range * cos(start_angle), range * sin(start_angle));
                    auto world_point = world_tf * curr_point;
                    sros::core::Location loc(world_point[0], world_point[1]);
                    oba_msg->point_cloud.push_back(loc);
                }
            }
            start_angle += angle_incre;
        }
    }

    void convertLidarToObasWithAngle(Eigen::Affine2d& world_tf,const sros::core::LaserScan_ptr &scan_msg,sros::core::ObstacleMsg_ptr& oba_msg,const float angle_min,const float angle_max){
        double start_angle = scan_msg->angle_min;
        double angle_incre = scan_msg->angle_increment;
        int start_index = roundf((angle_min - start_angle) / angle_incre);
        int end_index = roundf((angle_max - start_angle) / angle_incre);
        oba_msg->point_cloud.reserve(scan_msg->ranges.size());
        double max_range_min =
                scan_msg->range_min > para->oba_laser_range_min ? scan_msg->range_min : para->oba_laser_range_min;
        double min_range_max =
                scan_msg->range_max < para->oba_laser_range_max ? scan_msg->range_max : para->oba_laser_range_max;
        auto& ranges = scan_msg->ranges;
        if (end_index >= ranges.size()) {
            LOG(INFO) << "end index is wrong!" << end_index << "," << scan_msg->angle_min << "," << angle_max;
            end_index = ranges.size();
        }
        if (start_index < 0) {
            LOG(INFO) << "start index is wrong!" << start_index << "," << scan_msg->angle_min << "," << angle_min;
            start_index = 0;
        }
        for (int i = start_index; i < end_index; ++i) {
            float angle = (float)i * angle_incre + start_angle;
            auto& range = ranges[i];
            if (angle > para->laser_angle_min && angle < para->laser_angle_max) {
                if (range < min_range_max && range > max_range_min) {
                    Eigen::Vector2d curr_point(range * cos(angle), range * sin(angle));
                    auto world_point = world_tf * curr_point;
                    sros::core::Location loc(world_point[0], world_point[1]);
                    oba_msg->point_cloud.push_back(loc);
                }
            }
        }
    }

    void convertDualLidarToObas(Eigen::Affine2d& world_tf,const sros::core::LaserScan_ptr &scan_msg,sros::core::ObstacleMsg_ptr& first_msg,sros::core::ObstacleMsg_ptr& second_msg){
        double angle = scan_msg->angle_min;
        double angle_incre = scan_msg->angle_increment;
        auto& ranges = scan_msg->ranges;
        int index = 0;
        double max_range_min =
            scan_msg->range_min > para->oba_laser_range_min ? scan_msg->range_min : para->oba_laser_range_min;
        double min_range_max =
            scan_msg->range_max < para->oba_laser_range_max ? scan_msg->range_max : para->oba_laser_range_max;
        while (angle <= scan_msg->angle_max) {
            int index = (int) round((angle - scan_msg->angle_min) / angle_incre);
            auto& range = ranges[index];
            if (range < min_range_max && range > max_range_min) {
                Eigen::Vector2d curr_point(range * cos(angle), range * sin(angle));
                auto world_point = world_tf * curr_point;
                sros::core::Location loc(world_point[0], world_point[1]);
                if (angle >= scan_msg->first_angle_min && angle < scan_msg->first_angle_max) {
                    first_msg->point_cloud.push_back(loc);
                }else{
                    second_msg->point_cloud.push_back(loc);
                }
            }
            angle += angle_incre;
            if (index++ > 10000) {
                LOG(INFO) << "enough size points! will break";
                break;
            }
        }
    }
private:
    Eigen::Affine2d scan_tf;
    std::string oba_name;
};


}


#endif //SROS_LASER_SENSOR_MANAGER_HPP
