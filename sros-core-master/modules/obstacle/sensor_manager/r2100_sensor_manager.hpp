//
// Created by lfc on 18-12-19.
//

#ifndef SROS_R2100_SENSOR_MANAGER_HPP
#define SROS_R2100_SENSOR_MANAGER_HPP
#include "base_sensor_manager.hpp"
#include "core/msg/sonar_data_msg.hpp"
#include <glog/logging.h>
#include "../rack_oba_filter_singleton.h"

namespace sensor{
class R2100SensorManager:public BaseSensorManager {
public:
    R2100SensorManager(oba::ObstacleModulePara_Ptr para,ObaMsgCallback msg):BaseSensorManager(para,msg,TYPE_R2100_SENSOR) {
        r2100_tf = Eigen::Translation2d(para->r2100_coord_x, para->r2100_coord_y) *
                  Eigen::Rotation2Dd(para->r2100_coord_yaw);
        oba_name = sros::device::DEVICE_R2100;
    }

    virtual void processMsg(sros::core::base_msg_ptr& m){
        sros::core::DistanceDataMsg_ptr r2100 = std::dynamic_pointer_cast<sros::core::DistanceDataMsg>(m);

        slam::tf::TransForm curr_pose;
        if(!base_to_world_tf->lookUpTransForm(r2100->time_, curr_pose,para->delta_time_thresh)){
            //LOG(INFO)<<"oba err to get the realtime msg!";
            return;
        }

        if(std::isnan(curr_pose.rotation.yaw()||!finite(curr_pose.rotation.yaw()))) {
            LOG(WARNING) << "yaw is nan value!" << curr_pose.rotation.yaw();
            return;
        }
        if(std::isnan(curr_pose.position.x()||!finite(curr_pose.position.x()))) {
            LOG(WARNING) << "x is nan value!" << curr_pose.position.x();
            return;
        }
        if(std::isnan(curr_pose.position.y()||!finite(curr_pose.position.y()))) {
            LOG(WARNING) << "y is nan value!" << curr_pose.position.y();
            return;
        }

        Eigen::Affine2d world_tf = Eigen::Translation2d(curr_pose.position.x(), curr_pose.position.y()) *
                                   Eigen::Rotation2Dd(curr_pose.rotation.yaw());
        auto scan_to_world_tf = world_tf * r2100_tf;
        sros::core::ObstacleMsg_ptr oba_msg(new sros::core::ObstacleMsg("OBSTACLES"));
        //sros::core::ObstacleMsg_ptr oba_topic(new sros::core::ObstacleMsg("TOPIC_OBSTACLE"));
        oba_msg->oba_name = r2100->sensor_name;
        oba_msg->time_ = r2100->time_;
        bool use_oba = true;
        if (para->is_load_full_state && !para->enable_r2100_points_when_load_full) {
            use_oba = false;
        }
        if (use_oba) {
            convertToObas(scan_to_world_tf, r2100, oba_msg);
        }

        if (sendObaMsg) {
            sendObaMsg(oba_msg);
            //*oba_topic = *oba_msg;
            //oba_topic->topic_ = "TOPIC_OBSTACLE";
            //if (oba_topic->point_cloud.size()) {
            //    sendObaMsg(oba_topic);
            //}
        }else {
            LOG(INFO) << "have not set the sendMsgCallback! cannot transform the obas!";
        }
    }

    virtual void setObaName(std::string name) {
        oba_name = name;
    }

    void convertToObas(Eigen::Affine2d& world_tf,const sros::core::DistanceDataMsg_ptr &msg,sros::core::ObstacleMsg_ptr& oba_msg){
        const double DEGREE_TO_RAD = M_PI / 180.0;
        // according to OMD8000-R2100 datasheet
        // https://www.pepperl-fuchs.com/usa/en/classid_53.htm?view=productdetails&prodid=62235
        double laser_angle_start = -44 * DEGREE_TO_RAD; // 以x轴正方向作为正方向，角度范围-44° - 44°
        double laser_angle_step = 8 * DEGREE_TO_RAD;

        int angle_index = 0;

        sros::core::Location_Vector local_points; // 在设备局部坐标系下的障碍点坐标
        sros::core::Location_Vector ob_points; // 在世界坐标系中的坐标，可用于避障

//    string debug_str;

        // 将距离信息转换为传感器坐标系下的坐标点
        for (auto d : msg->distances) {
            if (d >= 0xffffffff) { // 检测到距离无穷远
//            debug_str += "NA, ";
                angle_index += 1;
                continue;
            }
//        debug_str += to_string(d) + ",";

            double theta1 = laser_angle_start + laser_angle_step * angle_index;
            double theta2 = theta1 + laser_angle_step / 2;

//        double p1_x = d * cos(theta1);
//        double p1_y = d * sin(theta1);

            double p2_x = d * cos(theta2)/1000.0;
            double p2_y = d * sin(theta2)/1000.0;
            // p1, p2的中点
//        double p3_x = (p1_x + p2_x) / 2;
//        double p3_y = (p1_y + p2_y) / 2;

//        local_points.emplace_back(p1_x, p1_y);
            local_points.emplace_back(p2_x, p2_y);
            angle_index += 1;
        }

        auto filter_op = oba::RackObaFilterSingleton::getInstance();
        // 将传感器坐标系下的点转换为世界坐标系下的点
        for (auto p : local_points) {
            Eigen::Vector2d base_point(p.x(), p.y());
            if (para->is_load_full_state) {
                if(filter_op->isRackPoint(base_point)){
                    continue;
                }
            }
            Eigen::Vector2d world_point = world_tf * base_point;
            ob_points.emplace_back(world_point[0],world_point[1]);  // mm -> m
        }
        // 发送给NavigationModule对障碍点进行处理
        oba_msg->point_cloud = ob_points;
    }

private:
    Eigen::Affine2d r2100_tf;
    std::string oba_name;

};
}



#endif //SROS_R2100_SENSOR_MANAGER_HPP
