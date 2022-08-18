//
// Created by lfc on 19-3-1.
//

#ifndef SROS_STEREO_CAMERA_MANAGER_HPP
#define SROS_STEREO_CAMERA_MANAGER_HPP

#include "base_sensor_manager.hpp"
#include "core/msg/ObstacleMsg.hpp"

namespace sensor{
class StereoCameraManager : public BaseSensorManager {
public:
    StereoCameraManager(oba::ObstacleModulePara_Ptr para,ObaMsgCallback msg):BaseSensorManager(para,msg,TYPE_STEREO_CAMERA) {
        stereo_tfs[sros::device::DEVICE_CAMERA_D435] = Eigen::Translation2d(para->stereo_camera_coord_x, para->stereo_camera_coord_y) *
                  Eigen::Rotation2Dd(para->stereo_camera_coord_yaw);
        stereo_tfs[sros::device::DEVICE_CAMERA_D435_2] = Eigen::Translation2d(para->stereo_camera_2_coord_x, para->stereo_camera_2_coord_y) *
                  Eigen::Rotation2Dd(para->stereo_camera_2_coord_yaw);
        stereo_tfs[sros::device::DEVICE_CAMERA_DEPTH] = Eigen::Translation2d(para->stereo_camera_coord_x, para->stereo_camera_coord_y) *
                                                       Eigen::Rotation2Dd(para->stereo_camera_coord_yaw);
        stereo_tfs[sros::device::DEVICE_CAMERA_DEPTH_2] = Eigen::Translation2d(para->stereo_camera_2_coord_x, para->stereo_camera_2_coord_y) *
                                                         Eigen::Rotation2Dd(para->stereo_camera_2_coord_yaw);
        stereo_tfs[sros::device::DEVICE_CAMERA_D435_3] = Eigen::Translation2d(para->stereo_camera_3_coord_x, para->stereo_camera_3_coord_y) *
                  Eigen::Rotation2Dd(para->stereo_camera_3_coord_yaw);
        stereo_tfs[sros::device::DEVICE_CAMERA_D435_4] = Eigen::Translation2d(para->stereo_camera_4_coord_x, para->stereo_camera_4_coord_y) *
                  Eigen::Rotation2Dd(para->stereo_camera_4_coord_yaw);
        //oba_name = sros::device::DEVICE_CAMERA_D435;
        //oba_name = sros::device::DEVICE_CAMERA_DEPTH;
    }

    virtual void processMsg(sros::core::base_msg_ptr& m) {
        sros::core::ObstacleMsg_ptr origin_stereo = std::dynamic_pointer_cast<sros::core::ObstacleMsg>(m);
        auto stereo_iter = stereo_tfs.find(origin_stereo->oba_name);
        if (stereo_iter == stereo_tfs.end()) {
            LOG(INFO) << "cannot find stereo tf! will return!" << origin_stereo->oba_name;
        }
        auto stereo_tf = stereo_iter->second;
        slam::tf::TransForm curr_pose;
        if(!base_to_world_tf->lookUpTransForm(origin_stereo->time_, curr_pose,para->delta_time_thresh)){
            //LOG(INFO) << oba_name << "oba err to get the realtime msg!";
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
        auto camera_to_world_tf = world_tf * stereo_tf;
        sros::core::ObstacleMsg_ptr stereo_obas(new sros::core::ObstacleMsg("OBSTACLES"));
        stereo_obas->oba_name = origin_stereo->oba_name;
        stereo_obas->time_ = m->time_;
        convertToObas(camera_to_world_tf, origin_stereo, stereo_obas);
        auto points = stereo_obas->point_cloud;
        if (!para->use_stereo_points) {//这里不能够直接用这个开关来控制D435的开闭
            stereo_obas->point_cloud.clear();
        }
        if (!para->is_load_full_state) {
            stereo_obas->point_cloud.clear();
            for (auto point:points) {
                if (point.z() < para->car_heght){
                    stereo_obas->point_cloud.push_back(point);
                }
            }
        }
        sendObaMsg(stereo_obas);
    }

private:

    void convertToObas(Eigen::Affine2d& world_tf,const sros::core::ObstacleMsg_ptr &origin_msg,sros::core::ObstacleMsg_ptr& oba_msg){
        for (auto &point:origin_msg->point_cloud) {
            auto world_point = world_tf * Eigen::Vector2d(point.x(), point.y());
            sros::core::Location loc(world_point[0], world_point[1],point.z());
            oba_msg->point_cloud.push_back(loc);
        }
    }

    std::map<std::string,Eigen::Affine2d> stereo_tfs;
    std::string oba_name;

};
}



#endif //SROS_STEREO_CAMERA_MANAGER_HPP
