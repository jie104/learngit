/**
 * @file ifm_sensor_manager.hpp
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author chaohuo wu
 * @date create date：2022/04/11.
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_IFM_SENSOR_MANAGER_HPP
#define SROS_IFM_SENSOR_MANAGER_HPP
#include "base_sensor_manager.hpp"
#include "core/msg/image_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/pose.h"
#include "core/core.h"
#include "../../perception/common/o3d3xxFrame.hpp"
#include "../../perception/common/point_cloud.hpp"
#include "../../perception/perception-sros/detector_base/unit.hpp"
#include "../../perception/perception-sros/detector_base/detection_base.h"
#include "sensor_process_alg.h"
#include "core/module.h"
using namespace sros::core;

namespace sensor{
class IfmSensorManager : public BaseSensorManager{
    public:
        IfmSensorManager(oba::ObstacleModulePara_Ptr para,ObaMsgCallback msg):BaseSensorManager(para,msg,TYPE_IFM_SENSOR) {
            ifm_tfs[sros::device::DEVICE_CAMERA_O3D303] = Eigen::Translation2d(para->ifm_camera_coord_x, para->ifm_camera_coord_y) *
                    Eigen::Rotation2Dd(para->ifm_camera_coord_yaw);
            
            oba_name = sros::device::DEVICE_CAMERA_O3D303;
            xyz_range = para->detect_range;

            auto &s = sros::core::Settings::getInstance();
            fork_arm_length = s.getValue("forklift.fork_arm_length", 0.9);
        }

        virtual void processMsg(sros::core::base_msg_ptr& m){
            int64_t start, finish;

            sros::core::image_msg_ptr origin_ifm = std::dynamic_pointer_cast<sros::core::ImageMsg>(m);
            origin_ifm->mat_xyz_.convertTo(o3d3xx_frame.xyz_img, CV_16SC3);
            common_func::convertPointCloud(o3d3xx_frame.xyz_img, o3d3xx_frame.cloud);
            //LOG(INFO) << "o3d3xx_frame.cloud size : " << o3d3xx_frame.cloud->points.size();
            //LOG(INFO) << "o3d3xx_frame.xyz_img size : " << o3d3xx_frame.xyz_img.size(); // [352 x 264]
            //recordSensorData("/sros/log/", "o3d3xx_frame.cloud",  o3d3xx_frame.cloud);
            
            auto ifm_iter = ifm_tfs.find(oba_name);
            if (ifm_iter == ifm_tfs.end()) {
                LOG(INFO) << "cannot find ifm camera tf! will return!" << oba_name;
            }
            auto ifm_tf = ifm_iter->second;
            slam::tf::TransForm curr_pose;
            if(!base_to_world_tf->lookUpTransForm(origin_ifm->getTimestamp(), curr_pose,para->delta_time_thresh)){
                LOG(INFO) << oba_name << "oba err to get the realtime msg!";
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
            
            // passthrough filter 
            //oba::Range3D<float> xyz_range{2, 4, -1, 1, -1, 4};
            start = common_func::get_time_in_ms();
            PointCloudPtr filter_xyz_range_cloud_(new PointCloud());
            passThroughFilter(xyz_range, o3d3xx_frame.cloud, filter_xyz_range_cloud_);
            //finish = common_func::get_time_in_ms();

            // 排除叉臂的范围: 比叉臂长0.1m, 排除在叉臂表面和地面的距离。
            std::vector<double> arm_range{0.0, fork_arm_length+arm_offset, z_range_min, z_range_max};
            PointCloudPtr arm_xyz_range_cloud_(new PointCloud());
            outThroughFilter(arm_range, filter_xyz_range_cloud_, arm_xyz_range_cloud_);


            //recordSensorData("/sros/log/", "filter_xyz_range_cloud_",  filter_xyz_range_cloud_);
            // voxel filter 
            // Eigen::Vector3f voxel_range(0.05, 0.05, 0.05);
            // PointCloudPtr filter_voxelgrid_cloud_(new PointCloud());
            // voxelGridFilter(filter_xyz_range_cloud_, filter_voxelgrid_cloud_, voxel_range);
            
            // start = common_func::get_time_in_ms();
            // std::vector<Indices> clusters_indices;
            // euclidean(filter_xyz_range_cloud_,
            //           clusters_indices);
            // finish = common_func::get_time_in_ms();
            

            // outlier filter 
            PointCloudPtr outlierFilter_cloud_(new PointCloud());
            outlierFilter(arm_xyz_range_cloud_, outlierFilter_cloud_);
            //recordSensorData("/sros/log/", "outlierFilter_cloud_",  outlierFilter_cloud_);

            // tricks: 如果离群点滤波大于某个值，没必要进行半径滤波了。
            PointCloudPtr radiusFilter_cloud_(new PointCloud());
            if(outlierFilter_cloud_->points.size() > 5000){
                radiusFilter_cloud_ = outlierFilter_cloud_;
            }else{
                //radius removal 
                radiusFilter(outlierFilter_cloud_, radiusFilter_cloud_);
            }

            //recordSensorData("/sros/log/", "radiusFilter_cloud_",  radiusFilter_cloud_);
            // 在目标点左右前后0.8m处的点云会被忽略，即排除栈板周围的点云。// 后续需要改进
            
            oba::Range3D<double> pallet_xyz_range{target_position.x()-offset, target_position.x()+offset, target_position.y()-offset, target_position.y()+offset, -1.0, 4.0};

            Eigen::Affine2d world_tf = Eigen::Translation2d(curr_pose.position.x(), curr_pose.position.y()) *
                                   Eigen::Rotation2Dd(curr_pose.rotation.yaw());
            auto camera_to_world_tf = world_tf * ifm_tf;
            sros::core::ObstacleMsg_ptr ifm_obas(new sros::core::ObstacleMsg("OBSTACLES"));
            ifm_obas->oba_name = oba_name;
            ifm_obas->time_ = origin_ifm->getTimestamp();
            convertToObas(camera_to_world_tf, pallet_xyz_range, radiusFilter_cloud_, ifm_obas);
            LOG(INFO) << "ifm_obas->point_cloud.size(): " << ifm_obas->point_cloud.size();
            // tricks: 如果最后剩余小于5个点，直接忽略，排除噪点。
            // TODO: 聚类，求每个类别的最大距离，且每个类的点数如果小于某个直，可以忽略;聚类的最大小于50mm可以忽略
            if(ifm_obas->point_cloud.size() <= 5){
                ifm_obas->point_cloud.clear();
            }
            sendObaMsg(ifm_obas);
            finish = common_func::get_time_in_ms();
            LOG(INFO) << "time for processing ifm point cloud: " << finish - start;
            
        }

        virtual void set_target_position(sros::core::Pose position){
            target_position = position;
        }
        
    private:
        
        void convertToObas(Eigen::Affine2d& world_tf, oba::Range3D<double> pallet_xyz_range, const PointCloud::Ptr& cloud,sros::core::ObstacleMsg_ptr& oba_msg){
            oba_msg->point_cloud.reserve(cloud->points.size());
            for (auto &point: cloud->points) {
                auto world_point = world_tf * Eigen::Vector2d(point.x, point.y);
                // ignore points around target position
                if (IS_WITHIN_SCOPE(world_point[0], pallet_xyz_range.min_x, pallet_xyz_range.max_x)
                    && IS_WITHIN_SCOPE(world_point[1], pallet_xyz_range.min_y, pallet_xyz_range.max_y)) {
                    continue;
                }
                sros::core::Location loc(world_point[0], world_point[1],point.z);
                oba_msg->point_cloud.emplace_back(loc);
            }
        }

        O3d3xxFrame o3d3xx_frame;
        std::map<std::string,Eigen::Affine2d> ifm_tfs;
        std::string oba_name;
        oba::Range3D<float> xyz_range {};
        std::ofstream out_put_;
        sros::core::Pose  target_position;
        double fork_arm_length = 0.0;
        const double arm_offset = 0.1;
        const double z_range_min = -0.4;
        const double z_range_max = -0.27;
        const double offset = 0.8;
        
};

} // namespace sensor


#endif //SROS_IFM_SENSOR_MANAGER_HPP


