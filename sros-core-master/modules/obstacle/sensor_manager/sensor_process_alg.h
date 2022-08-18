//
// Created by lfc on 18-12-13.
//

#ifndef SROS_SENSOR_PROCESS_ALG_HPP
#define SROS_SENSOR_PROCESS_ALG_HPP
#include "core/msg/laser_scan_msg.hpp"
#include "base_sensor_manager.hpp"
#include "../../perception/lib/common/algorithm/statistical_outlier_removal.h"
#include "../../perception/lib/common/algorithm/euclidean_cluster_extraction.h"
#include "../../perception/lib/common/algorithm/radius_filter.h"
#include "../../perception/common/point_cloud.hpp"
#include "../../perception/common/cloud_memory.hpp"
#include "../obstacle_module_para.hpp"
using namespace oba;

namespace sensor{

void filterScanIsolatedPoint(sros::core::LaserScan_ptr scan,double min_thresh = 0.03);

template <class LaserScan_ptr>
void filterScanTrailPoint(LaserScan_ptr scan,double max_distance,double min_thresh = 0.02);

template <class LaserScan_ptr>
void filterLowIntenPoints(LaserScan_ptr scan,double max_distance);

BaseSensorManager_Ptr getSensorManager(oba::ObstacleModulePara_Ptr para,ObaMsgCallback sendMsg,SensorType type);



void passThroughFilter(const Range3D<float> &range, const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud);

void outThroughFilter(const std::vector<double> &range, const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud);

void voxelGridFilter(const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud, const Eigen::Vector3f &voxel_range);

void outlierFilter(const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud);

void radiusFilter(const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud);

void getMaxMin(const PointCloudConstPtr &cloud, Eigen::Vector4f& min_p, Eigen::Vector4f& max_p);

template<class Param>
void euclidean(const Param &param, const NormalCloudConstPtr &normals, const PointCloudConstPtr &cloud,
                std::vector<Indices> &clusters);
}


#endif //SROS_SENSOR_PROCESS_ALG_HPP
