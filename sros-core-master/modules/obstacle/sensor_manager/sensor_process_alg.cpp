//
// Created by lfc on 18-12-13.
//

#include "sensor_process_alg.h"
#include "laser_sensor_manager.hpp"
#include "r2100_sensor_manager.hpp"
#include "stereo_camera_manager.hpp"
#include "sh100_sensor_manager.hpp"
#include "sh200_sensor_manager.hpp"
#include "ifm_sensor_manager.hpp"
#include "eu100_tim320_sensor_manager.hpp"

namespace sensor{
void filterScanIsolatedPoint(sros::core::LaserScan_ptr scan,double min_thresh) {
    auto& ranges = scan->ranges;
    double theta_thresh=sin((double)scan->angle_increment)/sin(0.170);//临界值,用于识别断点
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


template <class LaserScan_ptr>
void filterScanTrailPoint(LaserScan_ptr scan,double max_distance,double min_thresh) {
    auto& ranges = scan->ranges;
    std::vector<int> indexes;
    const int step = 2;
    double cos_increment = cos(scan->angle_increment * (double)step);
    double theta_thresh=sin((double)scan->angle_increment * (double)step)/sin(0.17);//临界值,用于识别断点
    int scan_size = ranges.size() - step;

//        std::vector<int> indexes;
    for (int i = step; i < scan_size; i++) {
        if (ranges[i] == 100 || ranges[i] == 0 || ranges[i] > max_distance) {
            continue;
        }
        double dist_direction = (ranges[i + step] - ranges[i - step]);
        for (int k = -step; k < step; ++k) {
            double tmp_direction = (ranges[i + k + 1] - ranges[i + k]);
            if (dist_direction * tmp_direction <= 0) {
                continue;
            }
        }
        double dist_1 = std::sqrt(ranges[i] * ranges[i] + ranges[i - step] * ranges[i - step] -
                                  2 * ranges[i] * ranges[i - step] * cos_increment);
        double dist_2 = std::sqrt(ranges[i] * ranges[i] + ranges[i + step] * ranges[i + step] -
                                  2 * ranges[i] * ranges[i + step] * cos_increment);
        double range_thresh_1 = ranges[i] * theta_thresh + min_thresh;
        double range_thresh_2 = ranges[i + step] * theta_thresh + min_thresh;
        if(dist_1 > range_thresh_1 && dist_2 > range_thresh_2) {
            for (int j = -step; j <= step; ++j) {
                indexes.push_back(i + j);
            }
        }
    }
    for (auto &index:indexes) {
        ranges[index] = 100;
    }
}

template <class LaserScan_ptr>
void filterLowIntenPoints(LaserScan_ptr scan,double max_distance){
    int scan_size = scan->ranges.size();

    double tail_center_angle_min = -1.5707;
    double tail_center_angle_max = 1.5707;

    double angle = scan->angle_min;
    double step = scan->angle_increment;

    int first_tail_center = (int) round((tail_center_angle_min - angle) / step);
    int second_tail_center = (int) round((tail_center_angle_max - angle) / step);

    int delta_thresh = (int) round(0.51 / scan->angle_increment);

    for (int i = 0; i < scan_size; ++i) {
        if ((i - first_tail_center) < delta_thresh || (second_tail_center - i) < delta_thresh) {
            if (scan->ranges[i] < max_distance) {
                if (scan->intensities[i] < 150) {
                    scan->ranges[i] = 0;
                }
            }
        }
    }
}

template <class LaserScan_ptr>
void filterScanPoint(LaserScan_ptr scan) {
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

BaseSensorManager_Ptr getSensorManager(oba::ObstacleModulePara_Ptr para,ObaMsgCallback sendMsg,SensorType type) {
    BaseSensorManager_Ptr sensor_manager;
    switch (type) {
        case TYPE_LASER_SENSOR:
            sensor_manager.reset(new LaserSensorManager(para, sendMsg));
            break;
        case TYPE_R2100_SENSOR:
            sensor_manager.reset(new R2100SensorManager(para,sendMsg));
            break;
        case TYPE_SH100_SENSOR:
            sensor_manager.reset(new Sh100SensorManager(para,sendMsg));
            break;
        case TYPE_SH200_SENSOR:
            sensor_manager.reset(new Sh200SensorManager(para,sendMsg));
            break;
        case TYPE_EU100_TIM320_SENSOR:
            sensor_manager.reset(new Eu100Tim320SensorManager(para,sendMsg));
            break;
        case TYPE_STEREO_CAMERA:
            sensor_manager.reset(new StereoCameraManager(para, sendMsg));
            break;
        case TYPE_IFM_SENSOR:
            sensor_manager.reset(new IfmSensorManager(para, sendMsg));
    }
    return sensor_manager;

}


void passThroughFilter(const Range3D<float> &range, const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud) {
    if (cloud->points.empty()) {
        LOG(INFO) << "passthrough filter: cloud.size=0";
        return;
    }
    const int size = cloud->size();
    output_cloud->reserve(size);
    const Point3D *point;
    for (size_t i = 0; i < size; ++i) {
        point = &cloud->points.at(i);
        if (IS_WITHIN_SCOPE(point->x, range.min_x, range.max_x)
            && IS_WITHIN_SCOPE(point->y, range.min_y, range.max_y)
            && IS_WITHIN_SCOPE(point->z, range.min_z, range.max_z)) {
            output_cloud->emplace_back(*point, cloud->image_indices.at(i));
        }
    }
    LOG(INFO) << "after pass through filter: input_point.size=" << cloud->size()
                  << " -> output_cloud.size=" << output_cloud->size();
}

void outThroughFilter(const std::vector<double> &range, const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud) {
    if (cloud->points.empty()) {
        LOG(INFO) << "outthrough filter: cloud.size=0";
        return;
    }
    const int size = cloud->size();
    output_cloud->reserve(size);
    const Point3D *point;
    for (size_t i = 0; i < size; ++i) {
        point = &cloud->points.at(i);
        if (IS_WITHIN_SCOPE(point->x, range[0], range[1])
            && IS_WITHIN_SCOPE(point->z, range[2], range[3])) {
            continue;
        }
        output_cloud->emplace_back(*point, cloud->image_indices.at(i));
    }
    LOG(INFO) << "after out through filter: input_point.size=" << cloud->size()
                  << " -> output_cloud.size=" << output_cloud->size();
}




void getMaxMin(const PointCloudConstPtr &cloud, Eigen::Vector4f& min_p, Eigen::Vector4f& max_p){
    //find the minimum value of x, y, z
    if (cloud->points.empty()) {
		LOG(INFO) << "getMaxMin input : cloud.size=0";
        return;
    } 
    float min_x = (*std::min_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.x < b.x;})).x;
    float min_y = (*std::min_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.y < b.y;})).y;
    float min_z = (*std::min_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.z < b.z;})).z;
	min_p = {min_x, min_y, min_z, 1.0};


	float max_x = (*std::max_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.x < b.x;})).x;
	float max_y = (*std::max_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.y < b.y;})).y;
    float max_z = (*std::max_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.z < b.z;})).z;

    max_p = {max_x, max_y, max_z, 1.0};

}

void voxelGridFilter(const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud, const Eigen::Vector3f &voxel_range){
  
    const int size = cloud->size();
    if (cloud->points.empty()) {
		LOG(INFO) << "voxelgrid filter: cloud.size=0";
        return;
    } 
    
    Eigen::Vector4f min_p{0.0, 0.0, 0.0, 1.0}, max_p{0.0, 0.0, 0.0, 1.0};
	getMaxMin(cloud, min_p, max_p);

    Eigen::Vector4f inverse_leaf_size_;
	inverse_leaf_size_[0] = 1.0 / voxel_range[0];
	inverse_leaf_size_[1]=  1.0 / voxel_range[1];
	inverse_leaf_size_[2] = 1.0 / voxel_range[2];
	inverse_leaf_size_[3] = 1.0;
    
    //计算最小和最大边界框值
	Eigen::Vector4f min_b_, max_b_, div_b_, divb_mul_;
	min_b_[0] = static_cast<int> (std::floor(min_p[0] * inverse_leaf_size_[0]));
	max_b_[0] = static_cast<int> (std::floor(max_p[0] * inverse_leaf_size_[0]));
	min_b_[1] = static_cast<int> (std::floor(min_p[1] * inverse_leaf_size_[1]));
	max_b_[1] = static_cast<int> (std::floor(max_p[1] * inverse_leaf_size_[1]));
	min_b_[2] = static_cast<int> (std::floor(min_p[2] * inverse_leaf_size_[2]));
	max_b_[2] = static_cast<int> (std::floor(max_p[2] * inverse_leaf_size_[2]));

    //计算沿所有轴所需的分割数
    div_b_[0] = max_b_[0] - min_b_[0] + 1;
    div_b_[1] = max_b_[1] - min_b_[1] + 1;
    div_b_[2] = max_b_[2] - min_b_[2] + 1;
    div_b_[3] = 0;

    //设置除法乘数
    divb_mul_[0] = 1;
    divb_mul_[1] = div_b_[0];
    divb_mul_[2] = div_b_[0] * div_b_[1];
    divb_mul_[3] = 0;

    //用于计算idx和pointcloud索引的存储
	std::vector<PointIndices> index_vector;
	index_vector.reserve(cloud->points.size());

    //第一步：遍历所有点并将它们插入到具有计算idx的index_vector向量中;具有相同idx值的点将有助于产生CloudPoint的相同点
	for (std::size_t i = 0; i < cloud->points.size();++i)
	{
		int ijk0 = static_cast<int> (std::floor(cloud->points[i].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
		int ijk1 = static_cast<int> (std::floor(cloud->points[i].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
		int ijk2 = static_cast<int> (std::floor(cloud->points[i].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

		//计算质心叶索引
		int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1]+ ijk2 * divb_mul_[2];
		index_vector.push_back(PointIndices(static_cast<unsigned int> (idx), i));
	}
    //第二步：使用表示目标单元格的值作为索引对index_vector向量进行排序;实际上属于同一输出单元格的所有点都将彼此相邻
    std::sort(index_vector.begin(), index_vector.end(), std::less<PointIndices>());

	//第三步：计数输出单元格，我们需要跳过所有相同的，相邻的idx值
	unsigned int total = 0;
	unsigned int index = 0;
	unsigned int min_points_per_voxel_ = 0;
	
	std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
	first_and_last_indices_vector.reserve(index_vector.size());                              //分配内存空间

    while (index < index_vector.size())
	{
		unsigned int i = index + 1;
		while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
			++i;
		if ((i - index) >= min_points_per_voxel_)
		{
			++total;
            //first_and_last_indices_vector.emplace_back(index, i);
			first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
		}
		index = i;
	}
    //第四步：计算质心，将它们插入最终位置
	//OutPointCloud.resize(total);      //给输出点云分配内存空间
	float x_Sum, y_Sum, z_Sum;
	Point3D PointCloud;
	unsigned int first_index, last_index;
	for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
	{
		// 计算质心 - 来自所有输入点的和值，这些值在index_vector数组中具有相同的idx值
		first_index = first_and_last_indices_vector[cp].first;
		last_index = first_and_last_indices_vector[cp].second;
		x_Sum = 0;
		y_Sum = 0;
		z_Sum = 0;
		for (unsigned int li = first_index; li < last_index; ++li)
		{
			x_Sum += cloud->points[index_vector[li].cloud_point_index].x;
			y_Sum += cloud->points[index_vector[li].cloud_point_index].y;
			z_Sum += cloud->points[index_vector[li].cloud_point_index].z;

		}
		PointCloud.x = x_Sum / (last_index - first_index);
		PointCloud.y = y_Sum / (last_index - first_index);
		PointCloud.z = z_Sum / (last_index - first_index);
		output_cloud->push_back(PointCloud, cp);

	}
}

void outlierFilter(const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud) {

    KdTree3dPtr kd_tree = std::make_shared<KdTree3d>();
    kd_tree->build(cloud->points);

    std::vector<int> indices;
    StatisticalOutlierRemoval outlier_filter;
    outlier_filter.setKdTree(kd_tree);
    outlier_filter.setInputCloud(cloud);
    outlier_filter.setMeanK(20);
    outlier_filter.setStddevMulThresh(1);
    outlier_filter.filter(indices);

    copyPointCloudIndices(cloud, indices, output_cloud);

    LOG(INFO) << "after outlier filter: input_point.size=" << cloud->size()
                << " -> output_cloud.size=" << output_cloud->size();
}

void radiusFilter(const PointCloudConstPtr &cloud, const PointCloudPtr &output_cloud) {

    KdTree3dPtr kd_tree = std::make_shared<KdTree3d>();
    kd_tree->build(cloud->points);

    std::vector<int> indices;
    RadiusFilter radius_filter;
    radius_filter.setKdTree(kd_tree);
    radius_filter.setInputCloud(cloud);
    radius_filter.setRadiusSearch(0.05);
    radius_filter.setMinNeighborsInRadius(50);
    radius_filter.filter(indices);

    copyPointCloudIndices(cloud, indices, output_cloud);

    LOG(INFO) << "after radius filter: input_point.size=" << cloud->size()
                << " -> output_cloud.size=" << output_cloud->size();
}
/**
 * @brief euclidean segmentation.
 * @param[in] param card detect param.
 * @param[in] cloud input point cloud.
 * @param[out] cluster segmentation set.
 */
void euclidean(const PointCloudConstPtr &cloud,
               std::vector<Indices> &clusters) {
    int64_t start, finish;
    start = common_func::get_timestamp_in_ms();

    // The KD tree object is established to search the nearest point.
    KdTree3d::Ptr kd_tree = std::make_shared<KdTree3d>();
    kd_tree->build(cloud->points);

    // Euclidean Clustering objects.
    EuclideanClusterExtraction clustering;
    // set small values of cluster point size  (small values may cause objects to be divided
    // in several clusters, whereas big values may join objects in a same cluster).
    clustering.setClusterTolerance(0.02);
    clustering.setMinClusterSize(50);
    clustering.setMaxClusterSize(1000);
    clustering.setSearchMethod(kd_tree);
    clustering.setInputCloud(cloud);
    clustering.extract(clusters);

    finish = common_func::get_timestamp_in_ms();
    int64_t euclidean_segmentation_time = finish - start;
    LOG(INFO) << "euclidean Segmentation: cloud.size=" << cloud->size() << " -> clusters.size=" << clusters.size()
                << " time consuming " << euclidean_segmentation_time << "ms";
}
}
