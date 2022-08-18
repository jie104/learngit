/**
 * @file detector.cpp
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/17
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "detector.h"
#include "../../common/algorithm/statistical_outlier_removal.h"
#include "../../common/algorithm/estimated_normal.h"
#include "../../common/algorithm/fitting_plane.h"
#include <queue>
#include <opencv2/opencv.hpp>
// CODE
namespace perception{


#define iszerof(a) ((fabsf(a) < EPSILON) ? true : false);

Detector::Detector() 
    : filter_mask_cloud_(new PointCloud())
    , filter_front_cloud_(new PointCloud())
    , filter_xyz_range_cloud_(new PointCloud())
    , filter_median_cloud_(new PointCloud())
    , filter_outlier_cloud_(new PointCloud())
    , filter_normal_point_cloud_(new PointCloud())
    , filter_normal_normal_cloud_(new NormalCloud())
    , target_cloud_(new PointCloud())
    , clusters_cloud_(new PointCloud()) {
    
}

    void
    Detector::drawMask() {
        common_func::GrayMaptoColor(this->filter_mask_img_, this->mask_img_,
                                    cv::COLORMAP_OCEAN);
        putText(this->mask_img_, "mask", cv::Point(10, 20),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                1, 1);
    }

    void
    Detector::drawCameraImage() {
        common_func::GrayMaptoColor(this->amplitude_normalize_, this->camera_img_,
                                    cv::COLORMAP_BONE);
        putText(this->camera_img_, "camera image", cv::Point(10, 20),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                1, 1);
    }

    void
    Detector::gatherAllClusterCenter() {
        clusters_cloud_->clear();
        for (const auto &cluster: clusters_map_) {
            auto points = cluster.second.getPointCloud();
            for (auto point: points->points) {
                clusters_cloud_->push_back(point, 0);
            }
        }
    }

    void
    Detector::getCameraImage(cv::Mat &img) const {
        img = this->camera_img_.clone();
    }

    void
    Detector::getTargetImage(cv::Mat &img) const {
        img = this->target_img_.clone();
    }

    void
    Detector::getMaskCloud(const PointCloudPtr &filter_mask_cloud) const {
        copyPointCloud(this->filter_mask_cloud_, filter_mask_cloud);
    }

    void
    Detector::getXYZRangeCloud(const PointCloudPtr &filter_xyz_range_cloud) const {
        copyPointCloud(this->filter_xyz_range_cloud_, filter_xyz_range_cloud);
    }

    void
    Detector::getMedianFilterCloud(const PointCloudPtr &after_median_filter_cloud) const {
        copyPointCloud(this->filter_median_cloud_, after_median_filter_cloud);
    }

    void
    Detector::getOutlierCloud(const PointCloudPtr &filter_median_cloud) const {
        copyPointCloud(this->filter_outlier_cloud_, filter_median_cloud);
    }

    void
    Detector::getFilterNormalPointCloud(const PointCloudPtr &filter_normal_poing_cloud) const {
        copyPointCloud(this->filter_normal_point_cloud_, filter_normal_poing_cloud);
    }

    void
    Detector::getNormalFilterNormalCloud(const NormalCloudPtr &filter_normal_normal_cloud) const {
        copyNormalCloud(this->filter_normal_normal_cloud_, filter_normal_normal_cloud);
    }

    void
    Detector::getTargetPointCloud(const PointCloudPtr &cloud) const {
        copyPointCloud(this->target_cloud_, cloud);
    }

    void
    Detector::getClusterPointCloud(const PointCloudPtr &cloud) const {
        copyPointCloud(this->clusters_cloud_, cloud);
    }

    void
    Detector::getClusterCenter(std::vector <Point3D> &points) const {
        float x, y, z;
        for (const auto &cluster: this->clusters_map_) {
            x = cluster.second.getAvgX();
            y = cluster.second.getAvgY();
            z = cluster.second.getAvgZ();
            points.emplace_back(x, y, z);
        }
    }

    void
    Detector::getClusterNormal(std::vector <Normal> &centers) const {
        for (const auto &cluster: this->clusters_map_)
            centers.push_back(cluster.second.getNormal());
    }

    void
    Detector::getConsumingTime(std::vector <std::string> &consume_time) const {
        consume_time.resize(this->consuming_time_recorder_.size());
        copy(this->consuming_time_recorder_.begin(), this->consuming_time_recorder_.end(),
             consume_time.begin());
    }


    void
    Detector::generateDistanceImage(const PointCloudConstPtr &cloud,
                                    cv::Mat_ <uint16_t> &distance_img) const {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        distance_img = cv::Mat::zeros(this->image_height_, this->image_width_, CV_16UC1);
        int r, c, size;
        const Point3D *pt;
        size = cloud->size();
        for (size_t i = 0; i < size; ++i) {
            c = cloud->image_indices.at(i) % this->image_width_;
            r = cloud->image_indices.at(i) / this->image_width_;
            pt = &cloud->points.at(i);
            distance_img(r, c) = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z) * 1000;
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t generate_distance_image_time = finish - start;
        LOG(INFO) << "generate distance image: time consuming " << generate_distance_image_time << "ms";
    }

    void
    Detector::maskFilter(const cv::Mat &confidence_img,
                         const cv::Mat &amplitude_img,
                         const PointCloudConstPtr &input_cloud,
                         cv::Mat &confidence_normalize,
                         cv::Mat &amplitude_normalize,
                         cv::Mat &filter_mask,
                         PointCloudPtr &output_cloud) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        cv::imwrite("/sros/debug_data/confidence_img.jpg",confidence_img);
        cv::imwrite("/sros/debug_data/amplitude_img.jpg",amplitude_img);


        auto getMask = [](const cv::Mat_<uint8_t> &src_img, cv::Mat &dst_img) {
            cv::normalize(src_img, dst_img, 0, 255, cv::NORM_MINMAX);
            cv::Scalar scalar = cv::mean(dst_img);

            // default select pix > mean
            double mean = scalar.val[0];
            cv::Mat mask = dst_img > mean;

//   cv::imshow("dst_img",dst_img);
//   cv::imshow("mask", mask);
//   cv::waitKey(0);

            return mask;
        };

        const cv::Mat confident_mask = getMask(confidence_img, confidence_normalize);
        const cv::Mat amplitude_mask = getMask(amplitude_img, amplitude_normalize);
        filter_mask = amplitude_mask & (255 - confident_mask);
        cv::imwrite("/sros/debug_data/confident_mask.jpg",confident_mask);
        cv::imwrite("/sros/debug_data/amplitude_mask.jpg",amplitude_mask);


        // cv::imshow("confident_img", confidence_img);
        // cv::imshow("amplitude_img", amplitude_img);
        // cv::imshow("confident_mask", confident_mask);
        // cv::imshow("amplitude_mask", amplitude_mask);
        // cv::imshow("confidence_normalize", confidence_normalize);
        // cv::imshow("amplitude_normalize", amplitude_normalize);
        // cv::imshow("filter_mask",filter_mask);
        //cv::waitKey(0);

        // Set structure element type, size and anchor location .
        // cv::Mat elemStruct = getStructuringElement(
        //         cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));

        // dilate(filter_mask, filter_mask, elemStruct);
        // erode(filter_mask, filter_mask, elemStruct);
        // dilate(filter_mask, filter_mask, elemStruct);
        // erode(filter_mask, filter_mask, elemStruct);

        // Keep only usable points; .
        size_t available_count = 0, index;
        output_cloud->reserve(filter_mask.rows * filter_mask.cols);
        for (size_t i = 0; i < filter_mask.rows; ++i) {
            for (size_t j = 0; j < filter_mask.cols; ++j) {
                if (0 == filter_mask.at<uint8_t>(i, j)) {
                    continue;
                }
                index = i * filter_mask.cols + j;
                output_cloud->push_back(input_cloud->points.at(index), index);
                available_count++;
            }
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t filter_pointcloud_by_mask_time = finish - start;
        LOG(INFO) << "after filter by mask: input_cloud.size=" << input_cloud->size() << " -> output_cloud.size="
                  << available_count << " time consuming " << filter_pointcloud_by_mask_time << "ms";
    }

    void
    Detector::maskFilter(const cv::Mat &mask_img,
                         const PointCloudConstPtr &input_cloud,
                         PointCloudPtr &output_cloud) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();
        // Keep only usable points;
        size_t available_count = 0, index;
        LOG(INFO) << "reserve output_could.size:" << mask_img.rows << "  " << mask_img.cols;
        output_cloud->reserve(mask_img.rows * mask_img.cols);
        LOG(INFO) << "output_could.size:" << output_cloud->size();
        for (size_t i = 0; i < mask_img.rows; ++i) {
            for (size_t j = 0; j < mask_img.cols; ++j) {
                if (0 == mask_img.at<uint8_t>(i, j)) {
                    continue;
                }
                index = i * mask_img.cols + j;
                output_cloud->push_back(input_cloud->points.at(index), index);
                available_count++;
            }
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t filter_pointcloud_by_mask_time = finish - start;
        LOG(INFO) << "after filter by mask: input_cloud.size=" << input_cloud->size() << " -> output_cloud.size="
                  << available_count << " time consuming " << filter_pointcloud_by_mask_time << "ms";

    }

    void Detector::getMaxMin(const PointCloudConstPtr &cloud, Eigen::Vector4f& min_p, Eigen::Vector4f& max_p){
        //find the minimum value of x, y, z
        if (cloud->points.empty()) {
            LOG(INFO) << "getMaxMin input : cloud.size=0";
            return;
        }
        float min_x = (*std::min_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.x < b.x;})).x;
        float min_y = (*std::min_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.y < b.y;})).y;
        float min_z = (*std::min_element(cloud->points.begin(), cloud->points.end(), [](const Point3D& a, const Point3D& b){return a.z < b.z;})).z;
        min_p = {min_x, min_y, min_z, 1.0};
    }
    void
    Detector::normalFilter(const NormalCloudConstPtr &normals,
                           const PointCloudConstPtr &cloud,
                           const float &angle_tolerance,
                           PointCloudPtr &output_cloud,
                           NormalCloudPtr &output_normal) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        float len, theta;
        int available_count = 0;
        const Normal *n;
        const int size = normals->size();
        output_cloud->reserve(size);
        output_normal->reserve(size);
        for (size_t i = 0; i < size; ++i) {
            n = &normals->points.at(i);

            if (isnan(n->normal_x) || isnan(n->normal_y) || isnan(n->normal_z)) {
                continue;
            }

            len = sqrtf(n->normal_y * n->normal_y + n->normal_z * n->normal_z);
            theta = atan2f(len, n->normal_x) * 180 / (float) M_PI;
            theta = theta > 90 ? theta - 180 : theta;

            // Filter points with X-axis angle greater than set value theta
            if (fabsf(theta) < angle_tolerance) {
                ++available_count;
                output_normal->push_back(normals->points.at(i), normals->image_indices.at(i));
                output_cloud->push_back(cloud->points.at(i), cloud->image_indices.at(i));
            }
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t filter_pointcloud_by_normal_time = finish - start;
        LOG(INFO) << "after filter by normal: input_cloud.size=" << normals->size()
                  << " -> output_cloud.size=" << available_count
                  << " time consuming " << filter_pointcloud_by_normal_time << "ms";
    }

    void
    Detector::normalFilter(const NormalCloudConstPtr &normals,
                           const PointCloudConstPtr &cloud,
                           const float &angle_tolerance,
                           const std::string axle_name,
                           PointCloudPtr &output_cloud,
                           NormalCloudPtr &output_normal) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        float len, theta;
        int available_count = 0;
        const Normal *n;
        const int size = normals->size();
        output_cloud->reserve(size);
        output_normal->reserve(size);

        if (axle_name == "x") {
            for (size_t i = 0; i < size; ++i) {
                n = &normals->points.at(i);

                if (isnan(n->normal_x) || isnan(n->normal_y) || isnan(n->normal_z)) {
                    continue;
                }

                len = sqrtf(n->normal_y * n->normal_y + n->normal_z * n->normal_z);
                theta = atan2f(len, n->normal_x) * 180 / (float) M_PI;
                theta = theta > 90 ? theta - 180 : theta;

                // Filter points with X-axis angle greater than set value theta
                if (fabsf(theta) < angle_tolerance) {
                    ++available_count;
                    output_normal->push_back(normals->points.at(i), normals->image_indices.at(i));
                    output_cloud->push_back(cloud->points.at(i), cloud->image_indices.at(i));
                }
            }
        } else if (axle_name == "y") {
            for (size_t i = 0; i < size; ++i) {
                n = &normals->points.at(i);

                if (isnan(n->normal_x) || isnan(n->normal_y) || isnan(n->normal_z)) {
                    continue;
                }

                len = sqrtf(n->normal_x * n->normal_x + n->normal_z * n->normal_z);
                theta = atan2f(len, n->normal_y) * 180 / (float) M_PI;
                theta = theta > 90 ? theta - 180 : theta;

                // Filter points with Y-axis angle greater than set value theta
                if (fabsf(theta) < angle_tolerance) {
                    ++available_count;
                    output_normal->push_back(normals->points.at(i), normals->image_indices.at(i));
                    output_cloud->push_back(cloud->points.at(i), cloud->image_indices.at(i));
                }
            }
        } else if (axle_name == "z") {
            for (size_t i = 0; i < size; ++i) {
                n = &normals->points.at(i);

                if (isnan(n->normal_x) || isnan(n->normal_y) || isnan(n->normal_z)) {
                    continue;
                }

                len = sqrtf(n->normal_x * n->normal_x + n->normal_y * n->normal_y);
                theta = atan2f(len, n->normal_z) * 180 / (float) M_PI;
                theta = theta > 90 ? theta - 180 : theta;

                // Filter points with Z-axis angle greater than set value theta
                if (fabsf(theta) < angle_tolerance) {
                    ++available_count;
                    output_normal->push_back(normals->points.at(i), normals->image_indices.at(i));
                    output_cloud->push_back(cloud->points.at(i), cloud->image_indices.at(i));
                }
            }
        } else {
            for (size_t i = 0; i < size; ++i) {
                n = &normals->points.at(i);

                if (isnan(n->normal_x) || isnan(n->normal_y) || isnan(n->normal_z)) {
                    continue;
                }

                len = sqrtf(n->normal_y * n->normal_y + n->normal_z * n->normal_z);
                theta = atan2f(len, n->normal_x) * 180 / (float) M_PI;
                theta = theta > 90 ? theta - 180 : theta;

                // Filter points with X-axis angle greater than set value theta
                if (fabsf(theta) < angle_tolerance) {
                    ++available_count;
                    output_normal->push_back(normals->points.at(i), normals->image_indices.at(i));
                    output_cloud->push_back(cloud->points.at(i), cloud->image_indices.at(i));
                }
            }
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t filter_pointcloud_by_normal_time = finish - start;
        LOG(INFO) << "after filter by normal: input_cloud.size=" << normals->size()
                  << " -> output_cloud.size=" << available_count
                  << " time consuming " << filter_pointcloud_by_normal_time << "ms";
    }

    void
    Detector::calculateNormalEstimation(const PointCloudConstPtr &cloud,
                                        NormalCloudPtr &normals) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        KdTree3dPtr kd_tree = std::make_shared<KdTree3d>();
        kd_tree->build(cloud->points);

        NormalEstimation ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(kd_tree);
        ne.setRadiusSearch(0.03);
        ne.compute(*normals);

        finish = common_func::get_timestamp_in_ms();
        int64_t normal_estimation_time = finish - start;
        LOG(INFO) << "calculate normal: normals.size= " << cloud->size()
                  << " time consuming " << normal_estimation_time << "ms";
    }

    void Detector::voxelGridFilter(const PointCloudConstPtr &cloud,
                                   const PointCloudPtr &output_cloud,
                                   const Eigen::Vector3f &voxel_range){
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();
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
        finish = common_func::get_timestamp_in_ms();
        int64_t voxelgrid_filter_time = finish - start;

        LOG(INFO) << "after voxelgrid filter: input_point.size=" << size
                  << " -> output_cloud.size=" << output_cloud->size()
                  << " time consuming " << voxelgrid_filter_time << "ms";
    }


    void
    Detector::passThroughFilter(const Range3D<float> &range,
                                const PointCloudConstPtr &cloud,
                                const PointCloudPtr &output_cloud) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

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
                output_cloud->push_back(*point, cloud->image_indices.at(i));
            }
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t passthrough_filter_time = finish - start;

        LOG(INFO) << "after passthrough filter: input_point.size=" << size
                  << " -> output_cloud.size=" << output_cloud->size()
                  << " time consuming " << passthrough_filter_time << "ms";
    }

    void
    Detector::meanFilter(const PointCloudConstPtr &cloud,
                         const PointCloudConstPtr &xyz_cloud,
                         const PointCloudPtr &output_cloud) const {
        if (cloud->empty() || xyz_cloud->empty()) return;

        auto get_median_point = [&](size_t r, size_t c, size_t kernel_size) -> Point3D {
            size_t range = kernel_size / 2;
            size_t index, count;
            const Point3D *point;
            std::vector<float> x_vector, y_vector, z_vector;
            x_vector.resize(kernel_size * kernel_size);
            y_vector.resize(kernel_size * kernel_size);
            z_vector.resize(kernel_size * kernel_size);

            // collect the surrounding points.
            count = 0;
            for (size_t i = r - range; i <= r + range; ++i) {
                for (size_t j = c - range; j <= c + range; ++j) {
                    index = i * this->image_width_ + j;
                    point = &cloud->points[index];
                    if (!isnan(point->x) && !isnan(point->y) && !isnan(point->z)) {
                        x_vector[count] = point->x;
                        y_vector[count] = point->y;
                        z_vector[count] = point->z;
                        count++;
                    }
                }
            }

            // find the median.
            const size_t mid = count / 2;
            std::nth_element(x_vector.begin(), x_vector.begin() + mid, x_vector.begin() + count);
            std::nth_element(y_vector.begin(), y_vector.begin() + mid, y_vector.begin() + count);
            std::nth_element(z_vector.begin(), z_vector.begin() + mid, z_vector.begin() + count);

            return Point3D(x_vector[mid], y_vector[mid], z_vector[mid]);
        };

        // found median-point around the current point.
        copyPointCloud(xyz_cloud, output_cloud);
        size_t size = xyz_cloud->size();
        size_t r, c;
        for (size_t i = 0; i < size; ++i) {
            r = xyz_cloud->image_indices[i] / this->image_width_;
            c = xyz_cloud->image_indices[i] % this->image_width_;
            if ((r - 1) > 0 && (r + 1) < this->image_height_
                && (c - 1) > 0 && (c + 1) < this->image_width_) {
                output_cloud->points[i] = get_median_point(r, c, 3);
            }
        }
    }

    void
    Detector::medianFilter(const PointCloudConstPtr &cloud,
                           const PointCloudConstPtr &selected_cloud,
                           const PointCloudPtr &output_cloud) const {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        if (cloud->empty() || selected_cloud->empty()) return;

        auto get_mean_point = [&](size_t r, size_t c, size_t kernel_size) -> Point3D {
            size_t range = kernel_size / 2;
            size_t index, count;
            const Point3D *point;
            float x_total, y_total, z_total;

            // collect the surrounding points.
            count = 0;
            x_total = y_total = z_total = .0f;
            for (size_t i = r - range; i <= r + range; ++i) {
                for (size_t j = c - range; j <= c + range; ++j) {
                    index = i * this->image_width_ + j;
                    point = &cloud->points[index];
                    if (fabsf(point->x) < EPSILON && fabsf(point->y) < EPSILON &&
                        fabsf(point->z) < EPSILON) {
                        continue;
                    }

                    if (isnan(point->x) || isnan(point->y) || isnan(point->z)) {
                        continue;
                    }
                    x_total += point->x;
                    y_total += point->y;
                    z_total += point->z;
                    count++;
                }
            }

            return Point3D(x_total / count, y_total / count, z_total / count);
        };

        // found median-point around the current point.
        copyPointCloud(selected_cloud, output_cloud);

        size_t size = selected_cloud->size();
        int r, c;
        Point3D mean_point{};
        for (size_t i = 0; i < size; ++i) {
            r = selected_cloud->image_indices[i] / this->image_width_;
            c = selected_cloud->image_indices[i] % this->image_width_;
            if ((r - 1) > 0 && (r + 1) < this->image_height_
                && (c - 1) > 0 && (c + 1) < this->image_width_) {
                mean_point = get_mean_point(r, c, 3);
                if (isnan(mean_point.x) || isnan(mean_point.y) || isnan(mean_point.z)) {
                    continue;
                }
                output_cloud->points[i] = mean_point;
            }
        }


        finish = common_func::get_timestamp_in_ms();
        int64_t calculate_mean_time = finish - start;
        LOG(INFO) << "calculate mean: input_cloud.size=" << selected_cloud->size() << " -> output_cloud.size="
                  << output_cloud->points.size() << " time consuming " << calculate_mean_time << "ms";
    }

    void
    Detector::outlierFilter(const PointCloudConstPtr &cloud,
                            const PointCloudPtr &output_cloud) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

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

        finish = common_func::get_timestamp_in_ms();
        int64_t outlier_filter_time = finish - start;

        LOG(INFO) << "after outlier filter: input_point.size=" << cloud->size()
                  << " -> output_cloud.size=" << output_cloud->size() << " time consuming "
                  << outlier_filter_time << "ms";
    }

    void
    Detector::findMaxConnectedComponent(const cv::Mat &binImg,
                                        cv::Rect &card_rect) {
        /*
         * connected component analysis(4-component) use seed filling algorithm.
         * 1. begin with a foreground pixel and push its foreground neighbors into a queue.
         * 2. pop the pop pixel on the stack and label it with the same label until the queue is empty.
         *
         *  foreground pixel: binImg(x,y) = 1
         *  background pixel: binImg(x,y) = 0
         */

        if (binImg.empty() ||
            binImg.type() != CV_8UC1) {
            return;
        }

        int rows = binImg.rows;
        int cols = binImg.cols;
        cv::Mat img = binImg.clone();
        int min_x, min_y, max_x, max_y, pixel_count, max_pixel_count;
        max_pixel_count = std::numeric_limits<int>::min();

        // image coordinate  ———> x(c) .
        for (int c = 0; c < cols; ++c) {
            for (int r = 0; r < rows; ++r) {
                if (img.at<uchar>(r, c) == 0) continue;

                img.at<uchar>(r, c) = 0;
                std::queue <std::pair<int, int>> pixel_queue;
                pixel_queue.push(std::pair<int, int>(r, c));
                pixel_count = 0;
                min_x = INT_MAX;
                min_y = INT_MAX;
                max_x = INT_MIN;
                max_y = INT_MIN;

                while (!pixel_queue.empty()) {

//                cv::imshow("img", img);
//                cv::waitKey(10);

                    std::pair<int, int> curPixel = pixel_queue.front();
                    pixel_queue.pop();
                    pixel_count++;

                    int curX = curPixel.second;  // image-X = pixel_col .
                    int curY = curPixel.first;   // image-Y = pixel_row .

                    min_x = MIN(curX, min_x);
                    min_y = MIN(curY, min_y);
                    max_x = MAX(curX, max_x);
                    max_y = MAX(curY, max_y);

                    auto visit_pixel = [&](const int &x, const int &y) {
                        if (0 == img.at<uchar>(y, x)) return;
                        if (y >= 0 && y < rows && x >= 0 && x < cols) {
                            pixel_queue.push(std::pair<int, int>(y, x));
                            img.at<uchar>(y, x) = 0;
                        }
                    };

                    //push the 4-neighbors(foreground pixels) .
                    visit_pixel(curX, curY - 1); // up .
                    visit_pixel(curX, curY + 1); // down .
                    visit_pixel(curX - 1, curY); // left .
                    visit_pixel(curX + 1, curY); // right .
                }

                if (pixel_count > max_pixel_count) {
                    LOG(INFO) << "pixel_count=" << pixel_count << " > max_pixel_count=" << max_pixel_count
                              << " UPDATE max_pixel_count=" << pixel_count;
                    max_pixel_count = pixel_count;

                    card_rect.x = min_x < 0 ? 0 : min_x;
                    card_rect.y = min_y < 0 ? 0 : min_y;
                    card_rect.height = max_y > rows ? rows : max_y - min_y + 1;
                    card_rect.width = max_x > cols ? cols : max_x - min_x + 1;
                } else {
                    LOG(INFO) << "pixel_count=" << pixel_count << " < max_pixel_count=" << max_pixel_count
                              << " Drop";
                }
            }
        }
    }

    void
    Detector::ransac(const PointCloudConstPtr &cloud,
                     const NormalCloudConstPtr &normals,
                     std::vector<int> &plane_indices,
                     std::vector<float> &coefficients_plane) {

        // Create the segmentation object for the planar model and set all the parameters .
        FittingPlane fit_plane;

        // Set surface normal weight coefficient .
        fit_plane.setNormalThreshold(0.30);

        // Set the maximum allowable distance from the interior point to the model .
        fit_plane.setDistanceThreshold(0.05);
        fit_plane.setMaxIterations(20);
        fit_plane.setPointRate(0.8);
        fit_plane.setInputCloud(cloud);
        fit_plane.setInputNormals(normals);

        // Obtain the plane inliers and coefficients .
        fit_plane.fitting(plane_indices, coefficients_plane);
    }

} // end of namespace perception
