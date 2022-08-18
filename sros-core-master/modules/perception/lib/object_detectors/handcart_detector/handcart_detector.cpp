/**
 * @file handcarts_detector.cpp
 * @brief main functions of handcart detection
 *
 * main functions of handcart detection.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/31
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
//INCLUDE
#include "handcart_detector.h"
#include "../../../common/math_unit.hpp"
#include <queue>
#include <iomanip>
#include <opencv2/highgui.hpp>

//CODE
namespace perception {

HandcartDetector::~HandcartDetector() {
    LOG(INFO) << "Handcarts detector was destroyed!";
}

bool
HandcartDetector::init(const HandcartDetectParam &param,
                   const std::map<int, Handcart> &handcarts) {

    if (handcarts.empty()){
        LOG(INFO) << "handcards map is empty!";
        return false;
    }

    this->param_ = param;
    this->handcarts_.insert(handcarts.begin(), handcarts.end());
    this->consuming_time_recorder_.clear();

    showParam();

    return true;
}

void
HandcartDetector::showParam() {
    std::stringstream string_buff;
    string_buff << std::endl;
    string_buff << "min_x                           = " << param_.detect_range.min_x << std::endl;
    string_buff << "max_x                           = " << param_.detect_range.max_x << std::endl;
    string_buff << "min_y                           = " << param_.detect_range.min_y << std::endl;
    string_buff << "max_y                           = " << param_.detect_range.max_y << std::endl;
    string_buff << "min_z                           = " << param_.detect_range.min_z << std::endl;
    string_buff << "max_z                           = " << param_.detect_range.max_z << std::endl;
    string_buff << "min_cluster_size                = " << param_.min_cluster_size << std::endl;
    string_buff << "max_cluster_size                = " << param_.max_cluster_size << std::endl;
    string_buff << "point_dist_tolerance            = " << param_.point_dist_tolerance << std::endl;
    string_buff << "rate_threshold                  = " << param_.rate_threshold << std::endl;
    string_buff << "normal_angle_tolerance          = " << param_.normal_angle_tolerance << std::endl;
    string_buff << "across_dist_allow               = " << param_.across_dist_allow << std::endl;
    LOG(INFO) << string_buff.str();

    string_buff.str("");
    string_buff.clear();
    string_buff <<"handcarts:" << std::endl;
    for (const auto &handcart : handcarts_) {
        string_buff << "id = " << handcart.second.getId() << std::endl;
        string_buff << " length = " << handcart.second.getLength() << std::endl;
        string_buff << " across_height = " << handcart.second.getAcrossHeight() << std::endl;
        string_buff << " width = " << handcart.second.getWidth() << std::endl;
    }
    LOG(INFO) << string_buff.str();
}

void
HandcartDetector::reset() {
    this->tf_frame_.reset();

    this->mask_img_.setTo(0);
    this->filter_mask_img_.setTo(0);
    this->deep_img_.setTo(0);
    this->amplitude_normalize_.setTo(0);
    this->confidence_normalize_.setTo(0);
    this->camera_img_.setTo(0);
    this->target_img_.setTo(0);

    this->result_.reset();
    this->target_.reset();

    this->filter_mask_cloud_->clear();
    this->filter_xyz_range_cloud_->clear();
    this->filter_outlier_cloud_->clear();
    this->filter_normal_point_cloud_->clear();
    this->filter_normal_normal_cloud_->clear();
    this->target_cloud_->clear();
    this->clusters_cloud_->clear();
    this->consuming_time_recorder_.clear();
}

void
HandcartDetector::filterClusters(const NormalCloudConstPtr &normals,
                                 const PointCloudConstPtr &cloud,
                                 const std::vector<Indices> &clusters_indices,
                                 std::map<int, Cluster> &clusters_map) {
    int clusterIndex = 0;
    std::priority_queue<Cluster, std::vector<Cluster>, std::less<Cluster> > mat_queue;

    if (clusters_indices.empty()){
        LOG(INFO) << "filter clusters: clusters.size=0 time consuming 0ms";
        return;
    }

    LOG(INFO) << "filter clusters:";
    for (const auto &cluster : clusters_indices) {
        LOG(INFO) << "cluster index=" << clusterIndex << " size = " << clusters_indices.size();

        // Histogram statistics, find the boundary in Y direction.
        int *count_arr = new int[this->image_width_]{0};
        int left_index = this->image_width_, right_index = 0;
        for (auto idx : cluster.indices) {
            size_t index = cloud->image_indices[idx] % this->image_width_;
            count_arr[index]++;
            left_index = MIN(left_index, index);
            right_index = MAX(right_index, index);
        }

        // Find the position of the peak up and the peak down of the histogram.
        // method 1：Start from both sides and record the first position greater than the mean
        int i, diff, max, min, up_index, down_index;
//        int avg = static_cast<int>(cluster.indices.size() / (right_index - left_index + 2));
//        i = left_index;
//        while (i <= right_index && count_arr[i] < avg) { i++; }
//        up_index = i;
//
//        i = right_index;
//        while (i >= left_index && count_arr[i] < avg) { i--; }
//        down_index = i;

        up_index = left_index;
        down_index = right_index;

        // log
        LOG(INFO) << " peak range: left=" << left_index << " up_index=" << up_index << " down=" << down_index
                  << " right=" << right_index;
        std::stringstream string_buff;
        string_buff << " count:";
        for (i = left_index; i <= right_index; ++i) {
            if (i == up_index)
                string_buff << " [" << count_arr[i];
            else if (i == down_index)
                string_buff << " " << count_arr[i] << "]";
            else
                string_buff << " " << count_arr[i];
        }
        LOG(INFO) << string_buff.str();

        delete[]count_arr;

        // Keep the point cloud between the peak rising and falling .
        NormalCloudPtr cluster_normals(new NormalCloud);
        PointCloudPtr cluster_cloud(new PointCloud);
        int row, col;
        cv::Mat cluster_img = cv::Mat::zeros(this->image_height_, this->image_width_, CV_8UC1);
        for (auto idx : cluster.indices) {
            const int p_index = cloud->image_indices[idx];
            row = p_index / this->image_width_;
            col = p_index % this->image_width_;
            if (col >= up_index && col <= down_index) {
                cluster_img.at<uint8_t>(row, col) = 255;
                cluster_cloud->push_back(cloud->points[idx], p_index);
                cluster_normals->push_back(normals->points[idx], p_index);
            }
        }

        // Calculation of normal vector of mat surface .
        Normal cluster_normal;
        if (!cluster_normals->empty()) {
            // Method 1: calculate the mean value of normal vector of point cloud as
            // the normal vector of pier surface
            cluster_normal.normal_x = cluster_normal.normal_y = cluster_normal.normal_z = 0;
            for (auto n : cluster_normals->points) {
                cluster_normal.normal_x += n.normal_x;
                cluster_normal.normal_y += n.normal_y;
                cluster_normal.normal_z += n.normal_z;
            }
            cluster_normal.normal_x = cluster_normal.normal_x / cluster_normals->size();
            cluster_normal.normal_y = cluster_normal.normal_y / cluster_normals->size();
            cluster_normal.normal_z = cluster_normal.normal_z / cluster_normals->size();

            LOG(INFO) << " cluster normal: (" << cluster_normal.normal_x << ", " << cluster_normal.normal_y
                      << ", " << cluster_normal.normal_z << ")";
        } else {
            LOG(INFO) << "cluster normal.size=0";
        }

        // Store the deburred point cloud.
        Cluster across;
        if (cluster_normals->size() > this->param_.min_cluster_size) {
            across.setPointCloud(cluster_cloud, this->image_width_);
            across.setNormal(cluster_normal);
            mat_queue.push(across);

            LOG(INFO) << " after remove burrs: input_point_size=" << std::setfill(' ') << std::setw(3)
                      << cluster.indices.size() << " -> output_point_size=" << std::setfill(' ') << std::setw(3)
                      << cluster_normals->size() << " > " << this->param_.min_cluster_size << " OK";
        } else {
            LOG(INFO) << " after remove burrs: input_point_size=" << std::setfill(' ') << std::setw(3)
                      << cluster.indices.size() << " -> output_point_size=" << std::setfill(' ') << std::setw(3)
                      << cluster_normals->size() << " < " << this->param_.min_cluster_size << " Drop";
        }

        clusterIndex++;
    }

    LOG(INFO) << "after sort by Y-axis";

    clusters_map.clear();
    clusterIndex = 0;
    while (!mat_queue.empty()) {
        clusters_map.insert(std::make_pair(clusterIndex, mat_queue.top()));
        mat_queue.pop();
        clusterIndex++;
    }

//    cv::Mat img = cv::Mat::zeros(this->image_height_, this->image_width_, CV_8UC1);
    for (size_t i = 0; i < clusters_map.size(); ++i) {
        cv::Rect rect = clusters_map[i].getRect();
        LOG(INFO) << " cluster " << i << " size=" << std::setfill(' ') << std::setw(3) << clusters_map[i].getSize()
                  << " rect: [" << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << "]";
//        cv::rectangle(img, rect, cv::Scalar(255-i*20));
//        cv::imshow("img", img);
//        cv::waitKey(0);
    }
}

bool
HandcartDetector::searchTarget() {
    int64_t start, finish;
    start = common_func::get_timestamp_in_ms();

    size_t mat_num = this->clusters_map_.size();
    if (this->clusters_map_.empty()) {
        LOG(INFO) << "search target: clusters.size=0 time consuming 0ms";
        return false;
    }

    // find max cluster.
    std::stringstream string_buff;
    int max_point_size = -1;
    int max_point_index = 0;
    string_buff << "cluster size list: [";
    for (int i = 0; i < mat_num; ++i) {
        string_buff << std::fixed << std::setw(5) << std::setprecision(3)
                    << this->clusters_map_[i].getSize() << " ";
        int cluster_point_size = this->clusters_map_[i].getSize();
        if (cluster_point_size > max_point_size) {
            max_point_size = this->clusters_map_[i].getSize();
            max_point_index = i;
        }
    }
    string_buff << "] max: " << max_point_size << std::endl;
    LOG(INFO) << string_buff.str();

    // the biggest cluster is the target.
    Cluster *select = &this->clusters_map_[max_point_index];

    // search for handcart based on target size.
    bool is_find = false;
    size_t handcart_num = this->handcarts_.size();
    HandcartTarget target;
    for (auto const &handcart : handcarts_) {
        float measure_width = select->getWidth();
        float real_width = handcart.second.getWidth();
        float distance = fabsf(measure_width - real_width);
        // check handcart across length.
        if ( distance < param_.across_dist_allow) {
            target_.setId(handcart.first);
            target_.setRect(select->getRect());
            target_.setNormal(select->getNormal());
            target_.setNormalCloud(select->getNormalCloud());
            target_.setPointCloud(select->getPointCloud());
            Point3D pose;
            pose.x = select->getAvgX();
            pose.y = (select->getMostLeftPoint().y + select->getMostRightPoint().y) / 2;
            pose.z = select->getAvgZ();
            target_.setPose(std::move(pose));
//            target_.setPose(Point3D(select->getAvgX(), select->getAvgY(), select->getAvgZ()));
            is_find = true;
            LOG(INFO) << "across length = " << distance << " < " << param_.across_dist_allow << " Ok";
        } else {
            LOG(INFO) << "across length = " << distance << " > " << param_.across_dist_allow << " Drop";
        }
    }

    finish = common_func::get_timestamp_in_ms();
    int64_t search_target_time = finish - start;
    LOG(INFO) << "search target time consuming " << search_target_time << "ms";
    return is_find;
}

void
HandcartDetector::getGenerateCloud(const cv::Mat &mask,
                                   const PointCloudConstPtr &cloud,
                                   const PointCloudPtr &mask_cloud,
                                   const PointCloudPtr &front_cloud) {
    if (cloud->empty()){
        LOG(INFO) << "generate cloud: input_cloud.size=0 time consuming 0ms";
        return;
    }
    const int SIZE = image_height_ * image_width_;
    mask_cloud->resize(SIZE);
    front_cloud->reserve(SIZE);
    for (int r = 0; r < image_height_; ++r) {
        for (int c = 0; c < image_width_; ++c) {
            int idx = r * image_width_ + c;
            mask_cloud->image_indices[idx] = idx;
//            if(INT_MAX != cube_->front_index_map_[idx].min_x) {
            if (mask.at<uint8_t>(r, c) > 0) {
                size_t idx_in_cloud = cube_->front_index_map_[idx].idx_in_cloud;
                mask_cloud->points[idx] = cloud->points[idx_in_cloud];
                front_cloud->push_back(cloud->points[idx_in_cloud], idx);
            }
        }
    }

    LOG(INFO) << "generate cloud: input_cloud.size=" << cloud->points.size() << " -> mask_cloud.size="
              << mask_cloud->points.size() << " -> front_cloud.size=" << front_cloud->size();
}

bool
HandcartDetector::calculateResult() {

    int64_t start, finish;
    start = common_func::get_timestamp_in_ms();

    Normal normal = this->target_.getNormal();
    auto a = -normal.normal_x;      /// TODO: 为什么要加一个负号反向,原因未知
    // auto a = normal.normal_x;
    auto b = normal.normal_y;
    auto c = normal.normal_z;

    auto angle = static_cast<float>(atan2(-b, -a));
    this->target_.setAngle(angle);

    LOG(INFO) << "plane equation: ax+by+cz+d=0" << " (a=" << a << " b=" << b << " c=" << c << ")";
    LOG(INFO) << "angle = atan2(b,a)*180/M_PI = " << angle * 180 / M_PI;

    // Normal vector angle of filter handcart .
    if (fabsf(a) < fabsf(b) || fabsf(a) < fabsf(c)) {
        this->target_.reset();
        LOG(INFO) << "target normal: fabsf(a) < fabsf(b) || fabsf(a) < fabsf(c) Drop.";
    }

    finish = common_func::get_timestamp_in_ms();
    int64_t ransac_time = finish - start;
    LOG(INFO) << "calculate result: cloud_.size=" << this->target_.getPointCloud()->size()
              << " time consuming " << ransac_time << "ms";

    return true;
}

void
HandcartDetector::outlierFilter(const PointCloudConstPtr &cloud,
                                const PointCloudConstPtr &selected_cloud,
                                const PointCloudPtr &output_cloud,
                                cv::Mat &mask) {
    int64_t start, finish;
    start = common_func::get_timestamp_in_ms();

    if (cloud->empty() || selected_cloud->empty()) {
        LOG(INFO) << "outlier filter: cloud->point.size()=0 time consuming 0ms";
        return;
    }

    // collect the surrounding points.
    auto get_kernel_count = [&](const Point3D &base, size_t r, size_t c, size_t kernel_size) -> int {
        size_t range = kernel_size / 2;
        int count = 0;
        for (size_t i = r - range; i <= r + range; ++i) {
            for (size_t j = c - range; j <= c + range; ++j) {
                int index = i * image_width_ + j;
                float dist = distanceBothPoint3d(base, cloud->points[index]);
                if (dist < 0.02 && mask.at<uint8_t>(i,j) > 0){
                    ++count;
                }
            }
        }
        return count;
    };

    // found median-point around the current point.
    copyPointCloud(selected_cloud, output_cloud);

    size_t size = selected_cloud->size();
    int r, c;
    for (size_t i = 0; i < size; ++i) {
        r = selected_cloud->image_indices[i] / this->image_width_;
        c = selected_cloud->image_indices[i] % this->image_width_;
        if ((r - 1) > 0 && (r + 1) < this->image_height_
            && (c - 1) > 0 && (c + 1) < this->image_width_) {
            int count = get_kernel_count(selected_cloud->points[i], r, c, 3);
//            LOG(INFO) << "i=" << i << " count=" << count;
            if (count < 5){
                output_cloud->points[i].x = 0;
                output_cloud->points[i].y = 0;
                output_cloud->points[i].z = 0;
            }
        }
    }

    finish = common_func::get_timestamp_in_ms();
    int64_t outlier_filter_time = finish - start;
    LOG(INFO) << "outlier filter: cloud->point.size()=" << selected_cloud->size() << " -> output_cloud.size="
              << output_cloud->points.size() << " time consuming " << outlier_filter_time << "ms";
}

DetectResult
HandcartDetector::detect(const PointCloudPtr &cloud) {
    int64_t start, finish;
    bool is_find_target;

    // Clear the data stored in the last processing.
    start = common_func::get_timestamp_in_ms();
    this->reset();
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    if (handcarts_.empty()){
        LOG(INFO) << "detection target is empty";
        return this->result_;
    }

    if (cloud->empty()){
        LOG(INFO) << "receive empty cloud";
        return this->result_;
    }

    // Copy data for processing.
    start = common_func::get_timestamp_in_ms();
    PointCloudPtr cloud_ptr(new PointCloud());
    copyPointCloud(cloud, cloud_ptr);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Filter out the points in the detection area.
    start = common_func::get_timestamp_in_ms();
    passThroughFilter(this->param_.detect_range, cloud_ptr, this->filter_xyz_range_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Gridding of point clouds.
    cube_ = std::make_shared<Cube>(param_.detect_range, 0.005, 0.005, 0.005);
    cube_->init(this->filter_xyz_range_cloud_, this->image_height_, this->image_width_);
    bool ret = cube_->verticalFilterByFirstPeak(handcarts_[1].getAcrossHeight(), param_.rate_threshold, this->mask_img_);
    if (ret) {
        filter_front_cloud_ = std::make_shared<PointCloud>();
        getGenerateCloud(this->mask_img_, this->filter_xyz_range_cloud_, this->filter_mask_cloud_,
                         this->filter_front_cloud_);

        // Median filtering
        start = common_func::get_timestamp_in_ms();
        medianFilter(this->filter_mask_cloud_, this->filter_front_cloud_, this->filter_median_cloud_);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Filtering outliers.
        start = common_func::get_timestamp_in_ms();
        outlierFilter(this->filter_mask_cloud_, this->filter_median_cloud_,
                      this->filter_outlier_cloud_, this->mask_img_);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Calculating the normal vector of point cloud.
        start = common_func::get_timestamp_in_ms();
        NormalCloud::Ptr normals(new NormalCloud());
        calculateNormalEstimation(this->filter_outlier_cloud_, normals);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Filter out the point cloud toward the camera.
        start = common_func::get_timestamp_in_ms();
        normalFilter(normals,
                     this->filter_outlier_cloud_,
                     this->param_.normal_angle_tolerance,
                     this->filter_normal_point_cloud_,
                     this->filter_normal_normal_cloud_);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Point cloud segmentation, generating pier list.
        start = common_func::get_timestamp_in_ms();
        std::vector<Indices> clusters_indices;
        euclidean<HandcartDetectParam>(this->param_,
                                       this->filter_normal_normal_cloud_,
                                       this->filter_normal_point_cloud_,
                                       clusters_indices);
        filterClusters(this->filter_normal_normal_cloud_,
                       this->filter_normal_point_cloud_,
                       clusters_indices,
                       this->clusters_map_);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Search card target.
        start = common_func::get_timestamp_in_ms();
        is_find_target = searchTarget();
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));
    } else {
        LOG(INFO) << "didn't find sudden change peak.";
    }

    if (is_find_target) {
            // Calculate target position and angle.
            start = common_func::get_timestamp_in_ms();
            this->result_.is_available = calculateResult();
            finish = common_func::get_timestamp_in_ms();
            this->consuming_time_recorder_.push_back(std::to_string(finish - start));

            this->result_.id = target_.getId();
            this->result_.x = target_.getPose().x;
            this->result_.y = target_.getPose().y;
            this->result_.z = target_.getPose().z;
            // this->result_.angle = target_.getAngle();
            this->result_.angle = -target_.getAngle(); /// TODO: 为什么要加一个负号反向,没弄明白，但好使.
            this->result_.width = target_.getWidth();
            this->result_.height = target_.getHeight();
            this->result_.normal = target_.getNormal();
    } else {
        LOG(INFO) << "didn't find handcart";
    }

    return this->result_;
}

} // end of namespace perception