/**
 * @file cards_detector.cpp
 * @brief main functions of card detection
 *
 * main functions of card detection.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/31
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
//INCLUDE
#include "put_space_detector.h"
#include "../../../common/math_unit.hpp"
#include <queue>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "../../../lib/common/algorithm/machine_vision.hpp"

//CODE
namespace perception {

PutspaceDetector::~PutspaceDetector() {
    LOG(INFO) << "Put space detector was destroyed!";
}

bool
PutspaceDetector::init(const PutspaceDetectParam &param) {

    this->param_ = param;
    this->consuming_time_recorder_.clear();

    showParam();

    return true;
}

void
PutspaceDetector::showParam() {
    std::stringstream string_buff;
    string_buff << std::endl;
    string_buff << "min_x                           = " << param_.detect_range.min_x << std::endl;
    string_buff << "max_x                           = " << param_.detect_range.max_x << std::endl;
    string_buff << "min_y                           = " << param_.detect_range.min_y << std::endl;
    string_buff << "max_y                           = " << param_.detect_range.max_y << std::endl;
    string_buff << "min_z                           = " << param_.detect_range.min_z << std::endl;
    string_buff << "max_z                           = " << param_.detect_range.max_z << std::endl;
//    string_buff << "camera_fixed.x                  = " << param_.camera_fixed[0] << std::endl;
//    string_buff << "camera_fixed.y                  = " << param_.camera_fixed[1] << std::endl;
//    string_buff << "camera_fixed.yaw                = " << param_.camera_fixed[2] << std::endl;
//    string_buff << "laser_fixed.x                   = " << param_.laser_fixed[0] << std::endl;
//    string_buff << "laser_fixed.y                   = " << param_.laser_fixed[1] << std::endl;
//    string_buff << "laser_fixed.yaw                 = " << param_.laser_fixed[2] << std::endl;
//    string_buff << "fork_end_x                      = " << param_.fork_end[0] << std::endl;
//    string_buff << "fork_end_y                      = " << param_.fork_end[1] << std::endl;
//    string_buff << "fork_end_z                      = " << param_.fork_end[2] << std::endl;
    string_buff << "min_cluster_size                = " << param_.min_cluster_size << std::endl;
    string_buff << "max_cluster_size                = " << param_.max_cluster_size << std::endl;
    string_buff << "point_dist_tolerance            = " << param_.point_dist_tolerance << std::endl;
    string_buff << "threshold                       = " << param_.threshold << std::endl;
    string_buff << "normal_angle_tolerance          = " << param_.normal_angle_tolerance << std::endl;
    string_buff << "both_pallet_dist_allow          = " << param_.both_pallet_dist_allow << std::endl;
    string_buff << "up_extend_pixel                 = " << param_.up_extend_pixel << std::endl;
    string_buff << "down_extend_pixel               = " << param_.down_extend_pixel << std::endl;
    string_buff << "min_area_rate                   = " << param_.min_area_rate << std::endl;
    string_buff << "max_area_rate                   = " << param_.max_area_rate << std::endl;
    string_buff << "hole_height_rate_diff_allow     = " << param_.hole_height_rate_diff_allow << std::endl;
    string_buff << "pallet_normal_diff_angle_allow  = " << param_.pallet_normal_diff_angle_allow << std::endl;
    LOG(INFO) << string_buff.str();

}

void
PutspaceDetector::reset() {
    this->roi_rect_.x = 0;
    this->roi_rect_.y = 0;
    this->roi_rect_.width = 0;
    this->roi_rect_.height = 0;

    this->card_rect_.x = 0;
    this->card_rect_.y = 0;
    this->card_rect_.width = 0;
    this->card_rect_.height = 0;

    this->tf_frame_.reset();

    this->mask_img_.setTo(0);
    this->filter_mask_img_.setTo(0);
    this->deep_img_.setTo(0);
    this->amplitude_normalize_.setTo(0);
    this->confidence_normalize_.setTo(0);
    this->camera_img_.setTo(0);
    this->target_img_.setTo(0);
    this->pallet_img_.setTo(0);
    this->hole_img_.setTo(0);

    this->result_.reset();
    this->target_.reset();

    this->filter_mask_cloud_->clear();
    this->filter_xyz_range_cloud_->clear();
    this->filter_median_cloud_->clear();
    this->filter_outlier_cloud_->clear();
    this->filter_normal_point_cloud_->clear();
    this->filter_normal_normal_cloud_->clear();
    this->target_cloud_->clear();
    this->clusters_cloud_->clear();
    this->consuming_time_recorder_.clear();
}



void
PutspaceDetector::getAmplitudeImage(cv::Mat &img) const {
    img = this->amplitude_normalize_.clone();
}

void
PutspaceDetector::getConfidenceImage(cv::Mat &img) const {
    img = this->confidence_normalize_.clone();
}

void
PutspaceDetector::getPalletImage(cv::Mat &img) const {
    img = this->pallet_img_.clone();
}

void
PutspaceDetector::getHoleImage(cv::Mat &img) const {
    img = this->hole_img_.clone();
}

void
PutspaceDetector::getDeepImage(cv::Mat &img) const {
    img = this->deep_img_.clone();
}

void
PutspaceDetector::getXYZThresholdImage(cv::Mat &img) const {
    img = this->xyz_threshold_img_.clone();
}


void
PutspaceDetector::filterClusters(const NormalCloudConstPtr &normals,
                             const PointCloudConstPtr &cloud,
                             const std::vector<Indices> &clusters_indices,
                             std::map<int, Cluster> &clusters_map) {
    int clusterIndex = 0;
    std::priority_queue<Cluster, std::vector<Cluster>, std::less<Cluster> > pallet_queue;

    LOG(INFO) << "cluster size = " << clusters_indices.size();
    for (const auto &cluster : clusters_indices) {
        Cluster pallet;
        cv::Mat cluster_img = cv::Mat::zeros(this->image_height_, this->image_width_, CV_8UC1);
        int row, col;

        LOG(INFO) << "index=" << clusterIndex;

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
        int avg = static_cast<int>(cluster.indices.size() / (right_index - left_index + 2));

        i = left_index;
        while (i <= right_index && count_arr[i] < avg) { i++; }
        up_index = i;

        i = right_index;
        while (i >= left_index && count_arr[i] < avg) { i--; }
        down_index = i;


        // Method 2: calculate the difference between two adjacent statistics and
        // record the position with the largest difference.

//       max = count_arr[left_index];
//       min = -count_arr[right_index];
//       up_index = left_index;
//       down_index = right_index;
//
//       for (int i = left_index+1; i <= right_index; ++i){
//           diff = count_arr[i] - count_arr[i-1];
//           if (diff > max){
//               up_index = i;
//               max = diff;
//           } else if (diff < min){
//               down_index = i-1;
//               min = diff;
//           }
//       }
//
//       if (up_index > down_index){
//           up_index = left_index;
//           down_index = right_index;
//           LOG(INFO) << "find single-peak fail: up_index > down_index";
//       }

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

        // Calculation of normal vector of pallet surface .
        // Method 1: calculate the mean value of normal vector of point cloud as
        // the normal vector of pier surface
        Normal cluster_normal;
        cluster_normal.normal_x = cluster_normal.normal_y = cluster_normal.normal_z = 0;
        for (auto n : cluster_normals->points) {
            cluster_normal.normal_x += n.normal_x;
            cluster_normal.normal_y += n.normal_y;
            cluster_normal.normal_z += n.normal_z;
        }
        cluster_normal.normal_x = cluster_normal.normal_x / cluster_normals->size();
        cluster_normal.normal_y = cluster_normal.normal_y / cluster_normals->size();
        cluster_normal.normal_z = cluster_normal.normal_z / cluster_normals->size();


        // Method 2: fitting the pier surface, calculating the plane normal vector as
        // the normal vector of the pier surface
//            PointIndices::Ptr plane_indices(new PointIndices());
//            ModelCoefficients::Ptr coefficients_plane(new ModelCoefficients);
//            ransac(cluster_cloud, cluster_normals, plane_indices, coefficients_plane);
//            cluster_normal.normal_x = coefficients_plane->values.at(0);
//            cluster_normal.normal_y = coefficients_plane->values.at(1);
//            cluster_normal.normal_z = coefficients_plane->values.at(2);

        LOG(INFO) << " cluster normal: (" << cluster_normal.normal_x << ", " << cluster_normal.normal_y
                  << ", " << cluster_normal.normal_z << ")";

        // Store the deburred point cloud.
        if (cluster_normals->size() > this->param_.min_cluster_size) {
            pallet.setPointCloud(cluster_cloud, this->image_width_);
            pallet.setNormal(cluster_normal);
            pallet_queue.push(pallet);

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
    while (!pallet_queue.empty()) {
        clusters_map.insert(std::make_pair(clusterIndex, pallet_queue.top()));
        pallet_queue.pop();
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


void
PutspaceDetector::findRectangle(const PutspaceDetectParam &param_,
                            const cv::Mat &img_src,
                            std::vector<cv::Rect> &rects) {

    std::vector<cv::Vec4i> hierarchy;
    cv::Point offset = cv::Point();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> hull;
    std::vector<cv::RotatedRect> boxs;
    cv::Mat img_dst;
    cv::Mat kernal;

    img_src.copyTo(img_dst);
    threshold(img_dst, img_dst, param_.threshold, 255, CV_THRESH_BINARY);
    findContours(img_dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    hull.resize(contours.size());
    for (size_t i = 0; i < contours.size(); i++) {
        convexHull(cv::Mat(contours[i]), hull[i], false);
    }

    boxs.resize(hull.size());
    for (size_t i = 0; i < hull.size(); i++) {
        boxs[i] = cv::minAreaRect(cv::Mat(hull[i]));
        hull[i].size();
    }

    cv::Rect rect;
    rects.clear();
    rects.reserve(boxs.size());
    std::vector<float> x_arr(4), y_arr(4);

    for (auto &box : boxs) {
        cv::Point2f vertex[4];
        box.points(vertex);

        for (int j = 0; j < 4; j++) {
            x_arr[j] = vertex[j].x;
            y_arr[j] = vertex[j].y;
        }

        std::sort(x_arr.begin(), x_arr.end(), common_func::compare_xy);
        std::sort(y_arr.begin(), y_arr.end(), common_func::compare_xy);
        rect.x = x_arr[1];
        rect.y = y_arr[1];
        rect.width = abs(x_arr[2] - x_arr[1] + 1);
        rect.height = abs(y_arr[2] - y_arr[1] + 1);
        rects.push_back(rect);
    }
}

void
PutspaceDetector::getGenerateCloud(const cv::Mat &mask,
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

DetectResult
PutspaceDetector::putGoodsSpaceDetect(const PointCloudPtr &cloud) {
    int64_t start, finish;
    bool is_find_target;
    NormalCloud::Ptr normals(new NormalCloud());

    // Clear the data stored in the last processing.
    start = common_func::get_timestamp_in_ms();
    this->reset();
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Copy data for processing.
    start = common_func::get_timestamp_in_ms();
    PointCloudPtr cloud_ptr(new PointCloud());
    copyPointCloud(cloud, cloud_ptr);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));




    return result_;
}

DetectResult
PutspaceDetector::headupGoodsDetect(const PointCloudPtr &cloud){
    this->result_.is_available = false;
    int64_t start, finish;
    bool is_find_target;
    NormalCloud::Ptr normals(new NormalCloud());

    // Clear the data stored in the last processing.
    start = common_func::get_timestamp_in_ms();
    this->reset();
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Copy data for processing.
    start = common_func::get_timestamp_in_ms();
    PointCloudPtr cloud_ptr(new PointCloud());
    copyPointCloud(cloud, cloud_ptr);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Filter out the points in the detection area.
    start = common_func::get_timestamp_in_ms();
    PointCloudPtr temp_cloud(new PointCloud);
    Range3D<float> temp_range = param_.detect_range;
    start = common_func::get_timestamp_in_ms();
    passThroughFilter(temp_range, cloud_ptr, filter_mask_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // According to the region point cloud generated mask image and deep image.
    start = common_func::get_timestamp_in_ms();
    cube_ = std::make_shared<Cube>(temp_range, 0.004, 0.004,0.004);
    cube_->init(filter_mask_cloud_, this->image_height_, this->image_width_);
    cube_->generateMaskImage(mask_img_);
    cube_->generateDeepImage(deep_img_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // calc y and z
    cv::Mat mask_dilate_img, mask_erode_img;
    cv::Mat element_rect5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat element_rect7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

    cv::morphologyEx(mask_img_, mask_dilate_img, cv::MORPH_DILATE, element_rect5x5);
    mvision::fillUp(mask_dilate_img, mask_dilate_img);
    mvision::LargestConnecttedComponent(mask_dilate_img, mask_dilate_img);
    cv::Rect mask_dilate_rect = cv::boundingRect(mask_dilate_img);
    cv::morphologyEx(mask_dilate_img, mask_erode_img, cv::MORPH_ERODE, element_rect7x7);
    cv::Mat mask_sub_img = mask_dilate_img - mask_erode_img;
    cv::Mat mask_and_img = mask_sub_img & mask_img_;

    cv::Point left_rect_tl(mask_dilate_rect.tl().x, mask_dilate_rect.tl().y + mask_dilate_rect.height/6);
    cv::Point left_rect_br(left_rect_tl.x + 10, mask_dilate_rect.br().y - mask_dilate_rect.height/6 );
    cv::Rect mask_left_line_rect(left_rect_tl, left_rect_br);
    cv::Mat mask_left_img = cv::Mat::zeros(this->mask_img_.rows, this->mask_img_.cols, CV_8UC1);
    mask_and_img(mask_left_line_rect).copyTo(mask_left_img(mask_left_line_rect));

    cv::Point right_rect_br(mask_dilate_rect.br().x, mask_dilate_rect.br().y -mask_dilate_rect.height/6 );
    cv::Point right_rect_tl(right_rect_br.x - 10 , mask_dilate_rect.tl().y + mask_dilate_rect.height/6);
    cv::Rect mask_right_line_rect(right_rect_tl, right_rect_br);
    cv::Mat mask_right_img = cv::Mat::zeros(this->mask_img_.rows, this->mask_img_.cols, CV_8UC1);
    mask_and_img(mask_right_line_rect).copyTo(mask_right_img(mask_right_line_rect));

    cv::Point top_rect_tl (mask_dilate_rect.tl().x + 10, mask_dilate_rect.tl().y);
    cv::Point top_rect_br(mask_right_line_rect.br().x -10, mask_dilate_rect.tl().y+10);
    cv::Rect mask_top_line_rect(top_rect_tl, top_rect_br);
    cv::Mat mask_top_img = cv::Mat::zeros(this->mask_img_.rows, this->mask_img_.cols, CV_8UC1);
    mask_and_img(mask_top_line_rect).copyTo(mask_top_img(mask_top_line_rect));

    cv::Point center_rect_tl (mask_dilate_rect.tl().x +  mask_dilate_rect.width/4, mask_dilate_rect.tl().y + mask_dilate_rect.height/4);
    cv::Point center_rect_br (mask_dilate_rect.br().x -  mask_dilate_rect.width/4, mask_dilate_rect.br().y - mask_dilate_rect.height/4);
    cv::Rect mask_center_rect(center_rect_tl, center_rect_br);
    cv::Mat mask_center_img = cv::Mat::zeros(this->mask_img_.rows, this->mask_img_.cols, CV_8UC1);
    mask_img_(mask_center_rect).copyTo(mask_center_img(mask_center_rect));

    PointCloudPtr filter_front_cloud(new PointCloud);
    PointCloudPtr filter_mask_cloud(new PointCloud);
    getGenerateCloud(mask_left_img, filter_mask_cloud_, filter_mask_cloud,
                     filter_front_cloud);
    float left_mean_y = 0 ;
    if(!common_func::calcPointsXYZAvg(filter_front_cloud, "y", left_mean_y)){
        LOG(INFO) << "calc cube y: left line is empty!";
        //return result_;
    }


    // record debug image
    if(param_.enable_record_debug_image){
        if(!mask_img_.empty()) cv::imwrite("/sros/debug_data/0001_mask_img_.jpg",mask_img_);
        if(!deep_img_.empty()) cv::imwrite("/sros/debug_data/0002_deep_img_.jpg",deep_img_);
        if(!mask_and_img.empty()) cv::imwrite("/sros/debug_data/0003_mask_and_img.jpg",mask_and_img);
        if(!mask_left_img.empty()) cv::imwrite("/sros/debug_data/0004_mask_left_img.jpg",mask_left_img);
        if(!mask_right_img.empty()) cv::imwrite("/sros/debug_data/0005_mask_right_img.jpg",mask_right_img);
        if(!mask_top_img.empty()) cv::imwrite("/sros/debug_data/0006_mask_top_img.jpg",mask_top_img);
        if(!mask_center_img.empty()) cv::imwrite("/sros/debug_data/0007_mask_center_img.jpg",mask_center_img);
    }



    filter_mask_cloud.reset(new PointCloud);
    filter_front_cloud.reset(new PointCloud);
    getGenerateCloud(mask_right_img, filter_mask_cloud_, filter_mask_cloud,
                     filter_front_cloud);
    float right_mean_y = 0 ;
    if(!common_func::calcPointsXYZAvg(filter_front_cloud, "y", right_mean_y)){
        LOG(INFO) << "calc cube y: right line is empty!";
        //return result_;
    }

    filter_mask_cloud.reset(new PointCloud);
    filter_front_cloud.reset(new PointCloud);
    getGenerateCloud(mask_top_img, filter_mask_cloud_, filter_mask_cloud,
                     filter_front_cloud);

    float top_mean_z = 0 ;
    if(!common_func::calcPointsXYZAvg(filter_front_cloud, "z", top_mean_z)){
        LOG(INFO) << "calc cube z: top line is empty!";
        return result_;
    }

    LOG(INFO) << "calc cube y: "<< (left_mean_y + right_mean_y)/2.0f << " z:" << top_mean_z;


    // According to mask image generated point cloud.
    start = common_func::get_timestamp_in_ms();
    PointCloudPtr filter_front_cloud_(new PointCloud);

    getGenerateCloud(mask_img_, filter_mask_cloud_, temp_cloud,
                     filter_front_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Filter out the points in the pallets detection area.
    start = common_func::get_timestamp_in_ms();
    passThroughFilter(param_.detect_range, filter_front_cloud_, this->filter_xyz_range_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Median filtering
    start = common_func::get_timestamp_in_ms();
    medianFilter(filter_front_cloud_, this->filter_xyz_range_cloud_, this->filter_median_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Filtering outliers.
    start = common_func::get_timestamp_in_ms();
    outlierFilter(this->filter_median_cloud_, this->filter_outlier_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // voxel filter
    PointCloudPtr filter_voxelgrid_cloud_(new PointCloud());
    start = common_func::get_timestamp_in_ms();
    Eigen::Vector3f  voxel_range(0.02,0.02,0.02);
    Detector::voxelGridFilter(this->filter_outlier_cloud_, filter_voxelgrid_cloud_,voxel_range);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Calculating the normal vector of point cloud.
    start = common_func::get_timestamp_in_ms();
    calculateNormalEstimation(filter_voxelgrid_cloud_, normals);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    LOG(INFO) << "front_normal_angle_tolerance = "<<param_.front_normal_angle_tolerance;
    // Filter out the point cloud toward the camera.
    start = common_func::get_timestamp_in_ms();
    normalFilter(normals,
                 filter_voxelgrid_cloud_,
                 this->param_.front_normal_angle_tolerance,
                 this->filter_normal_point_cloud_,
                 this->filter_normal_normal_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // judget plane point cloud size
    int min_pointcloud_size = param_.min_cloud_size;
    int max_pointcloud_size = param_.max_cloud_size;
    int cloud_size = this->filter_normal_point_cloud_->size();
    LOG(INFO) << "@001 cloud size:"<< cloud_size << " image width:"<< this->image_width_;
    if(cloud_size < min_pointcloud_size || cloud_size > max_pointcloud_size || this->image_width_ <=0){
        LOG(INFO) << "target is not card, cloud size:"<< cloud_size << " image width:"<< this->image_width_;
        return result_;
    }

    Eigen::Vector4f  min_p, max_p;
    getMaxMin(this->filter_normal_point_cloud_, min_p, max_p);

    Cluster front_side;
    front_side.setPointCloud(this->filter_normal_point_cloud_, this->image_width_);
    front_side.setNormalCloud(this->filter_normal_normal_cloud_);

    // fitting plane ax + by + cz + d = 0 .
    std::vector<int> plane_indices;
    std::vector<float> plane_coefficients;
    ransac(front_side.getPointCloud(), front_side.getNormalCloud(),
           plane_indices, plane_coefficients);
    if (plane_coefficients.empty()) {
        LOG(INFO) << "ransac plane_coefficients is empty!";
        return result_;
    }

    // calculate Result
    const float a = plane_coefficients[0];
    const float b = plane_coefficients[1];
    const float c = plane_coefficients[2];
    auto angle = static_cast<float>(atan2(-b, -a));
    LOG(INFO) << "plane equation: ax+by+cz+d=0" << " (a=" << a << " b=" << b << " c=" << c << ")";
    LOG(INFO) << "angle = atan2(b,a)*180/M_PI = " << angle * 180 / M_PI;


    this->result_.is_available = true;
    this->result_.id = 0;
    this->result_.x = front_side.getAvgX();
    //this->result_.y = (left_mean_y + right_mean_y)/2.0f;
    this->result_.y = front_side.getAvgY(); 
    this->result_.z = top_mean_z;
    this->result_.angle = angle;
    this->result_.width = max_p.y() - min_p.y();
    this->result_.height = top_mean_z;
    this->result_.normal = front_side.getNormal();

    LOG(INFO) << "@001 Cube Target x:" << this->result_.x << " y:"<<this->result_.y << " z:" << top_mean_z << " angle:"<<angle * 180 / M_PI
              <<  " width:" << this->result_.width <<  " height:";


    // draw shapes on the picture.
    drawMask();
    drawCameraImage();

    return this->result_;
}


DetectResult
PutspaceDetector::detect(const O3d3xxFrame &frame) {
    return result_;
}

DetectResult
PutspaceDetector::detect(const PointCloudPtr &cloud) {
    return result_;
}

DetectResult
PutspaceDetector::detect(const PointCloudPtr &cloud, int flag){
    // 根据堆叠类型进行分类 目前只有立方体物体堆叠
    if(flag ==0) {
        return headupGoodsDetect(cloud);
    } else if (flag ==1){
        return headupGoodsDetect(cloud);
    } else {
        return headupGoodsDetect(cloud);
    }
    return result_;
}

DetectResult
PutspaceDetector::detect(const O3d3xxFrame &frame, int flat) {
    return result_;
}

bool
PutspaceDetector::searchTarget() {
    return false;
}

bool
PutspaceDetector::calculateResult() {
    return false;
}


} // end of namespace perception