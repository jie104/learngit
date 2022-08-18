/**
 * @file circle_detector.cpp
 * @brief Used to detect circular objects
 *
 * Class for detecting circular targets
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/7
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "circle_detector.h"
#include "../../common/algorithm/fitting_circle.h"
#include "../../../common/math_unit.hpp"

#include <queue>
#include <iomanip>

// CODE
namespace perception {
CircleDetector::CircleDetector() = default;

CircleDetector::~CircleDetector() {
    LOG(INFO) << "Circle detector was destroyed!";
}

bool
CircleDetector::init(const CircleDetectParam &param,
                     const std::map<int, Circle> &circles_map) {

    if (circles_map.empty()) {
        return false;
    }

    this->param_ = param;
    this->circles_map_.insert(circles_map.begin(), circles_map.end());
    this->consuming_time_recorder_.clear();

    showParam();

    return true;
}

void
CircleDetector::showParam() {
    std::stringstream string_buff;
    string_buff << std::endl;
    string_buff << "min_x                   = " << param_.detect_range.min_x << std::endl;
    string_buff << "max_x                   = " << param_.detect_range.max_x << std::endl;
    string_buff << "min_y                   = " << param_.detect_range.min_y << std::endl;
    string_buff << "max_y                   = " << param_.detect_range.max_y << std::endl;
    string_buff << "min_z                   = " << param_.detect_range.min_z << std::endl;
    string_buff << "max_z                   = " << param_.detect_range.max_z << std::endl;
    string_buff << "camera_fixed.x          = " << param_.camera_fixed[0] << std::endl;
    string_buff << "camera_fixed.y          = " << param_.camera_fixed[1] << std::endl;
    string_buff << "camera_fixed.yaw        = " << param_.camera_fixed[2] << std::endl;
    string_buff << "laser_fixed.x           = " << param_.laser_fixed[0] << std::endl;
    string_buff << "laser_fixed.y           = " << param_.laser_fixed[1] << std::endl;
    string_buff << "laser_fixed.yaw         = " << param_.laser_fixed[2] << std::endl;
    string_buff << "fork_end_x              = " << param_.fork_end[0] << std::endl;
    string_buff << "fork_end_y              = " << param_.fork_end[1] << std::endl;
    string_buff << "fork_end_z              = " << param_.fork_end[2] << std::endl;
    string_buff << "min_cluster_size        = " << param_.min_cluster_size << std::endl;
    string_buff << "max_cluster_size        = " << param_.max_cluster_size << std::endl;
    string_buff << "point_dist_tolerance    = " << param_.point_dist_tolerance << std::endl;
    string_buff << "threshold               = " << param_.threshold << std::endl;
    string_buff << "normal_angle_tolerance  = " << param_.normal_angle_tolerance << std::endl;
    string_buff << "max_iteration           = " << param_.max_iteration << std::endl;
    string_buff << "fitting_rate            = " << param_.fitting_rate << std::endl;
    string_buff << "pixel_dist_allow        = " << param_.pixel_dist_allow << std::endl;
    string_buff << "contour_min_size        = " << param_.contour_min_size << std::endl;

    LOG(INFO) << string_buff.str();

    string_buff.str("");
    string_buff.clear();
    string_buff << std::endl;
    for (const auto &circle : circles_map_) {
        string_buff << "circle_id = " << circle.first << " width="<< circle.second.getWidth()
                    << " length=" << circle.second.getLength()<< std::endl;
        string_buff << "  radius = " << circle.second.getRadius() << std::endl;
    }
    LOG(INFO) << string_buff.str();
}

void
CircleDetector::reset() {
    this->image_width_ = 0;
    this->image_width_ = 0;
    this->deep_img_.setTo(0);
    this->filter_mask_img_.setTo(0);
    this->amplitude_normalize_.setTo(0);
    this->confidence_normalize_.setTo(0);
    this->tf_frame_.reset();
    this->mask_img_.setTo(0);
    this->camera_img_.setTo(0);
    this->target_img_.setTo(0);

    this->filter_mask_cloud_->clear();
    this->filter_xyz_range_cloud_->clear();
    this->filter_median_cloud_->clear();
    this->filter_outlier_cloud_->clear();
    this->filter_normal_point_cloud_->clear();
    this->filter_normal_normal_cloud_->clear();
    this->clusters_map_.clear();
    this->result_.reset();
    this->target_cloud_->clear();
    this->clusters_cloud_->clear();
    this->consuming_time_recorder_.clear();

    this->contours_img_.setTo(0);
    this->clusters_img_.setTo(0);
    this->target_.reset();
}

void CircleDetector::drawContour(){
    putText(this->contours_img_, "contours", cv::Point(10, 20),
            CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
            1, 1);
}

void CircleDetector::drawCluster(){
    putText(this->clusters_img_, "clusters", cv::Point(10, 20),
            CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
            1, 1);
}

void
CircleDetector::drawTarget() {
    cv::cvtColor(this->contours_img_, this->target_img_, cv::COLOR_GRAY2BGR);
    putText(this->target_img_, "target", cv::Point(10, 20),
            CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
            1, 1);

    if (!this->result_.is_available) return;

    const cv::Point center = target_.getCenterInImage();
//    LOG(INFO) << "center(" << center.x << "," << center.y << ")";
    const int radius = target_.getRadiusPixel();
    const cv::Scalar color(0,0,255);
    circle(target_img_, center, 3, color, cv::FILLED, cv::LINE_AA);
    circle(target_img_, center, radius, color, 1, cv::LINE_AA);
}

void
CircleDetector::getContourImage(cv::Mat &img) const {
//    img = this->contours_img_.clone();
    cv::cvtColor(this->contours_img_, img, cv::COLOR_GRAY2BGR);
}

void
CircleDetector::getClusterImage(cv::Mat &img) const {
//    img = this->clusters_img_.clone();
    cv::cvtColor(this->clusters_img_, img, cv::COLOR_GRAY2BGR);
}

void
CircleDetector::filterClusters(const NormalCloudConstPtr &input_normal_cloud,
                               const PointCloudConstPtr &input_point_cloud,
                               const std::vector<Indices> &clusters_indices,
                               std::map<int, Cluster> &clusters_map) {
    int cluster_idx = 0;
    for (const auto &cluster_indices : clusters_indices) {

        // Get point cloud data of cluster.
        const int size = cluster_indices.indices.size();
        NormalCloudPtr cluster_normal_cloud(new NormalCloud);
        PointCloudPtr cluster_point_cloud(new PointCloud);
        cluster_point_cloud->reserve(size);
        cluster_normal_cloud->reserve(size);
        for (auto i : cluster_indices.indices) {
            int idx = input_point_cloud->image_indices[i];
            cluster_point_cloud->push_back(input_point_cloud->points[i], idx);
            cluster_normal_cloud->push_back(input_normal_cloud->points[i], idx);
        }

        this->clusters_cloud_->add(*cluster_point_cloud);

        Cluster cluster;
        cluster.setPointCloud(cluster_point_cloud, this->image_width_);
        cluster.setNormalCloud(cluster_normal_cloud);
        clusters_map.insert(std::make_pair(cluster_idx, cluster));
        cluster_idx++;
    }

    LOG(INFO) << "cluster size = " << clusters_map.size();
    for (size_t i = 0; i < clusters_map.size(); ++i) {
        auto rect = clusters_map[i].getRect();
        LOG(INFO) << " cluster " << i << " size=" << std::setfill(' ') << std::setw(3) << clusters_map[i].getSize()
                  << " rect: [" << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << "]";
    }
}

bool
CircleDetector::searchTarget() {
    std::priority_queue<CircleTarget> targets_queue;
    this->clusters_img_ = cv::Mat::zeros(this->image_height_, this->image_width_, CV_8UC1);
    this->contours_img_ = cv::Mat::zeros(this->image_height_, this->image_width_, CV_8UC1);
    for (auto const &cluster: this->clusters_map_) {
        // Generating binary images of clusters.
        auto cloud = cluster.second.getPointCloud();
        cv::Mat cluster_img = cv::Mat::zeros(this->image_height_, this->image_width_, CV_8UC1);
        for (auto idx : cloud->image_indices) {
            int r = idx / this->image_width_;
            int c = idx % this->image_width_;
            cluster_img.at<uint8_t>(r, c) = 255;
        }

        // Generating binary image of cluster contour.
        cv::Mat contour_img;
        std::vector<cv::Point> contour_points;
        findContourPoint(cluster_img, contour_img, contour_points);

        // Search for circles in a profile.
        cv::Point center_pixel;
        int radius_pixel;
        float score = findCircle(contour_points, center_pixel, radius_pixel);

        // Add the search circle to the priority queue.
        CircleTarget circleTarget;
        circleTarget.setContourImage(contour_img);
        circleTarget.setContourPoint(contour_points);
        circleTarget.setScore(score);
        circleTarget.setRadiusPixel(radius_pixel);
        circleTarget.setCenterInImage(center_pixel);
        circleTarget.setPointCloud(cluster.second.getPointCloud());
        circleTarget.setNormalCloud(cluster.second.getNormalCloud());

        targets_queue.push(circleTarget);

        this->contours_img_ |= contour_img;
        this->clusters_img_ |= cluster_img;
//        cv::imshow("contour image", contour_img);
//        cv::imshow("cluster image", cluster_img);
//        cv::imshow("contours image", this->contours_img_);
//        cv::imshow("clusters image", this->clusters_img_);
//        cv::waitKey(0);
    }

    if (!targets_queue.empty()) {
        // Get the cluster with the highest score.
        this->target_ = targets_queue.top();
//        LOG(INFO) << "contour size = " << this->target_.getContourPoint().size();
//        LOG(INFO) << "point cloud size = " << this->target_.getPointCloud()->size();
//        LOG(INFO) << "normal cloud size = " << this->target_.getNormalCloud()->size();
        return true;
    } else {
        return false;
    }
}

void fillHole(const cv::Mat& srcBw, cv::Mat &dstBw)
{
    // extended images
    cv::Size m_Size = srcBw.size();
    cv::Mat Temp=cv::Mat::zeros(m_Size.height+2,m_Size.width+2,srcBw.type());
    srcBw.copyTo(Temp(cv::Range(1, m_Size.height + 1),
                      cv::Range(1, m_Size.width + 1)));

    cv::floodFill(Temp, cv::Point(0, 0), cv::Scalar(255));

    //Crop extended images
    cv::Mat cutImg;
    Temp(cv::Range(1, m_Size.height + 1),
         cv::Range(1, m_Size.width + 1)).copyTo(cutImg);

    dstBw = srcBw | (~cutImg);
}

void
CircleDetector::findContourPoint(const cv::Mat_<uint8_t> &src,
                                 cv::Mat &contour_img,
                                 std::vector<cv::Point> &contour_points) const {

    // fill hole at image.
    cv::Mat fill_hole;
    fillHole(src, fill_hole);

    // Eliminating small unconnected regions.
//    // Set structure element type, size and anchor location .
//    cv::Mat elemStruct = getStructuringElement(
//        cv::MORPH_RECT, cv::Size(5, 5), cv::Point(1, 1));
//
//    dilate(fill_hole, fill_hole, elemStruct);
//    erode(fill_hole, fill_hole, elemStruct);
//    dilate(fill_hole, fill_hole, elemStruct);
//    erode(fill_hole, fill_hole, elemStruct);
//    cv::imshow("fill hole", fill_hole);

    cv::Mat blurImage;
    int edgeThresh = 100;
    blur(fill_hole, blurImage, cv::Size(3, 3));
    Canny(blurImage, contour_img, edgeThresh, edgeThresh * 3, 3);

//    cv::imshow("fill_hole", fill_hole);
//    cv::imshow("dst", dst);
//    cv::waitKey(0);

    //find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // CV_RETR_EXTERNAL: Only the outermost contour is detected, and the inner contour contained
    //                   in the outer contour is ignored.
    // CV_CHAIN_APPROX_NONE: Save all the continuous contour points on the object boundary to
    //                       the contours vector.
    findContours(contour_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    LOG(INFO) << "contours.size = " << contours.size();

    //get contour pixel set
    contour_points.clear();
    for (const auto &contour : contours) {
        if (contour.size() < this->param_.contour_min_size) {
            LOG(INFO) << "  contour.size = " << contour.size() << " Dorp.";
            continue;
        }
        LOG(INFO) << "  contour.size = " << contour.size() << " Ok.";
        for (const auto &point: contour) {
            contour_points.emplace_back(point.x, point.y);
        }

        // Displays the edges added each time.
//        cv::Mat img = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
//        for (const auto &point: contour_points) {
//            img.at<cv::Vec3b>((int) point.y, (int) point.x) = cv::Vec3b(0, 255, 0);
//        }
//        cv::imshow("current contour img", img);
//        cv::waitKey(0);
    }

    LOG(INFO) << "contour points.size = "<< contour_points.size();
}

float
CircleDetector::findCircle(const std::vector<cv::Point> &points,
                           cv::Point &center,
                           int &radius ) const {
    FittingCircle fitting;
    std::vector<int> indices;
    std::vector<float> coefficients;

    const size_t size = points.size();
    std::vector<Point2D> points_2d(size);
    for (int i = 0; i < size; ++i) {
        points_2d[i].x = points[i].x;
        points_2d[i].y = points[i].y;
    }

    fitting.setInputCloud(points_2d);
    fitting.setMaxIterations(this->param_.max_iteration);
    fitting.setPointRate(this->param_.fitting_rate);
    fitting.setDistanceThreshold(this->param_.pixel_dist_allow);
    fitting.fitting(indices, coefficients);

    if (3==coefficients.size()) {
        center.x = coefficients[0];
        center.y = coefficients[1];
        radius = coefficients[2];

        // The score is calculated according to the ratio of fitting points
        float score = static_cast<float>(indices.size()) / static_cast<float>(size);
        LOG(INFO) << "find circle center=(" << center.x << "," << center.y
                  << ") radius=" << radius << " indices.size=" << indices.size()
                  << " size=" << size << " score=" << score;
        return score;
    } else {
        LOG(WARNING) << "not find circle! input point size = " << points.size();
        return 0;
    }
}

void
CircleDetector::filterCircleCloud(const PointCloudConstPtr &input,
                                  const cv::Point &center_pixel,
                                  const int &radius_pixel,
                                  const PointCloudPtr &output) const {
    const size_t size = input->size();
    int pixel_x, pixel_y;
    int dist;
    for (size_t i = 0; i < size; ++i){
        const int idx = input->image_indices[i];
        pixel_y = idx / this->image_width_;
        pixel_x = idx % this->image_width_;
        dist = sqrt((pixel_x-center_pixel.x)*(pixel_x-center_pixel.x)
                     + (pixel_y-center_pixel.y)*(pixel_y-center_pixel.y));
        if (dist < radius_pixel-1){
            output->push_back(input->points[i], idx);
        }
    }
    LOG(INFO) << "filter circle cloud: input.size()=" << input->points.size() << " -> output.size()="
              << output->points.size();
}

bool CircleDetector::targetIsCircle()  {

    PointCloudPtr circle_cloud(new PointCloud);
    auto center_pixel = this->target_.getCenterInImage();
    auto radius_pixel = this->target_.getRadiusPixel();
    filterCircleCloud(this->tf_frame_.cloud, center_pixel, radius_pixel, circle_cloud);

    if (this->circles_map_.empty()){
        LOG(INFO) << " circles.size()=" << this->circles_map_.size() << "please circles target!";
    }

    Cluster cluster;
    cluster.setPointCloud(circle_cloud,this->image_width_);

    Point3D up_3d = cluster.getMostUpPoint();
    Point3D down_3d = cluster.getMostDownPoint();
    Point3D left_3d = cluster.getMostLeftPoint();
    Point3D right_3d = cluster.getMostRightPoint();

    Point2D up_2d(up_3d.y, up_3d.z);
    Point2D down_2d(down_3d.y, down_3d.z);
    Point2D left_2d(left_3d.y, left_3d.z);
    Point2D right_2d(right_3d.y, right_3d.z);

    float radius = (left_2d - right_2d) / 2;

    for (auto const circle : this->circles_map_) {
        float dist = fabsf(radius - circle.second.getRadius());
        if (dist < this->param_.point_dist_tolerance) {
            LOG(INFO) << "diff radius dist=" << dist << " < " << param_.point_dist_tolerance << " Ok.";
            this->target_.setId(circle.first);
            return true;
        } else {
            LOG(INFO) << "diff radius dist=" << dist << " > " << param_.point_dist_tolerance << " Drop.";
        }
    }

    return false;
}

bool
CircleDetector::calculateResult() {
    int64_t start, finish;
    start = common_func::get_timestamp_in_ms();

    // fit plane
    std::vector<int> plane_indices;
    std::vector<float> plane_coefficients;
    ransac( this->target_.getPointCloud(), this->target_.getNormalCloud(),
            plane_indices,plane_coefficients);

    if (plane_coefficients.empty()) {
        LOG(INFO) << "ransac plane_coefficients is empty!";
        return false;
    }

    copyPointCloudIndices(this->target_.getPointCloud(), plane_indices, target_cloud_);

    // Calculate this->target_ position.
    Point3D point_avg{};
    float total_x = .0f;
    float total_y = .0f;
    float total_z = .0f;
    for (auto point : target_cloud_->points) {
        total_x += point.x;
        total_y += point.y;
        total_z += point.z;
    }
    const size_t size = target_cloud_->size();
    point_avg.x = total_x / size;
    point_avg.y = total_y / size;
    point_avg.z = total_z / size;

    this->target_.setPose(point_avg);

    // calculate angle .
    const float a = plane_coefficients[0];
    const float b = plane_coefficients[1];
    const float c = plane_coefficients[2];

    this->target_.setNormal(Normal(a, b, c));

    auto angle = static_cast<float>(atan2(-b, -a));
    this->target_.setAngle(angle);

    LOG(INFO) << "plane equation: ax+by+cz+d=0" << " (a=" << a << " b=" << b << " c=" << c << ")";
    LOG(INFO) << "angle = atan2(b,a)*180/M_PI = " << angle * 180 / M_PI;

    // Normal vector angle of filter card .
    if (fabsf(a) < fabsf(b) || fabsf(a) < fabsf(c)) {
        this->target_.reset();
        LOG(INFO) << "target normal: fabsf(a) < fabsf(b) || fabsf(a) < fabsf(c) Drop.";
    }

    finish = common_func::get_timestamp_in_ms();
    int64_t ransac_time = finish - start;
    LOG(INFO) << "ransac fitting planar: circle_cloud_.size=" << target_cloud_->size()
              << " time consuming " << ransac_time << "ms";

    return true;
}


DetectResult
CircleDetector::detect(const O3d3xxFrame &frame) {
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
    this->tf_frame_ = frame;
    this->image_height_ = frame.confidence_img.rows;
    this->image_width_ = frame.confidence_img.cols;
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Camera coordinate system to AGV coordinate system.
//    start = common_func::get_timestamp_in_ms();
//    common_func::transformPoint(this->param_.camera_fixed, frame.cloud->points,
//                                this->tf_frame_.cloud->points);
//    finish = common_func::get_timestamp_in_ms();
//    this->consuming_time_recorder_.push_back(std::to_string(finish-start));

    // Point clouds with high credibility and small amplitude are filtered out.
    start = common_func::get_timestamp_in_ms();

    maskFilter(this->tf_frame_.confidence_img,
               this->tf_frame_.amplitude_img,
               this->tf_frame_.cloud,
               this->confidence_normalize_,
               this->amplitude_normalize_,
               this->filter_mask_img_,
               this->filter_mask_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Generate depth imageGenerate depth image.
    start = common_func::get_timestamp_in_ms();
    generateDistanceImage(this->filter_mask_cloud_, this->tf_frame_.distance_img);
    cv::normalize(this->tf_frame_.distance_img,
                  this->deep_img_, 0, 255, cv::NORM_MINMAX);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Filter out the points in the detection area.
    start = common_func::get_timestamp_in_ms();
    passThroughFilter(this->param_.detect_range,this->filter_mask_cloud_, this->filter_xyz_range_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Median filtering
    start = common_func::get_timestamp_in_ms();
    medianFilter(this->tf_frame_.cloud,this->filter_xyz_range_cloud_, this->filter_median_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Filtering outliers.
    start = common_func::get_timestamp_in_ms();
    outlierFilter(this->filter_median_cloud_, this->filter_outlier_cloud_);
    finish = common_func::get_timestamp_in_ms();
    this->consuming_time_recorder_.push_back(std::to_string(finish - start));

    // Calculating the normal vector of point cloud.
    start = common_func::get_timestamp_in_ms();

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
    euclidean<CircleDetectParam>(this->param_,
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

    if (is_find_target && targetIsCircle()) {
        this->result_.is_available = calculateResult();
    } else {
        this->result_.is_available = false;
    }

    this->result_.id = target_.getId();
    this->result_.x = target_.getPose().x;
    this->result_.y = target_.getPose().y;
    this->result_.z = target_.getPose().z;
    this->result_.angle = target_.getAngle();
    this->result_.width = std::numeric_limits<float>::quiet_NaN();
    this->result_.height = std::numeric_limits<float>::quiet_NaN();
    this->result_.normal = target_.getNormal();

    // draw shapes on the picture.
    drawCameraImage();
    drawMask();
    drawTarget();
    drawContour();
    drawCluster();

    return this->result_;
}

} // end of namespace perception