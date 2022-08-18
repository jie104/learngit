//
// Created by ljh on 2021/12/2.
//

// INCLUDE 
#include "decards_detector.h"
#include "../../../common/math_unit.hpp"
#include <queue>
#include <iomanip>
#include "../../common/algorithm/machine_vision.hpp"
#include "core/logger.h"

// CODE
namespace perception {

    DecardsDetector::DecardsDetector()
            : top_card_cloud_(new PointCloud()), bottom_card_cloud_(new PointCloud()) {

    }

    DecardsDetector::~DecardsDetector() {
        LOG(INFO) << "Decards detector was destroyed!";
    }

    bool
    DecardsDetector::init(const DecardsDetectParam &param,
                          const std::map<int, Card> &cards_map) {
        if (cards_map.empty()) {
            return false;
        }

        this->param_ = param;
        this->cards_map_.insert(cards_map.begin(), cards_map.end());
        this->consuming_time_recorder_.clear();

        showParam();

        return true;
    }

    void
    DecardsDetector::showParam() {
        std::stringstream string_buff;
        string_buff << std::endl;
        string_buff << "min_x                           = " << param_.detect_range.min_x << std::endl;
        string_buff << "max_x                           = " << param_.detect_range.max_x << std::endl;
        string_buff << "min_y                           = " << param_.detect_range.min_y << std::endl;
        string_buff << "max_y                           = " << param_.detect_range.max_y << std::endl;
        string_buff << "min_z                           = " << param_.detect_range.min_z << std::endl;
        string_buff << "max_z                           = " << param_.detect_range.max_z << std::endl;
        string_buff << "threshold                       = " << param_.threshold << std::endl;
        string_buff << "stack_top_margin                = " << param_.stack_top_margin << std::endl;
        string_buff << "min_cluster_size          = " << param_.min_cluster_size << std::endl;
        string_buff << "max_cluster_size          = " << param_.max_cluster_size << std::endl;
        string_buff << "point_dist_tolerance          = " << param_.point_dist_tolerance << std::endl;
        string_buff << "normal_angle_tolerance          = " << param_.normal_angle_tolerance << std::endl;
        string_buff << "both_pallet_dist_allow          = " << param_.both_pallet_dist_allow << std::endl;
        string_buff << "up_extend_pixel          = " << param_.up_extend_pixel << std::endl;
        string_buff << "down_extend_pixel          = " << param_.down_extend_pixel << std::endl;
        string_buff << "min_area_rate          = " << param_.min_area_rate << std::endl;
        string_buff << "max_area_rate          = " << param_.max_area_rate << std::endl;
        string_buff << "hole_height_rate_diff_allow          = " << param_.hole_height_rate_diff_allow << std::endl;
        string_buff << "pallet_normal_diff_angle_allow          = " << param_.pallet_normal_diff_angle_allow << std::endl;
        string_buff << "enable_record_debug_image          = " << param_.enable_record_debug_image << std::endl;

        LOG(INFO) << string_buff.str();

    }

    void
    DecardsDetector::reset() {
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
        this->filter_outlier_cloud_->clear();
        this->filter_normal_point_cloud_->clear();
        this->filter_normal_normal_cloud_->clear();
        this->target_cloud_->clear();
        this->clusters_cloud_->clear();
        this->consuming_time_recorder_.clear();
        this->top_card_cloud_->clear();
        this->bottom_card_cloud_->clear();
    }

    void
    DecardsDetector::drawCardHole() {
        common_func::GrayMaptoColor(this->roi_img_, this->hole_img_, cv::COLORMAP_OCEAN);
        for (const auto &rect: this->target_.getHoleRect()) {
            cv::rectangle(this->hole_img_, rect, cv::Scalar(0, 0, 255), 2);
        }
        putText(this->hole_img_, "card hole", cv::Point(10, 20),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                1, 1);
    }

    void
    DecardsDetector::drawCardPallet() {
        common_func::GrayMaptoColor(roi_img_, this->pallet_img_, cv::COLORMAP_OCEAN);
        for (const auto &rect: this->target_.getPalletRect()) {
            cv::rectangle(this->pallet_img_, rect, cv::Scalar(0, 0, 255), 2);
        }
        putText(this->pallet_img_, "card pallet", cv::Point(10, 20),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                1, 1);
    }

    void
    DecardsDetector::drawTarget() {
        common_func::GrayMaptoColor(this->roi_img_, this->target_img_, cv::COLORMAP_OCEAN);
        cv::rectangle(this->target_img_, this->target_.getRect(),
                      cv::Scalar(0, 0, 255), 2);
        putText(this->target_img_, "card target", cv::Point(10, 20),
                CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255),
                1, 1);
    }

    void
    DecardsDetector::getAmplitudeImage(cv::Mat &img) const {
        img = this->amplitude_normalize_.clone();
    }

    void
    DecardsDetector::getConfidenceImage(cv::Mat &img) const {
        img = this->confidence_normalize_.clone();
    }

    void
    DecardsDetector::getPalletImage(cv::Mat &img) const {
        img = this->pallet_img_.clone();
    }

    void
    DecardsDetector::getHoleImage(cv::Mat &img) const {
        img = this->hole_img_.clone();
    }

    void
    DecardsDetector::getDeepImage(cv::Mat &img) const {
        img = this->deep_img_.clone();
    }

    void
    DecardsDetector::getXYZThresholdImage(cv::Mat &img) const {
        img = this->xyz_threshold_img_.clone();
    }

    void
    DecardsDetector::filterClusters(const NormalCloudConstPtr &normals,
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

    bool
    DecardsDetector::calculateResult() {

        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        const std::vector<int> pallets_indices = this->target_.getPalletsIndices();
        const size_t pallet_num = pallets_indices.size();
        const Cluster *left_pallet = &this->clusters_map_.at(pallets_indices[0]);
        const Cluster *right_pallet = &this->clusters_map_.at(pallets_indices[pallet_num - 1]);

        // Calculate target position .
        const float x = (left_pallet->getAvgX() + right_pallet->getAvgX()) / 2;
        const float y = (left_pallet->getAvgY() + right_pallet->getAvgY()) / 2;

        float sum_z = 0;
        for (auto idx: pallets_indices){
            sum_z += this->clusters_map_.at(idx).getMostUpPoint().z;
            LOG(INFO) <<   "this->clusters_map_.at(idx).getAvgZ();" << this->clusters_map_.at(idx).getMostUpPoint().z;
        }
        const float z = sum_z / pallet_num;
        
        LOG(INFO) <<   "pallet_num:   " << pallet_num;
        this->target_.setPose(Point3D(x, y, z));

        // fitting plane ax + by + cz + d = 0 .
        std::vector<int> plane_indices;
        std::vector<float> plane_coefficients;
        ransac(this->target_.getPointCloud(), this->target_.getNormalCloud(),
               plane_indices, plane_coefficients);

        if (plane_coefficients.empty()) {
            LOG(INFO) << "ransac plane_coefficients is empty!";
            return false;
        }

        copyPointCloudIndices(this->target_.getPointCloud(), plane_indices, this->target_cloud_);

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
        LOG(INFO) << "ransac fitting planar: card_cloud_.size=" << this->target_cloud_->size()
                  << " time consuming " << ransac_time << "ms";

        return true;
    }

    bool
    DecardsDetector::stackIsTop(const O3d3xxFrame &frame) {
        // 1. If stack 's top is in camera FOV

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


        // Filter out the points in the detection area.
        start = common_func::get_timestamp_in_ms();
        passThroughFilter(this->param_.detect_range, this->filter_mask_cloud_, this->filter_xyz_range_cloud_);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Generate depth imageGenerate depth image.
        start = common_func::get_timestamp_in_ms();
        generateDistanceImage(this->filter_xyz_range_cloud_, this->tf_frame_.distance_img);
        cv::normalize(this->tf_frame_.distance_img,
                      this->deep_img_, 0, 255, cv::NORM_MINMAX);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Deep image Shape trans rectangle1 Image and detect if top.
        cv::Mat gray_image;
        cv::Mat rect_tran_image;
        start = common_func::get_timestamp_in_ms();
        mvision::threshold2(this->deep_img_, gray_image, 138, 255);
        mvision::shapeTrans(gray_image, rect_tran_image, mvision::ShapeTransType::RECTANGLE1);
        std::vector<std::vector<cv::Point>> stack_contours;
        cv::findContours(rect_tran_image, stack_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        cv::Rect stack_rect = cv::boundingRect(stack_contours[0]);
        cv::Point stack_tl = stack_rect.tl();
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        if (stack_tl.y < 5) {
//        this->result_.is_stacktop = false;
            return false;
        } else {
//        this->result_.is_stacktop = true ;
            return true;
        }
    }

    DetectResult
    DecardsDetector::detect(const O3d3xxFrame &frame) {
        // 1. If stack 's top is in camera FOV
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

        if(!filter_mask_img_.empty()) cv::imwrite("/sros/debug_data/001filter_mask_img_.jpg",filter_mask_img_);


        // Filter out the points in the detection area.
        start = common_func::get_timestamp_in_ms();
        passThroughFilter(this->param_.detect_range, this->filter_mask_cloud_, this->filter_xyz_range_cloud_);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Median filtering
        // start = common_func::get_timestamp_in_ms();
        // medianFilter(this->tf_frame_.cloud, this->filter_xyz_range_cloud_, this->filter_median_cloud_);
        // finish = common_func::get_timestamp_in_ms();
        // this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        //Filtering outliers.
        start = common_func::get_timestamp_in_ms();
        outlierFilter(this->filter_xyz_range_cloud_, this->filter_outlier_cloud_);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        // Generate depth imageGenerate depth image.
        start = common_func::get_timestamp_in_ms();
        generateDistanceImage(this->filter_outlier_cloud_, this->tf_frame_.distance_img);
        cv::normalize(this->tf_frame_.distance_img,
                      this->deep_img_, 0, 255, cv::NORM_MINMAX);
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        if(!this->tf_frame_.distance_img.empty()) cv::imwrite("/sros/debug_data/002distance_img.jpg",this->tf_frame_.distance_img);

        if(!deep_img_.empty()) cv::imwrite("/sros/debug_data/003deep_img_.jpg",deep_img_);

        // Deep image Shape trans rectangle1 Image and detect if top.
        cv::Mat thresh_img, rect1_img, fill_img, select_fill_img;
        start = common_func::get_timestamp_in_ms();
        mvision::threshold2(this->deep_img_, thresh_img, this->param_.threshold, 255);
        if(!thresh_img.empty()) cv::imwrite("/sros/debug_data/004thresh_img.jpg",thresh_img);

        if (thresh_img.empty()) {
            LOGGER(INFO, ACTION_TASK) << "#Decards Error: Deep threshold image is empty. threshold =" << this->param_.threshold;
            return result_;
        }
        // // 进行栈板孔洞补全
        // mvision::fillUp(thresh_img, fill_img);

        // 设置栈板垛筛选特征
        std::vector<mvision::SelectShapeType> shape_types;
        shape_types.push_back(mvision::SelectShapeType::SELECT_AREA);
        shape_types.push_back(mvision::SelectShapeType::SELECT_COLUMN);

        // 设置栈板垛筛选特征范围值
        std::vector<double> shape_type_mins, shape_type_maxs;
        shape_type_mins.push_back(this->deep_img_.rows * this->deep_img_.cols * 2 / 3.0 / 48);
        shape_type_mins.push_back(this->deep_img_.cols / 3.0);
        shape_type_maxs.push_back(this->deep_img_.rows * this->deep_img_.cols * 2 / 3.0);
        shape_type_maxs.push_back(this->deep_img_.cols * 2 / 3.0);

        // 进行栈板垛筛选
        int shape_count = mvision::selectShape(thresh_img, select_fill_img, shape_types,
                                               mvision::SelectOperation::SELECT_AND, shape_type_mins, shape_type_maxs);

        if(!select_fill_img.empty()) cv::imwrite("/sros/debug_data/005select_fill_img.jpg",select_fill_img);
                                       
        // 栈板垛数量不为1 则报错
        if (shape_count != 1) {
            LOGGER(INFO, ACTION_TASK) << "#Decards Error: " << shape_count <<" stacks in Perception Region";
            return result_;
        }

        // 将栈板垛图像转换成rectange1图像
        mvision::shapeTrans(select_fill_img, rect1_img, mvision::ShapeTransType::RECTANGLE1);
        if(!rect1_img.empty()) cv::imwrite("/sros/debug_data/006rect1_img.jpg",rect1_img);


        // 查找rectange1图像轮廓
        std::vector<std::vector<cv::Point>> stack_contours;
        cv::findContours(rect1_img, stack_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (stack_contours.size() != 1) return result_;

        // 计算轮廓的左上角
        cv::Rect stack_rect = cv::boundingRect(stack_contours[0]);
        cv::Point stack_tl = stack_rect.tl();
        finish = common_func::get_timestamp_in_ms();
        this->consuming_time_recorder_.push_back(std::to_string(finish - start));

        LOG(INFO) << "this->image_height_ - stack_tl.y: " << this->image_height_ - stack_tl.y;
        // 根据轮廓左上角度y值 判断 是否整个栈板垛在相机视野内 ,如果在相机视野内则进行下一步的顶层栈板识别,否则通知执行机构继续进行升叉
        if (!this->result_.is_stacktop) {
            if (this->image_height_ - stack_tl.y > this->param_.stack_top_margin) {
                this->result_.is_stacktop = false;
                //this->result_.is_available = true;
                LOGGER(INFO, ACTION_TASK) << "#Decards Info: Stack in Current Frame is no top." ;
                return result_;
            } else{
                this->result_.is_stacktop = true;
                //this->result_.is_available = false;
                LOGGER(INFO, ACTION_TASK) << "#Decards Info: Stack in Current Frame is top." ;
                //return result_;
            }
        }
        

        // 2.进行顶层栈板的识别
        cv::Mat hole_sub_img, hole_select_img;
        hole_sub_img = rect1_img - thresh_img;
        if(!hole_sub_img.empty()) cv::imwrite("/sros/debug_data/007hole_sub_img.jpg",hole_sub_img);

        std::vector<mvision::SelectShapeType> hole_shape_types;
        hole_shape_types.push_back(mvision::SelectShapeType::SELECT_WIDTH);
        hole_shape_types.push_back(mvision::SelectShapeType::SELECT_HEIGHT);

        std::vector<double> hole_shape_type_mins, hole_shape_type_maxs;
        // hole_shape_type_mins.push_back(stack_rect.width / 4);
        // hole_shape_type_mins.push_back(stack_rect.width / 30);
        // hole_shape_type_maxs.push_back(stack_rect.width / 2);
        // hole_shape_type_maxs.push_back(stack_rect.width / 8);

        hole_shape_type_mins.push_back(stack_rect.width / 6);
        hole_shape_type_mins.push_back(stack_rect.height / 6);
        hole_shape_type_maxs.push_back(stack_rect.width / 2);
        hole_shape_type_maxs.push_back(stack_rect.height);

        LOGGER(INFO, ACTION_TASK) << "#Decards Info: stack rectangle width: "<< stack_rect.width;

        cv::Mat element_rect7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::morphologyEx(hole_sub_img, hole_sub_img, cv::MORPH_OPEN, element_rect7x7);

        int hole_shape_count = mvision::selectShape(hole_sub_img, hole_select_img, hole_shape_types,
                                                    mvision::SelectOperation::SELECT_AND,
                                                    hole_shape_type_mins, hole_shape_type_maxs);

        // record debug image
        if(!hole_select_img.empty()) cv::imwrite("/sros/debug_data/008hole_select_img.jpg",hole_select_img);

        // if(param_.enable_record_debug_image){
            
        // }

        LOGGER(INFO, ACTION_TASK) << "#Decards Info: stack 's card holes "<< hole_shape_count;
        if (hole_shape_count <= 0) {
            LOGGER(INFO, ACTION_TASK) << "#Decards Error: stack 's card holes count <=0." ;
            this->result_.is_stacktop = true;
            this->result_.is_available = false;
            return result_;
        }

        cv::Mat element_rect1x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3));
        cv::Mat element_rect45x1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(45, 1));
        cv::morphologyEx(hole_select_img, hole_select_img, cv::MORPH_OPEN, element_rect1x3);
        cv::morphologyEx(hole_select_img, hole_select_img, cv::MORPH_CLOSE, element_rect45x1);
        if(!hole_select_img.empty()) cv::imwrite("/sros/debug_data/009hole_select_img.jpg",hole_select_img);

        std::vector<std::vector<cv::Point>> select_contours;
        std::vector<int> stack_min_area, sort_pos;
        cv::findContours(hole_select_img, select_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        mvision::sortRegion(hole_select_img, select_contours, sort_pos, mvision::SortCriterion::FIRST_POINT, false,
                            mvision::SortDirection::ROW);
        std::vector<std::vector<cv::Point> > top_contours, bottom_contours;
        top_contours.push_back(select_contours[sort_pos[sort_pos.size() - 1]]);
        bottom_contours.push_back(select_contours[sort_pos[0]]);
        cv::Mat card_target_img;
        card_target_img = hole_select_img.clone();

        cv::drawContours(card_target_img, top_contours, -1, cv::Scalar(0), CV_FILLED);
        cv::bitwise_xor(hole_select_img, card_target_img, card_target_img);
        if(!card_target_img.empty()) cv::imwrite("/sros/debug_data/010card_target_img.jpg",card_target_img);

        cv::Mat element_rect1x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 5));
        cv::morphologyEx(card_target_img, card_target_img, cv::MORPH_DILATE, element_rect45x1);
        if(!card_target_img.empty()) cv::imwrite("/sros/debug_data/011card_target_img.jpg",card_target_img);
      
        cv::morphologyEx(card_target_img, card_target_img, cv::MORPH_ERODE, element_rect1x5);
        if(!card_target_img.empty()) cv::imwrite("/sros/debug_data/012card_target_img.jpg",card_target_img);

        cv::bitwise_and(thresh_img, card_target_img, card_target_img);
        if(!card_target_img.empty()) cv::imwrite("/sros/debug_data/13card_target_img.jpg",card_target_img);

        cv::morphologyEx(card_target_img, card_target_img, cv::MORPH_DILATE, element_rect1x5);
        if(!card_target_img.empty()) cv::imwrite("/sros/debug_data/14card_target_img.jpg",card_target_img);


        // cv::Rect top_contours_rect = cv::boundingRect(select_contours[sort_pos[sort_pos.size() - 1]]);
        // cv::Point top_contours_tl = top_contours_rect.tl();


        // 校验，识别出的栈板高度与栈板垛的高度进行校验比较，避免取的不是顶部栈板
        // if(fabs(top_contours_tl.y - stack_tl.y) > top_contours_rect.height*1.5 ) {
        //     LOG(INFO) << "#Decards Error: detect obj card 's y more than stack 'y , stack top:" << stack_tl.y << ", card top: " << top_contours_tl.y
        //     << " card height: "<<top_contours_rect.height;;
        //     this->result_.is_stacktop = true;
        //     this->result_.is_available = false;
        //     return result_;
        // }

        maskFilter(card_target_img, this->tf_frame_.cloud, this->top_card_cloud_);

        // Median filtering
        start = common_func::get_timestamp_in_ms();
        medianFilter(this->tf_frame_.cloud, this->top_card_cloud_, this->filter_median_cloud_);
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
        euclidean<DecardsDetectParam>(this->param_,
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

        if (is_find_target) {

            // Correct target width.
            start = common_func::get_timestamp_in_ms();
            correctTargetWidth();
            finish = common_func::get_timestamp_in_ms();
            this->consuming_time_recorder_.push_back(std::to_string(finish - start));

            // Generate target image.
            start = common_func::get_timestamp_in_ms();
            PointCloudPtr target_point_cloud(new PointCloud());
            generateTargetImage(frame.cloud, target_point_cloud);
            finish = common_func::get_timestamp_in_ms();
            this->consuming_time_recorder_.push_back(std::to_string(finish - start));

            // Confirm whether the target is a pallet.
            start = common_func::get_timestamp_in_ms();
            if (targetIsCard()) {
                finish = common_func::get_timestamp_in_ms();
                this->consuming_time_recorder_.push_back(std::to_string(finish - start));

                // Correct target altitude.
                start = common_func::get_timestamp_in_ms();
                correctTargetHeight();
                finish = common_func::get_timestamp_in_ms();
                this->consuming_time_recorder_.push_back(std::to_string(finish - start));

                // Calculate target position and angle.
                start = common_func::get_timestamp_in_ms();
                this->result_.is_available = calculateResult();
                finish = common_func::get_timestamp_in_ms();
                this->consuming_time_recorder_.push_back(std::to_string(finish - start));

                this->result_.id = target_.getId();
                this->result_.x = target_.getPose().x;
                this->result_.y = target_.getPose().y;
                this->result_.z = target_.getPose().z;
                this->result_.angle = target_.getAngle();
                this->result_.width = target_.getCardWidth();
                this->result_.height = target_.getHoleHeight();
                this->result_.normal = target_.getNormal();
            }
        }

        

        return this->result_;
    }

    DetectResult
    DecardsDetector::detect_test(const O3d3xxFrame &frame) {
        {
            // Clear the data stored in the last processing.
            this->reset();

            // Copy data for processing.
            this->tf_frame_ = frame;
            this->image_height_ = frame.confidence_img.rows;
            this->image_width_ = frame.confidence_img.cols;

            std::vector<standard::PalletInfo> pallet_info;
            auto pallect_detector = std::make_shared<standard::LivoxPalletDetect>();
            LOG(INFO) << "this->tf_frame_.cloud size: " << this->tf_frame_.cloud->size();
            pallect_detector->onReceiveLivoxPoint(this->tf_frame_.cloud, pallet_info, param_.camera_to_agv_transf, param_.camera_to_agv_rotate);

            if (pallet_info.empty()) {
                LOG(ERROR) << "not detect any pallect.";
                return this->result_;
            }

            // 选出离当前目标栈板高度最高的栈板
            standard::PalletInfo target_pallet;

            auto selectPallet = [&](std::vector<standard::PalletInfo> &pallets, standard::PalletInfo &target) {
            
                std::sort(pallets.begin(), pallets.end(), [](const standard::PalletInfo& a, const standard::PalletInfo& b)
                                    {return a.mean[2] < b.mean[2];});
                
                target = pallets.front();
                LOG(INFO) << "Select the pallet, z value is: " << target.mean[2];
            };
            selectPallet(pallet_info, target_pallet);

            LOG(INFO) << "find pallet, the XYZ coordinate of pallet is: (" << target_pallet.mean[0] << ", "
                    << target_pallet.mean[1] << ", " << target_pallet.mean[2] << ").";
            LOG(INFO) << "The normal is: (" << target_pallet.norm[0] << ", " << target_pallet.norm[1] << ", "
                    << target_pallet.norm[2] << ")";
            auto angle = atan2(-target_pallet.norm[1], -target_pallet.norm[0]);
            LOG(INFO) << "The angle is : " << angle * 180 / M_PI ;

            this->result_.is_available = true;
            this->result_.id = 0;
            this->result_.x = target_pallet.mean[0];
            this->result_.y = target_pallet.mean[1];
            this->result_.z = target_pallet.mean[2];
            //    this->result_.angle = angle>0?angle-M_PI:angle+M_PI;
            this->result_.angle = angle;

            // this->result_.width = target_.getCardWidth();
            //         this->result_.height = target_.getHoleHeight();
            //         this->result_.normal = target_pallet.norm;

            return this->result_;
        }
    }

    bool
    DecardsDetector::searchTarget() {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        size_t pallet_num = this->clusters_map_.size();
        if (this->clusters_map_.empty()) {
            return false;
        }

        size_t i, j;
        std::vector<float> hole_width_vct;
        hole_width_vct.resize(pallet_num - 1);
        for (i = 0; i < pallet_num - 1; ++i) {
            // both pallet distance
            Point3D pa = this->clusters_map_.at(i).getMostRightPoint();
            Point3D pb = this->clusters_map_.at(i + 1).getMostLeftPoint();
            hole_width_vct[i] = distanceBothPoint2d(pa.x, pa.y, pb.x, pb.y);
        }

        std::stringstream string_buff;
        string_buff << "   hole_dist_vct: ";
        for (auto iter: hole_width_vct) {
            string_buff << setiosflags(std::ios::fixed) << std::setw(5) << std::setprecision(3)
                        << iter << " ";
        }
        LOG(INFO) << string_buff.str();

        string_buff.str("");
        string_buff << "pallet_width_vct: ";
        for (const auto &iter: this->clusters_map_) {
            string_buff << setiosflags(std::ios::fixed) << std::setw(5) << std::setprecision(3)
                        << iter.second.getWidth() << " ";
        }
        LOG(INFO) << string_buff.str();


        auto check_card = [&](const Card &card, const int &start_index, const int &hole_num,
                              const int &pallet_num) -> bool {
            float real_width, measure_width;

            // Confirm that the hole width measurement error is within the range.
            std::vector<float> holes_width = card.getHolesWidth();
            for (size_t k = 0; k < hole_num; ++k) {
                real_width = holes_width[k];
                measure_width = hole_width_vct[start_index + k];
                if (fabsf(real_width - measure_width) > this->param_.both_pallet_dist_allow) {
                    LOG(INFO) << "   hole: [" << start_index << "," << start_index + hole_num - 1
                              << "] real_width-measure_width = " << real_width << "-"
                              << setiosflags(std::ios::fixed) << std::setw(5) << std::setprecision(3)
                              << measure_width << "=" << fabsf(real_width - measure_width)
                              << " > " << this->param_.both_pallet_dist_allow << "=both_pallet_dist_allow Drop";
                    return false;
                } else {
                    LOG(INFO) << "   hole: [" << start_index << "," << start_index + hole_num - 1
                              << "] real_width-measure_width = " << real_width << "-"
                              << setiosflags(std::ios::fixed) << std::setw(5) << std::setprecision(3)
                              << measure_width << "=" << fabsf(real_width - measure_width)
                              << " < " << this->param_.both_pallet_dist_allow << "=both_pallet_dist_allow OK";
                }
            }

            // Confirm that the pallet width measurement error is within the range.
            std::vector<float> pallets_width = card.getPalletsWidth();
            for (size_t k = 1; k < pallet_num - 1; ++k) {
                real_width = pallets_width[k];
                measure_width = this->clusters_map_.at(start_index + k).getWidth();
                if (fabsf(real_width - measure_width) > this->param_.both_pallet_dist_allow) {
                    LOG(INFO) << " pallet: [" << start_index << "," << start_index + pallet_num - 1
                              << "] real_width-measure_width = " << real_width << "-"
                              << setiosflags(std::ios::fixed) << std::setw(5) << std::setprecision(3)
                              << measure_width << "=" << fabsf(real_width - measure_width)
                              << " > " << this->param_.both_pallet_dist_allow << "=both_pallet_dist_allow Drop";
                    return false;
                } else {
                    LOG(INFO) << " pallet: [" << start_index << "," << start_index + pallet_num - 1
                              << "] real_width-measure_width = " << real_width << "-"
                              << setiosflags(std::ios::fixed) << std::setw(5) << std::setprecision(3)
                              << measure_width << "=" << fabsf(real_width - measure_width)
                              << " < " << this->param_.both_pallet_dist_allow << "=both_pallet_dist_allow OK";
                }
            }

            // Confirm that the pallet normal measurement error is within the range.
            std::vector<Normal> normals(pallet_num);
            for (size_t k = 0; k < pallet_num; ++k) {
                normals[k] = this->clusters_map_.at(k).getNormal();
                LOG(INFO) << " normal[" << k << "]:" << "(" << normals[k].normal_x << ", " << normals[k].normal_y
                          << ", " << normals[k].normal_z << ")";
            }
            float angle;
            // The angle between any two normal vectors of pier surface shall not exceed the set value
            for (int i = 0; i < pallet_num; ++i) {
                for (int j = i + 1; j < pallet_num; ++j) {
                    angle = angleBothVector(normals[i], normals[j]);
                    if (angle > this->param_.pallet_normal_diff_angle_allow) {
                        LOG(INFO) << " normal[" << i << "]<->normal[" << j << "] angle=" << angle << " > "
                                  << this->param_.pallet_normal_diff_angle_allow << " Drop";
//                    return false;
                    } else {
                        LOG(INFO) << " normal[" << i << "]<->normal[" << j << "] angle=" << angle << " < "
                                  << this->param_.pallet_normal_diff_angle_allow << " OK";
                    }
                }
            }

            return true;
        };

        // find card pallets.
        std::vector<CardTarget> targets_vct;
        bool is_find = false;
        for (auto const &card: cards_map_) {
            size_t card_hole_size = card.second.getHolesWidth().size();
            size_t card_pallet_size = card.second.getPalletsWidth().size();
            LOG(INFO) << "id=" << card.first << " hole_size=" << card_hole_size << " pallet_size=" << card_pallet_size;
            if (pallet_num < card_pallet_size) {
                LOG(INFO) << "pallet_num < card_pallet_size";
                continue;
            }
            for (j = 0; j < this->clusters_map_.size() - card_hole_size; ++j) {
                if (check_card(card.second, j, card_hole_size, card_pallet_size)) {
                    CardTarget temp_target;
                    temp_target.setId(card.first);
                    temp_target.setPalletsIndices(j, card_pallet_size);
                    targets_vct.push_back(std::move(temp_target));
                    LOG(INFO) << "find target " << targets_vct.size() - 1 << " : id=" << card.first
                              << " pallet_index:[" << j << "~" << j + card_pallet_size - 1 << "]";
                    is_find = true;
                }
            }
        }

        // filter target.
        if (is_find) {
            int select_index = 0;
            if (targets_vct.size() > 1) {
                float min_avg_y, avg_y;
                min_avg_y = std::numeric_limits<float>::max();
                for (i = 0; i < targets_vct.size(); ++i) {
                    std::vector<int> indices = targets_vct[i].getPalletsIndices();
                    avg_y = .0f;
                    for (auto idx: indices) {
                        avg_y += this->clusters_map_.at(idx).getAvgY();
                    }
                    avg_y = fabsf(avg_y / indices.size());
                    LOG(INFO) << "target_index " << i << " avg_y=" << avg_y;
                    if (avg_y < min_avg_y) {
                        min_avg_y = avg_y;
                        select_index = i;
                        LOG(INFO) << "update min_avg_y = " << min_avg_y;
                    }
                }
                LOG(INFO) << "select target index=" << select_index << " min_avg_y=" << min_avg_y;
            }

            this->target_.setId(targets_vct[select_index].getId());
            this->target_.setPalletsIndices(targets_vct[select_index].getPalletsIndices());
        } else {
            LOG(INFO) << "not find target";
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t search_target_time = finish - start;
        LOG(INFO) << "search target time consuming " << search_target_time << "ms";

        if (is_find)
            return true;
        else
            return false;
    }

    void
    DecardsDetector::correctTargetWidth() {

        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        // Fixed the rectangle of pallet leg.
        std::stringstream string_buff;
        const std::vector<int> pallet_indices = this->target_.getPalletsIndices();
        Card *card_ptr = &this->cards_map_[this->target_.getId()];

        cv::Rect rect;
        size_t i;
        const size_t hole_num = card_ptr->getHolesSize();
        const size_t pallet_num = card_ptr->getPalletSize();

        // Calculate the scale coefficient between image pixel and real scale.
        float width_rate = .0f, height_rate = .0f;
        if (pallet_num >= 3) {
            float total_pixel_width = .0f;
            float total_pallet_width = .0f;
            for (i = 1; i < pallet_num - 1; ++i) {
                total_pixel_width += this->clusters_map_[pallet_indices[i]].getRect().width;
                total_pallet_width += this->clusters_map_[pallet_indices[i]].getWidth();
            }
            width_rate = total_pixel_width / total_pallet_width;

            const float pixel_height = this->clusters_map_[pallet_indices[1]].getRect().height;
            const float pallet_height = this->clusters_map_[pallet_indices[1]].getHeight();
            height_rate = pixel_height / pallet_height;

            LOG(INFO) << "pixel_width=" << total_pixel_width
                      << " pallet_width=" << total_pallet_width
                      << " width_rate=" << width_rate;

            LOG(INFO) << "pixel_height=" << pixel_height
                      << " pallet_height=" << pallet_height
                      << " height_rate=" << height_rate;

        } else {
            // Select the smaller pier to calculate the coefficient of scale transformation.
            float select_pallet_pixel_width = std::numeric_limits<float>::max();
            float select_pallet_width = .0f;
            float select_pallet_pixel_height = .0f;
            float select_pallet_height = .0f;
            for (auto index: pallet_indices) {
                rect = this->clusters_map_[index].getRect();
                LOG(INFO) << "pallet rect: [" << rect.x << " " << rect.y << " " << rect.width << " " << rect.height
                          << "]";
                if (rect.width < select_pallet_pixel_width) {
                    select_pallet_pixel_width = rect.width;
                    select_pallet_pixel_height = rect.height;
                    select_pallet_width = this->clusters_map_[index].getWidth();
                    select_pallet_height = this->clusters_map_[index].getHeight();
                }
            }

            width_rate = select_pallet_pixel_width / select_pallet_width;
            height_rate = select_pallet_pixel_height / select_pallet_height;

            LOG(INFO) << "pixel_width=" << select_pallet_pixel_width
                      << " pallet_width=" << select_pallet_width
                      << " width_rate=" << width_rate;

            LOG(INFO) << "pallet_pixel_height=" << select_pallet_pixel_height
                      << " pallet_height=" << select_pallet_height
                      << " height_rate=" << height_rate;
        }

        // Calculate the pixel height of the pallet leg.
        const int pallet_pixel_height = static_cast<int>(card_ptr->getPalletHeight() * height_rate);
        card_ptr->setPalletPixelHeight(pallet_pixel_height);
        LOG(INFO) << "setPalletPixelHeight: " << pallet_pixel_height;

        // Calculate the pixel height of the card hole .
        const int hole_pixel_height = static_cast<int>(card_ptr->getHoleHeight() * height_rate);
        card_ptr->setHolePixelHeight(hole_pixel_height);
        LOG(INFO) << "  setHolePixelHeight: " << hole_pixel_height;

        // Calculate the pixel width of the card hole .
        const std::vector<float> holes_width = card_ptr->getHolesWidth();
        std::vector<int> hole_pixel_width(holes_width.size());
        string_buff << "setHolePixelWidth: ";
        for (i = 0; i < hole_num; ++i) {
            hole_pixel_width[i] = holes_width[i] * width_rate;
            string_buff << hole_pixel_width[i] << " ";
        }
        card_ptr->setHolePixelWidth(hole_pixel_width);
        LOG(INFO) << string_buff.str();
        string_buff.str("");

        // Calculate the pixel width of the card leg .
        const std::vector<float> pallets_width = card_ptr->getPalletsWidth();
        std::vector<int> pallet_pixel_width(pallets_width.size());
        string_buff << "setPalletPixelWidth: ";
        for (i = 0; i < pallet_num; ++i) {
            pallet_pixel_width[i] = pallets_width[i] * width_rate;
            string_buff << pallet_pixel_width[i] << " ";
        }
        card_ptr->setPalletPixelWidth(pallet_pixel_width);
        LOG(INFO) << string_buff.str();
        string_buff.str("");

        // Correct the rectangle width of card leg.
        unsigned int left_pallet_index = 0;
        unsigned int right_pallet_index = pallet_num - 1;
        const Cluster *left_pallet = &this->clusters_map_[pallet_indices[left_pallet_index]];
        const Cluster *right_pallet = &this->clusters_map_[pallet_indices[right_pallet_index]];
        cv::Rect left_pallet_rect, right_pallet_rect;

        // Handling the left pallet leg .
        rect = left_pallet->getRect();
        left_pallet_rect = rect;
        left_pallet_rect.x += rect.width - pallet_pixel_width[left_pallet_index];
        left_pallet_rect.width = pallet_pixel_width[left_pallet_index];
        this->clusters_map_[pallet_indices[left_pallet_index]].setRect(left_pallet_rect, this->image_width_);

        // Handling the right pallet leg .
        rect = right_pallet->getRect();
        right_pallet_rect = rect;
        right_pallet_rect.width = pallet_pixel_width[right_pallet_index];
        this->clusters_map_[pallet_indices[right_pallet_index]].setRect(right_pallet_rect, this->image_width_);

        // Calculate the minimum bounding rectangle of the pallet
        rect.x = left_pallet_rect.x;
        rect.y = MIN(left_pallet_rect.y, right_pallet_rect.y);
        int diff_height = right_pallet_rect.y - left_pallet_rect.y;
        rect.width = (right_pallet_rect.x - left_pallet_rect.x + 1) + right_pallet_rect.width;
        rect.height = diff_height > 0 ?
                      right_pallet_rect.height + diff_height + 1 : left_pallet_rect.height - diff_height + 1;
        rect.width = rect.x + rect.width < this->image_width_ ? rect.width : this->image_width_ - rect.x - 1;
        rect.height = rect.y + rect.height < this->image_height_ ? rect.height : this->image_height_ - rect.y - 1;

        // Calculate the bottom left and bottom right points of the pallet .
        cv::Point left_down_point, right_down_point;
        left_down_point.x = rect.x;
        left_down_point.y = rect.y + rect.height;
        right_down_point.x = rect.x + rect.width;
        right_down_point.y = rect.y + rect.height;
        this->target_.setLeftDownPoint(left_down_point);
        this->target_.setRightDownPoint(right_down_point);
        this->target_.setRect(rect);

        finish = common_func::get_timestamp_in_ms();
        int64_t correct_target_width_time = finish - start;
        LOG(INFO) << "correct target width time consuming " << correct_target_width_time << "ms";
    }

    void
    DecardsDetector::generateTargetImage(const PointCloudConstPtr &cloud,
                                         PointCloudPtr &target_point_cloud) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        // Calculate target search area .
        cv::Rect rect = this->target_.getRect();
        rect.y -= this->param_.up_extend_pixel;
        rect.height += (this->param_.up_extend_pixel + this->param_.down_extend_pixel);

        // Restricted area search boundary .
        rect.y = rect.y > 0 ? rect.y : 0;
        rect.height = rect.y + rect.height < this->deep_img_.rows ? rect.height : this->deep_img_.rows - rect.y - 1;

        // Get the image of the corresponding area .
        this->roi_img_ = cv::Mat::zeros(this->deep_img_.rows, this->deep_img_.cols, CV_8UC1);
        cv::Mat roi = this->deep_img_(rect);
        roi.copyTo(this->roi_img_(rect));
        this->roi_img_ = (this->roi_img_ > 0) * 255;
        //cv::bitwise_not(target_image, target_image);

        // Get the point cloud data of the corresponding region .
        const Point3D *point = nullptr;
        size_t p_index;
        for (size_t r = rect.y; r < rect.y + rect.height; ++r) {
            for (size_t c = rect.x; c < rect.x + rect.width; ++c) {
                p_index = r * this->image_width_ + c;
                point = &cloud->points[p_index];
                if (this->roi_img_(r, c)) {
                    target_point_cloud->push_back(*point, p_index);
                }
            }
        }

        // Extraction of point cloud on pallet surface .
        PointCloudPtr pallets_points(new PointCloud());
        std::vector<int> indices = this->target_.getPalletsIndices();
        for (auto const idx: indices) {
            pallets_points->add(*this->clusters_map_.at(idx).getPointCloud());
        }

        // Find the normal vector of a point.
        NormalCloud::Ptr normals(new NormalCloud());
        calculateNormalEstimation(pallets_points, normals);

        // Fit the plane, leaving a point closer to the plane .
        std::vector<float> plane_coefficients;
        Normal pallet_normal;
        std::vector<int> filter_indices;
        ransac(pallets_points, normals, filter_indices, plane_coefficients);

        float A, B, C, D, molecule, dist;
        A = plane_coefficients[0];
        B = plane_coefficients[1];
        C = plane_coefficients[2];
        D = plane_coefficients[3];
        float denominator = sqrtf(A * A + B * B + C * C);
        PointCloudPtr pallet_plane_cloud(new PointCloud());
        int r, c;
        const size_t size = target_point_cloud->points.size();
        for (size_t i = 0; i < size; ++i) {
            c = target_point_cloud->image_indices[i] % this->image_width_;
            r = target_point_cloud->image_indices[i] / this->image_width_;
            point = &target_point_cloud->points[i];
            molecule = fabsf(A * point->x + B * point->y + C * point->z + D);
            dist = molecule / denominator;
            if (dist < 0.05) {
                pallet_plane_cloud->push_back(*point, target_point_cloud->image_indices[i]);
            } else {
                this->roi_img_(r, c) = 0;
            }
        }

//    target.setNormal(Normal(A,B,C));
//    auto angle = static_cast<float>(atan2(B, A) * 180 / M_PI);
//    target.setAngle(angle);
//    LOG(INFO) << "plane equation: ax+by+cz+d=0" << " (a=" << A << " b=" << B << " c=" << C << ")";
//    LOG(INFO) << "angle = atan2(b,a)*180/M_PI = " << angle;

        copyPointCloud(pallet_plane_cloud, target_point_cloud);

        // Set structure element type, size and anchor location。
        cv::Mat elemStruct = getStructuringElement(
                cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));

        dilate(this->roi_img_, this->roi_img_, elemStruct);
        erode(this->roi_img_, this->roi_img_, elemStruct);
        dilate(this->roi_img_, this->roi_img_, elemStruct);
        erode(this->roi_img_, this->roi_img_, elemStruct);

        finish = common_func::get_timestamp_in_ms();
        int64_t generate_target_image_time = finish - start;
        LOG(INFO) << "generate target image time consuming " << generate_target_image_time << "ms";
    }

    void
    DecardsDetector::findRectangle(const DecardsDetectParam &param_,
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

        for (auto &box: boxs) {
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

    bool
    DecardsDetector::targetIsCard() {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        Card *card_ptr = &this->cards_map_[this->target_.getId()];
        bool ret = false;
        std::vector<cv::Rect> rect_vct;
        const int line_thickness = 1;
        cv::Rect card_rect;
        findMaxConnectedComponent(this->roi_img_, card_rect);
        cv::Mat img = cv::Mat::zeros(card_rect.height + line_thickness, card_rect.width, CV_8UC1);

        this->roi_img_(card_rect).copyTo(img);

        cv::Mat element_rect25x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 3));
        dilate(img, img,element_rect25x3);

        LOG(INFO) << "target is card " << card_ptr->getId() << " ?";
        for (int h = img.rows; h > 0; --h) {
            rect_vct.clear();
            line(img, cv::Point(0, h), cv::Point(img.cols, h), cv::Scalar(255), line_thickness);

            findRectangle(this->param_, img, rect_vct);

//        cv::Mat img1 = img.clone();
//        for (auto iter : rect_vct) {
//            cv::rectangle(img1, iter, cv::Scalar(150));
//        }
//        cv::imshow("img1", img1);
//        cv::waitKey(0);

            // filter by area and width-height rate .
            LOG(INFO) << "find rectangle size = " << rect_vct.size();
            float hole_width = static_cast<float>(card_ptr->getHolePixelWidth()[0]);
            float hole_height = static_cast<float>(card_ptr->getHolePixelHeight());
            float card_hole_area = hole_width * hole_height;
            LOG(INFO) << " height=" << hole_height << " hole_width=" << hole_width;
            int min_area = static_cast<int>(card_hole_area * this->param_.min_area_rate);
            int max_area = static_cast<int>(card_hole_area * this->param_.max_area_rate);
            float hole_size_rate = hole_width / hole_height;
            for (auto it = rect_vct.begin(); it != rect_vct.end();) {
                // filter area .
                int target_hole_area = it->height * it->width;
                if (target_hole_area < min_area || target_hole_area > max_area) {
                    LOG(INFO) << " area=" << std::setfill(' ') << std::setw(4) << target_hole_area
                              << " not within range [min, max]=[" << min_area << ", " << max_area << "] Drop";
                    rect_vct.erase(it);
                    continue;
                } else {
                    LOG(INFO) << " area=" << std::setfill(' ') << std::setw(4) << target_hole_area
                              << " within range [min, max:]=[" << min_area << ", " << max_area << "] OK";
                }

                // filter rate .
                float target_size_rate = static_cast<float>(it->width) / static_cast<float>(it->height);
                if (fabsf(hole_size_rate - target_size_rate) > this->param_.hole_height_rate_diff_allow) {
                    LOG(INFO) << " hole_size_rate: width=" << hole_width << " height=" << hole_height << " rate="
                              << hole_size_rate;
                    LOG(INFO) << " target_size_rate: width=" << it->width << " height=" << it->height << " rate="
                              << target_size_rate;
                    LOG(INFO) << " fabsf(rate)=" << std::setprecision(3) << fabsf(hole_size_rate - target_size_rate)
                              << " > " << this->param_.hole_height_rate_diff_allow << " Drop";
                    rect_vct.erase(it);
                } else {
                    LOG(INFO) << " hole_size_rate: width=" << hole_width << " height=" << hole_height << " rate="
                              << hole_size_rate;
                    LOG(INFO) << " target_size_rate: width=" << it->width << " height=" << it->height << " rate="
                              << target_size_rate;
                    LOG(INFO) << " fabsf(rate)=" << std::setprecision(3) << fabsf(hole_size_rate - target_size_rate)
                              << " <= " << this->param_.hole_height_rate_diff_allow << " OK";
                    ++it;
                }
            }

//        cv::Mat img2 = img.clone();
//        for (auto iter : rect_vct) {
//            cv::rectangle(img2, iter, cv::Scalar(150));
//        }
//        cv::imshow("img2", img2);

            if (rect_vct.size() == card_ptr->getHolesWidth().size()) {

                // Calculate the position of the jack in the image
                for (auto &iter: rect_vct) {
                    iter.x += card_rect.x;
                    iter.y += card_rect.y;
                }
                this->target_.setHoleRect(rect_vct);

                cv::Mat img3 = this->roi_img_.clone();
                for (const auto &iter: this->target_.getHoleRect()) {
                    cv::rectangle(img3, iter, cv::Scalar(150));
                }

                LOG(INFO) << " rect_vct.size()=" << rect_vct.size() << " = " << card_ptr->getHolesWidth().size()
                          << "=card.getHoles.size() OK";

                //            cv::imshow("img3", img3);
                //            cv::waitKey(0);

                ret = true;
                break;
            } else {
                LOG(INFO) << " rect_vct.size()=" << rect_vct.size() << " != " << card_ptr->getHolesWidth().size()
                          << "=card.getHoles.size() Drop";
            }
        }

        finish = common_func::get_timestamp_in_ms();
        int64_t target_is_card_time = finish - start;
        LOG(INFO) << "check target is card time consuming " << target_is_card_time << "ms";

        return ret;
    }

    void
    DecardsDetector::correctTargetHeight() {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        const std::vector<cv::Rect> holes = this->target_.getHoleRect();
        const std::vector<int> pallets_indices = this->target_.getPalletsIndices();
        const cv::Rect pallet_rect = this->clusters_map_.at(pallets_indices[0]).getRect();

        // Height of pier extending upward .
        const int up_extend_pixel = MAX(0, pallet_rect.y - holes[0].y);

        // Get the height of the pallet .
        Card *card_ptr = &this->cards_map_[this->target_.getId()];
        const int pallet_height = card_ptr->getPalletPixelHeight();
        LOG(INFO) << "correct card height: " << holes[0].height - 1 << " --> " << pallet_height;
        auto is_within_rect = [](const int &x, const int &y, const cv::Rect &rect) -> bool {
            if (x < rect.x || x > rect.x + rect.width) return false;
            if (y < rect.y || y > rect.y + rect.height) return false;
            return true;
        };

        //    LOG(INFO) << "pallet height=" << pallet_height
        //    << " holes[0].y=" << holes[0].y
        //    << " pallet_rect.y=" << pallet_rect.y
        //    << " add_up_pixel=" << add_up_pixel;
        cv::Mat img = cv::Mat::zeros(this->image_height_, this->image_width_, CV_8UC1);
        int x, y;
        cv::Rect rect;

        PointCloudPtr card_point_cloud(new PointCloud());
        NormalCloudPtr card_normal_cloud(new NormalCloud());
        std::vector<cv::Rect> rects;
        for (auto index: pallets_indices) {
            auto *pallet = &this->clusters_map_.at(index);

            // Correct the height of the rectangle
            rect = pallet->getRect();
            rect.y -= up_extend_pixel;
            rect.height = pallet_height;

            // Get the point cloud in the correction rectangle; .
            PointCloudPtr pallet_point_cloud(new PointCloud());
            const Point3D *point = nullptr;
            size_t p_index;
            for (size_t r = rect.y; r < rect.y + rect.height; ++r) {
                for (size_t c = rect.x; c < rect.x + rect.width; ++c) {
                    p_index = r * this->image_width_ + c;
                    point = &this->tf_frame_.cloud->points[p_index];
                    if (IS_WITHIN_SCOPE(point->x, this->param_.detect_range.min_x, this->param_.detect_range.max_x) &&
                        IS_WITHIN_SCOPE(point->y, this->param_.detect_range.min_y, this->param_.detect_range.max_y) &&
                        IS_WITHIN_SCOPE(point->z, this->param_.detect_range.min_z, this->param_.detect_range.max_z)) {
                        pallet_point_cloud->push_back(*point, p_index);
                    }
                }
            }

            // median filter
            PointCloudPtr cloud_org(new PointCloud());
            PointCloudPtr pallet_after_median_filter(new PointCloud());

            copyPointCloud(this->tf_frame_.cloud, cloud_org);
            medianFilter(this->tf_frame_.cloud, pallet_point_cloud, pallet_after_median_filter);

            for (size_t i = 0; i < pallet_after_median_filter->size(); ++i) {
                int idx = pallet_after_median_filter->image_indices[i];
                cloud_org->points[idx] = pallet_after_median_filter->points[i];
            }

            PointCloudPtr pallet_after_mean_filter(new PointCloud());
            medianFilter(cloud_org, pallet_after_median_filter, pallet_after_mean_filter);

            Normal pallet_normal;
            std::vector<int> filter_indices;
            PointCloudPtr filter_card_point_cloud(new PointCloud());
            NormalCloudPtr filter_card_normal_cloud(new NormalCloud());

            // Calculating the normal vector of point cloud .
            NormalCloud::Ptr pallet_normal_cloud(new NormalCloud);
            calculateNormalEstimation(pallet_after_mean_filter, pallet_normal_cloud);

            // Method 1: the normal vector of pier surface is obtained by fitting plane .
            /*
            std::vector<float> plane_coefficients;
            ransac(pallet_after_mean_filter, pallet_normal_cloud, filter_indices, plane_coefficients);

            pallet_normal.normal_x = plane_coefficients[0];
            pallet_normal.normal_y = plane_coefficients[1];
            pallet_normal.normal_z = plane_coefficients[2];
            */

            // Method 2: filtering through point cloud normal vector .
            float len, theta;
            pallet_normal.normal_x = 0;
            pallet_normal.normal_y = 0;
            pallet_normal.normal_z = 0;
            const size_t pallet_cloud_size = pallet_normal_cloud->size();
            const Normal *n = nullptr;
            for (size_t i = 0; i < pallet_cloud_size; ++i) {
                n = &pallet_normal_cloud->points[i];
                if (isnan(n->normal_x) || isnan(n->normal_y) || isnan(n->normal_z)) {
                    continue;
                }

                len = sqrtf(n->normal_y * n->normal_y + n->normal_z * n->normal_z);
                theta = static_cast<float>(atan2f(len, n->normal_x) * 180 / M_PI);
                theta = theta > 90 ? theta - 180 : theta;

                if (fabsf(theta) < this->param_.normal_angle_tolerance) {
                    pallet_normal.normal_x += n->normal_x;
                    pallet_normal.normal_y += n->normal_y;
                    pallet_normal.normal_z += n->normal_z;
                    filter_indices.push_back(i);
                }
            }
            LOG(INFO) << "filter_indices.size()=" << filter_indices.size();
            pallet_normal.normal_x = pallet_normal.normal_x / filter_indices.size();
            pallet_normal.normal_y = pallet_normal.normal_y / filter_indices.size();
            pallet_normal.normal_z = pallet_normal.normal_z / filter_indices.size();

            // The filtered point cloud is obtained according to the index, and the point
            // cloud of pier surface is corrected
            copyPointCloudIndices(pallet_after_mean_filter, filter_indices, filter_card_point_cloud);
            copyNormalCloudIndices(pallet_normal_cloud, filter_indices, filter_card_normal_cloud);

            LOG(INFO) << "pallet[" << index << "] size " << pallet->getSize() << "->" << filter_card_point_cloud->size()
                      << " normal: (" << pallet_normal.normal_x << ", " << pallet_normal.normal_y
                      << ", " << pallet_normal.normal_z << ")";
            pallet->setPointCloud(filter_card_point_cloud, this->image_width_);
            pallet->setNormal(pallet_normal);

            // Record point cloud data .
            card_point_cloud->add(*filter_card_point_cloud);
            card_normal_cloud->add(*filter_card_normal_cloud);
            rects.push_back(rect);
        }

        this->target_.setNormalCloud(card_normal_cloud);
        this->target_.setPointCloud(card_point_cloud);
        this->target_.setPalletRect(rects);
//    cv::imshow("img", img);
//    cv::waitKey(10);

        finish = common_func::get_timestamp_in_ms();
        int64_t correct_target_height_time = finish - start;
        LOG(INFO) << "correct target height time consuming " << correct_target_height_time << "ms";
    }

//DetectResult
//DecardsDetector::detect(const O3d3xxFrame &frame) {
//    // 1. If stack 's top is in camera FOV
//
//    int64_t start, finish;
//    bool is_find_target;
//    NormalCloud::Ptr normals(new NormalCloud());
//
//    // Clear the data stored in the last processing.
//    start = common_func::get_timestamp_in_ms();
//    this->reset();
//    finish = common_func::get_timestamp_in_ms();
//    this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//    // Copy data for processing.
//    start = common_func::get_timestamp_in_ms();
//    this->tf_frame_ = frame;
//    this->image_height_ = frame.confidence_img.rows;
//    this->image_width_ = frame.confidence_img.cols;
//    finish = common_func::get_timestamp_in_ms();
//    this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//    // Point clouds with high credibility and small amplitude are filtered out.
//    start = common_func::get_timestamp_in_ms();
//    maskFilter(this->tf_frame_.confidence_img,
//               this->tf_frame_.amplitude_img,
//               this->tf_frame_.cloud,
//               this->confidence_normalize_,
//               this->amplitude_normalize_,
//               this->filter_mask_img_,
//               this->filter_mask_cloud_);
//    finish = common_func::get_timestamp_in_ms();
//    this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//    // Generate depth imageGenerate depth image.
////    start = common_func::get_timestamp_in_ms();
////    generateDistanceImage(this->filter_mask_cloud_,this->tf_frame_.distance_img);
////    cv::normalize(this->tf_frame_.distance_img,
////                  this->deep_img_, 0, 255, cv::NORM_MINMAX);
////    finish = common_func::get_timestamp_in_ms();
////    this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//    // Filter out the points in the detection area.
//    start = common_func::get_timestamp_in_ms();
//    passThroughFilter(this->param_.detect_range,this->filter_mask_cloud_, this->filter_xyz_range_cloud_);
//    finish = common_func::get_timestamp_in_ms();
//    this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//    // Median filtering
////    start = common_func::get_timestamp_in_ms();
////    medianFilter(this->tf_frame_.cloud,this->filter_xyz_range_cloud_, this->filter_median_cloud_);
////    finish = common_func::get_timestamp_in_ms();
////    this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//    // Generate depth imageGenerate depth image.
//    start = common_func::get_timestamp_in_ms();
//    generateDistanceImage(this->filter_xyz_range_cloud_,this->tf_frame_.distance_img);
//    cv::normalize(this->tf_frame_.distance_img,
//                  this->deep_img_, 0, 255, cv::NORM_MINMAX);
//    finish = common_func::get_timestamp_in_ms();
//    this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//    // Deep image Shape trans rectangle1 Image and detect if top.
//    cv::Mat gray_image;
//    cv::Mat rect_tran_image;
//    start = common_func::get_timestamp_in_ms();
//    mvision::threshold2(this->deep_img_, gray_image, 138, 255);
//
//    // 2. While stack 's top is in camera FOV , detect top card location (x, y, z, yaw);
//    // maskFilter
//    cv::Mat dst_image;
//    cv::Mat element_rect201x11 =  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(201, 11));
//    cv::morphologyEx(gray_image, dst_image, cv::MORPH_CLOSE, element_rect201x11);
//    cv::Mat image_sub = dst_image -  gray_image ;
//    cv::Mat select_image, top_pallet_image, bottom_pallet_image;
//    cv::Mat stack_rect_image;
//
//    std::vector<mvision::SelectShapeType> shape_types;
//    shape_types.push_back(mvision::SelectShapeType::SELECT_WIDTH );
//    shape_types.push_back(mvision::SelectShapeType::SELECT_HEIGHT);
//    std::vector<double> shape_type_mins, shape_type_maxs;
//    shape_types.resize(shape_types.size());
//    shape_types.resize(shape_types.size());
//    shape_type_mins.push_back(20);
//    shape_type_mins.push_back(7);
//    shape_type_maxs.push_back(39);
//    shape_type_maxs.push_back(24);
//    int shape_count = mvision::selectShape(image_sub, select_image, shape_types,
//                                           mvision::SelectOperation::SELECT_AND,
//                                           shape_type_mins, shape_type_maxs);
//
//    LOG(INFO) << "shape_count:" << shape_count;
//
//    if(shape_count >=2) {
//        cv::Mat element_rect1x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3));
//        cv::Mat element_rect25x1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 1));
//        cv::morphologyEx(select_image, select_image, cv::MORPH_OPEN, element_rect1x3);
//        cv::morphologyEx(select_image, select_image, cv::MORPH_CLOSE, element_rect25x1);
//
//        std::vector<std::vector<cv::Point>> select_contours;
//        std::vector<int> sort_pos;
//        cv::findContours(select_image, select_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//        mvision::sortRegion(select_image, select_contours, sort_pos, mvision::SortCriterion::FIRST_POINT, false,
//                            mvision::SortDirection::ROW);
//        std::vector<std::vector<cv::Point> > top_contours, bottom_contours;
//        top_contours.push_back(select_contours[sort_pos[sort_pos.size()-1]]);
//        bottom_contours.push_back(select_contours[sort_pos[0]]);
//        top_pallet_image = select_image.clone();
//        bottom_pallet_image = select_image.clone();
//
//        cv::drawContours(top_pallet_image, top_contours, -1, cv::Scalar(0), CV_FILLED);
//        cv::drawContours(bottom_pallet_image, bottom_contours, -1, cv::Scalar(0), CV_FILLED);
//        cv::bitwise_xor(select_image, top_pallet_image, top_pallet_image);
//        cv::bitwise_xor(select_image, bottom_pallet_image, bottom_pallet_image);
//
//
//        cv::Mat element_rect1x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 5));
//        cv::morphologyEx(top_pallet_image, top_pallet_image, cv::MORPH_DILATE, element_rect25x1);
//        cv::morphologyEx(top_pallet_image, top_pallet_image, cv::MORPH_ERODE, element_rect1x5);
//        cv::morphologyEx(bottom_pallet_image, bottom_pallet_image, cv::MORPH_DILATE, element_rect25x1);
//        cv::morphologyEx(bottom_pallet_image, bottom_pallet_image, cv::MORPH_ERODE, element_rect1x5);
//
//
//
//        cv::bitwise_and(gray_image, top_pallet_image, top_pallet_image);
//        cv::bitwise_and(gray_image, bottom_pallet_image, bottom_pallet_image);
//        cv::morphologyEx(top_pallet_image, top_pallet_image, cv::MORPH_DILATE, element_rect1x5);
//        cv::morphologyEx(bottom_pallet_image, bottom_pallet_image, cv::MORPH_DILATE, element_rect1x5);
//
//        maskFilter(top_pallet_image, this->tf_frame_.cloud, this->top_card_cloud_);
//
//        // Median filtering
//        start = common_func::get_timestamp_in_ms();
//        medianFilter(this->tf_frame_.cloud,this->top_card_cloud_, this->filter_median_cloud_);
//        finish = common_func::get_timestamp_in_ms();
//        this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//        // Filtering outliers.
//        start = common_func::get_timestamp_in_ms();
//        outlierFilter(this->filter_median_cloud_, this->filter_outlier_cloud_);
//        finish = common_func::get_timestamp_in_ms();
//        this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//        // Calculating the normal vector of point cloud.
//        start = common_func::get_timestamp_in_ms();
//        calculateNormalEstimation(this->filter_outlier_cloud_, normals);
//        finish = common_func::get_timestamp_in_ms();
//        this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//        // Filter out the point cloud toward the camera.
//        start = common_func::get_timestamp_in_ms();
//        normalFilter(normals,
//                     this->filter_outlier_cloud_,
//                     this->param_.normal_angle_tolerance,
//                     this->filter_normal_point_cloud_,
//                     this->filter_normal_normal_cloud_);
//        finish = common_func::get_timestamp_in_ms();
//        this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//        // Point cloud segmentation, generating pier list.
//        start = common_func::get_timestamp_in_ms();
//        std::vector<Indices> clusters_indices;
//        euclidean<DecardsDetectParam>(this->param_,
//                                   this->filter_normal_normal_cloud_,
//                                   this->filter_normal_point_cloud_,
//                                   clusters_indices);
//        filterClusters(this->filter_normal_normal_cloud_,
//                       this->filter_normal_point_cloud_,
//                       clusters_indices,
//                       this->clusters_map_);
//        finish = common_func::get_timestamp_in_ms();
//        this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//        // Search card target.
//        start = common_func::get_timestamp_in_ms();
//        is_find_target = searchTarget();
//        finish = common_func::get_timestamp_in_ms();
//        this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
////        cv::imshow("top_pallet_image",top_pallet_image);
////        cv::imshow("bottom_pallet_image",bottom_pallet_image);
////        cv::waitKey(-1);
//        LOG(INFO) <<"here 0!";
//        if (is_find_target) {
//
//            LOG(INFO) <<"here!";
//            // Correct target width.
//            start = common_func::get_timestamp_in_ms();
//            correctTargetWidth();
//            finish = common_func::get_timestamp_in_ms();
//            this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//            // Generate target image.
//            start = common_func::get_timestamp_in_ms();
//            PointCloudPtr target_point_cloud(new PointCloud());
//            generateTargetImage(frame.cloud, target_point_cloud);
//            finish = common_func::get_timestamp_in_ms();
//            this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//            // Confirm whether the target is a pallet.
//            start = common_func::get_timestamp_in_ms();
//            if (targetIsCard()) {
//                finish = common_func::get_timestamp_in_ms();
//                this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//                LOG(INFO) <<"here2!";
//
//                // Correct target altitude.
//                start = common_func::get_timestamp_in_ms();
//                correctTargetHeight();
//                finish = common_func::get_timestamp_in_ms();
//                this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//                // Calculate target position and angle.
//                start = common_func::get_timestamp_in_ms();
//                this->result_.is_available = calculateResult();
//                finish = common_func::get_timestamp_in_ms();
//                this->consuming_time_recorder_.push_back(std::to_string(finish - start));
//
//                this->result_.id = target_.getId();
//                this->result_.x = target_.getPose().x;
//                this->result_.y = target_.getPose().y;
//                this->result_.z = target_.getPose().z;
//                this->result_.angle = target_.getAngle();
//                this->result_.width = target_.getCardWidth();
//                this->result_.height = target_.getHoleHeight();
//                this->result_.normal = target_.getNormal();
//            } else {
//                LOG(INFO) << "target is not card";
//            }
//        }
//
//        // draw shapes on the picture.
//        drawMask();
//        drawTarget();
//        drawCameraImage();
//        drawCardHole();
//        drawCardPallet();
//        gatherAllClusterCenter();
//
//        return this->result_;
//
//    }else{
//
//    }
//
//
//}



} // end of namespace perception