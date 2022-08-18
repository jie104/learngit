/**
 * @file pallet.cpp
 * @brief pallet class function implementation.
 *
 * pallet class function implementation.。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/31
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
// INCLUDE
#include "cluster.h"

//#include <glog/logging.h>

// CODE
namespace perception {
void
Cluster::clear() {
    this->avg_x_ = .0f;
    this->avg_y_ = .0f;
    this->avg_z_ = .0f;

    this->rect_.x = 0;
    this->rect_.y = 0;
    this->rect_.width = 0;
    this->rect_.height = 0;

    this->width_ = .0f;
    this->height_ = .0f;

    this->normal_.normal_x = 0;
    this->normal_.normal_y = 0;
    this->normal_.normal_z = 0;

    this->most_left_point_.reset();
    this->most_right_point_.reset();
    this->most_up_point_.reset();
    this->most_down_point_.reset();

    this->point_cloud_->clear();
    this->normal_cloud_->clear();
}

void
Cluster::setPointCloud(const PointCloudConstPtr &input_cloud,
                       const int width) {

    const size_t size = input_cloud->size();
    copyPointCloud(input_cloud, this->point_cloud_);

    // find most left,up,down,right point index of point cloud.
    int x, y, min_x, max_x, min_y, max_y;
    int most_left_index, most_right_index, most_up_index, most_down_index;
    min_x = INT_MAX, max_x = INT_MIN;
    min_y = INT_MAX, max_y = INT_MIN;
    most_left_index = 0;
    most_right_index = 0;
    most_up_index = 0;
    most_down_index = 0;

    for (size_t i = 0; i < size; ++i) {
        y = this->point_cloud_->image_indices[i] / width;     // y: row
        x = this->point_cloud_->image_indices[i] % width;    // x: col
        if (x < min_x) {
            min_x = x;
            most_left_index = i;
        }

        if (x > max_x) {
            max_x = x;
            most_right_index = i;
        }

        if (y < min_y) {
            min_y = y;
            most_up_index = i;
        }
        if (y > max_y) {
            max_y = y;
            most_down_index = i;
        }
    }

//    LOG(INFO) << "min_x="<<min_x << " min_y="<< min_y << " max_x="<< max_x<< " max_y="<< max_y;
    this->most_left_point_ = this->point_cloud_->points[most_left_index];
    this->most_right_point_ = this->point_cloud_->points[most_right_index];
    this->most_up_point_ = this->point_cloud_->points[most_up_index];
    this->most_down_point_ = this->point_cloud_->points[most_down_index];

    // calculate pallet width and height.
    Point3D *pa, *pb;
    pa = &this->most_left_point_;
    pb = &this->most_right_point_;
    this->width_ = fabsf(pa->y - pb->y);

    pa = &this->most_up_point_;
    pb = &this->most_down_point_;
    this->height_ = fabsf(pa->z - pb->z);

    this->rect_.x = min_x;
    this->rect_.y = min_y;
    this->rect_.width = max_x - min_x + 1;
    this->rect_.height = max_y - min_y + 1;

    // calculate pallet average of points.
    calculatePointAvg();
}

void
Cluster::calculatePointAvg() {
    float total_x = .0f;
    float total_y = .0f;
    float total_z = .0f;

    for (auto point : this->point_cloud_->points) {
        total_x += point.x;
        total_y += point.y;
        total_z += point.z;
    }

    const size_t size = this->point_cloud_->size();

    this->avg_x_ = total_x / size;
    this->avg_y_ = total_y / size;
    this->avg_z_ = total_z / size;
}

void
Cluster::setRect(const cv::Rect &rect,
                 const int width) {

    // if new rectangle if big than cur rectangle. than the point cloud
    // does not change
    if (rect.x <= this->rect_.x &&
        rect.y <= this->rect_.y &&
        rect.x + rect.width >= this->rect_.x + this->rect_.width &&
        rect.y + rect.height >= this->rect_.y + this->rect_.height) {
        return;
    }

    // check point within new rectangle.
    auto is_with_rect = [](const int &x, const int &y, const cv::Rect &rect) -> bool {
        if (x < rect.x || x > rect.x + rect.width) return false;
        if (y < rect.y || y > rect.y + rect.height) return false;
        return true;
    };

    int x, y;
    const size_t size = this->point_cloud_->size();
    PointCloudPtr temp_points(new PointCloud());
    temp_points->reserve(size);
    for (size_t i = 0; i < size; ++i) {
        const int index = this->point_cloud_->image_indices[i];
        const Point3D *point = &(this->point_cloud_->points[i]);
        x = index % width;
        y = index / width;
        if (is_with_rect(x, y, rect)) {
            temp_points->push_back(*point, index);
        }
    }

    // update pallet point cloud.
    this->setPointCloud(temp_points, width);
    this->rect_ = rect;
}

} // end of namespace perception