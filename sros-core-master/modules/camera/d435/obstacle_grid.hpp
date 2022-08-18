#pragma once

#include "math.hpp"
#include <iostream>
#include <algorithm>
#include "../stereo_point.h"
//#include "nav_msgs/OccupancyGrid.h"
//#include "ros/ros.h"

namespace obstacle_detection {
//栅格
class ObstacleGrid {
public:
    ObstacleGrid(const float resolution = 0.0025f, const float x_range = 1.6f, const float y_range = 1.2f)
            : resolution_(resolution),
              width_(x_range / resolution),
              height_(y_range / resolution),
              height_bias_(height_ / 2) {
        reset();
    }

private:
    const float resolution_;                      //分辨率
    const int width_;                             //宽度（x方向）
    const int height_;                            //高度（y方向）
    const int height_bias_;                       //偏移量（辅助计算浮点型坐标对应栅格坐标）
    std::vector<signed char> data_;               //数据（存储栅格信息）
    static constexpr signed char threshold_ = 50; //阈值（判断栅格是否为障碍物）
    static constexpr int down_size_ = 8;

public:
    //设置（用score更新（x，y）对应的栅格信息）
    void set(const float x, const float y, const float score) {
        int ix = static_cast<int>(std::floor(x / resolution_));
        int iy = static_cast<int>(std::floor(y / resolution_)) + height_bias_;
        const int value = toValue(score);
        if (ix >= 0 && ix < width_ && iy >= 0 && iy < height_) {
            if (data_[width_ * iy + ix] < value) {
                data_[width_ * iy + ix] = value;
            }
        }
    }

    //转换
    void getStereoPoints(std::shared_ptr<StereoPoints> &stereo_points) const {
        stereo_points->clear();
        for (int iy = 0; iy < height_; iy += down_size_) {
            for (int ix = 0; ix < width_; ix += down_size_) {
                getStereoPoint(ix, iy, stereo_points);
            }
        }
    }

    //重置（清空数据）
    void reset() { data_.assign(height_ * width_, -1); }

    //腐蚀
    void erode() {
        std::vector<signed char> new_data;
        new_data.assign(height_ * width_, -1);
        for (int iy = 1; iy < height_ - 1; iy++) {
            for (int ix = 1; ix < width_ - 1; ix++) {
                const int pos = width_ * iy + ix;
                new_data[pos] = std::min({data_[pos], data_[pos - 1], data_[pos + 1],
                                          data_[pos - width_], data_[pos + width_]});
            }
        }
        data_ = new_data;
    }

    //膨胀
    void dilate() {
        std::vector<signed char> new_data;
        new_data.assign(height_ * width_, -1);
        for (int iy = 1; iy < height_ - 1; iy++) {
            for (int ix = 1; ix < width_ - 1; ix++) {
                const int pos = width_ * iy + ix;
                new_data[pos] = std::max({data_[pos], data_[pos - 1], data_[pos + 1],
                                          data_[pos - width_], data_[pos + width_],
                                          data_[pos - width_ - 1], data_[pos - width_ + 1], data_[pos + width_ - 1],
                                          data_[pos + width_ + 1]});
            }
        }
        data_ = new_data;
    }

    //开运算
    void open() {
        dilate();
        dilate();
        erode();
        erode();
    }

    //闭运算
    void close() {
        erode();
        dilate();
    }

    //二值化
    void thresh() {
        for (auto &value : data_) {
            if (value > threshold_) {
                value = 100;
            } else {
                value = 0;
            }
        }
    }

private:
    //[0,1]score转换成[0,100]value
    static const int toValue(const float score) {
        if (score < 0.f)
            return 0;
        if (score > 1.f)
            return 100;
        return static_cast<int>(score * 100.f);
    }

    void getStereoPoint(const int ix, const int iy, std::shared_ptr<StereoPoints> &stereo_points) const {
        StereoPoints::Point point;
        point.z = 0;
        point.inten = 1;
        for (int iiy = iy; iiy < iy + down_size_ && iiy < height_; iiy++) {
            for (int iix = ix; iix < ix + down_size_ && iix < width_; iix++) {
                const int pos = width_ * iiy + iix;
                if (data_[pos] == 100) {
                    point.x = resolution_ * static_cast<double>(iix);
                    point.y = resolution_ * static_cast<double>(iiy - height_bias_);
                    stereo_points->points.push_back(point);
                    return;
                }
            }
        }
    }
};
} // namespace obstacle_detection
