/**
 * @file cube.cpp
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2021/5/26
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include <glog/logging.h>
#include <memory>
#include <string.h>
#include <opencv2/opencv.hpp>

#include "cube.h"
#include "../../../common/cloud_memory.hpp"


#ifndef MIN
#define MIN(a,b) (a) < (b) ? (a) : (b)
#endif

#ifndef MAX
#define MAX(a,b) (a) > (b) ? (a) : (b)
#endif

#ifndef FLOAT_PRECISION
#define FLOAT_PRECISION 0.000001
#endif
// CODE
namespace perception {
Cube::Cube(const Range3D<float> range3d, const float x_step, const float y_step, const float z_step)
    : range3d_(range3d), x_step_(x_step), y_step_(y_step), z_step_(z_step), x_size_(0), y_size_(0), z_size_(0) {
    auto init_dim = [](std::vector<StepInfo> &dim, const int size, const float start, const float step) {
        dim.resize(size);
        for (size_t i = 0; i < size; ++i) {
            dim[i].begin = start + i * step;
            dim[i].end = dim[i].begin + step;
            dim[i].count = 0;
        }
    };

    x_size_ = std::floor((range3d.max_x - range3d.min_x) / x_step) + 1;
    y_size_ = std::floor((range3d.max_y - range3d.min_y) / y_step) + 1;
    z_size_ = std::floor((range3d.max_z - range3d.min_z) / z_step) + 1;

    init_dim(x_dim_, x_size_, range3d.min_x, x_step);
    init_dim(y_dim_, y_size_, range3d.min_y, y_step);
    init_dim(z_dim_, z_size_, range3d.min_z, z_step);

    boundary_.min_x = boundary_.min_y = boundary_.min_z = INT_MAX;
    boundary_.max_x = boundary_.max_y = boundary_.max_z = INT_MIN;

    grid_map_ = std::unique_ptr<Cell3D[]>(new Cell3D[x_size_ * y_size_ * z_size_]);

    front_index_map_ = std::unique_ptr<Cell2D[]>(new Cell2D[y_size_ * z_size_]);

    int count = y_size_ * z_size_;
    for (int i = 0; i < count; ++i) {
        front_index_map_[i].min_x = INT_MAX;
        front_index_map_[i].idx_in_cloud = -1;
        front_index_map_[i].distance = .0f;
    }
}

Cube::~Cube() {}

size_t Cube::calcIndex(const size_t x, const size_t y, const size_t z) {
    return (x * (y_size_ * z_size_) + z * y_size_ + y);
}

size_t Cube::calcIndex(const Node &cell) {
    return (cell.coordinate.x() * (y_size_ * z_size_) + cell.coordinate.z() * y_size_ + cell.coordinate.y());
}

double distance(const double x, const double y, const double z) { return sqrtf(x * x + y * y + z * z); }

void Cube::init(const PointCloudConstPtr &cloud, int &image_height, int &image_width) {
    image_height = z_size_;
    image_width = y_size_;

    // 将点云按设置尺寸投影到珊格内。
    size_t point_size = cloud->points.size();
    auto const *points = &cloud->points;
    auto const *indices = &cloud->image_indices;
    occupy_vct_.reserve(point_size);
    for (size_t i = 0; i < point_size; ++i) {
        // 更新点在点云中的索引, 认为立方体内为整个点云。
        //        cloud->image_indices[i] = i;
        int x_index = (points->at(i).x - range3d_.min_x) / x_step_;
        int y_index = fabs((points->at(i).y - range3d_.max_y)) / y_step_;
        int z_index = fabs((points->at(i).z - range3d_.max_z)) / z_step_;

        // 找包围点云的最小立方体
        boundary_.min_x = MIN(x_index, boundary_.min_x);
        boundary_.max_x = MAX(x_index, boundary_.max_x);
        boundary_.min_y = MIN(y_index, boundary_.min_y);
        boundary_.max_y = MAX(y_index, boundary_.max_y);
        boundary_.min_z = MIN(z_index, boundary_.min_z);
        boundary_.max_z = MAX(z_index, boundary_.max_z);

        // 记录被占用的珊格对应点在点云中的索引,统计落入珊格内的点的数量.
        size_t index_in_grid_map = calcIndex(x_index, y_index, z_index);
        if (0 == grid_map_[index_in_grid_map].number) {
            occupy_vct_.push_back(Node(i, x_index, y_index, z_index));
        }
        grid_map_[index_in_grid_map].idx_in_cloud = i;
        grid_map_[index_in_grid_map].number += 1;

        // 记录yoz面内x最小的的点索引.
        size_t front_index = z_index * y_size_ + y_index;
        Cell2D *front_grid = &this->front_index_map_[front_index];
        if (x_index < front_grid->min_x) {
            front_grid->min_x = x_index;
            front_grid->idx_in_cloud = i;
            front_grid->distance = distance(points->at(i).x, points->at(i).y, points->at(i).z);
        }

        // 统计点云投影到x,y,z三个轴在每个区间的数量.
        x_dim_[x_index].count++;
        y_dim_[y_index].count++;
        z_dim_[z_index].count++;
    }

    //    showDimStatistic();
}

void Cube::showDimStatistic() {
    auto show_dim_statistic = [](const std::string title, const std::vector<StepInfo> &dim) {
        LOG(INFO) << title << ":";
        for (auto const &step : dim) {
            LOG(INFO) << "  [" << std::showpoint << step.begin << ", " << std::showpoint << step.end
                      << "]  count=" << step.count;
        }
    };
    show_dim_statistic("x_dim", x_dim_);
    show_dim_statistic("y_dim", y_dim_);
    show_dim_statistic("z_dim", z_dim_);
}

int Cube::findFirstPeak(const float rate_threshold) {
    if (z_dim_.empty()) {
        LOG(INFO) << "z_dim is empty";
        return -1;
    }
    const size_t size = z_dim_.size();
    int total = 0;
    for (int i = 0; i < size; ++i) {
        total += z_dim_[i].count;
    }
    int avg = total / size;

    std::stringstream string_buff;
    string_buff << std::endl;
    float diff = .0f, rate = .0f, max_rate = .0f;
    int max_index = -1;
    for (int i = 1; i < size; ++i) {
        diff = z_dim_[i].count - z_dim_[i - 1].count;
        if (0 == diff) continue;
        rate = diff / z_dim_[i - 1].count;
        if ((rate > rate_threshold || z_dim_[i].count > avg) && rate > max_rate && z_dim_[i].count / avg < 2.0f) {
            max_rate = rate;
            max_index = i;
        }
        string_buff << "i=" << std::setw(3) << i << " base=" << std::setw(4) << std::setfill(' ') << z_dim_[i].count
                    << ((z_dim_[i].count > avg) ? ">" : "<") << std::setw(3) << std::setfill(' ') << avg
                    << " diff=" << std::setw(4) << static_cast<int>(diff) << " rate=" << std::fixed
                    << std::setprecision(2) << std::setfill(' ') << std::setw(6) << rate << "<" << rate_threshold
                    << std::endl;
    }

    LOG(INFO) << string_buff.str();
    return max_index;
}

void Cube::showVerticalFilter(const int start, const int end) {
    const size_t size = z_dim_.size();
    int total = 0, select_total = 0;
    for (int i = 0; i < size; ++i) {
        total += z_dim_[i].count;
        if (i >= start && i < end) {
            select_total += z_dim_[i].count;
        }
    }
    int avg = total / size;

    std::stringstream string_buff;
    float diff = .0f, rate = .0f;
    const int mid = (end + start) / 2;
    string_buff << std::endl;
    for (int i = 1; i < size; ++i) {
        diff = z_dim_[i].count - z_dim_[i - 1].count;
        rate = diff / z_dim_[i - 1].count;
        string_buff << "i=" << std::setw(2) << i << " base=" << std::setw(4) << std::setfill(' ') << z_dim_[i].count
                    << ((z_dim_[i].count > avg) ? ">" : "<") << std::setw(3) << std::setfill(' ') << avg
                    << " diff=" << std::setw(4) << static_cast<int>(diff) << " rate=" << std::fixed
                    << std::setprecision(2) << std::setfill(' ') << std::setw(6) << rate;
        if (i == start || i == end) {
            string_buff << " <---" << std::endl;
        } else if ((i > start && i < mid) || (i > mid && i < end)) {
            string_buff << "    |" << std::endl;
        } else if (i == mid) {
            string_buff << "    |-->total=" << select_total << std::endl;
        } else {
            string_buff << std::endl;
        }
    }
    LOG(INFO) << string_buff.str();
}

bool Cube::verticalFilterByFirstPeak(const float height, const float rate_threshold, cv::Mat &mask) {
    int base = findFirstPeak(rate_threshold);
    if (-1 == base) {
        return false;
    }

    const int section_length = height / z_step_;
    int end = base + section_length;
    end = end < z_size_ - 1 ? end : z_size_ - 1;

    // print height search interval.
    showVerticalFilter(base, end);

    // filter out boundry points;
    end -= 1;
    base += 1;

    // Construction the mask graph within the height search interval.
    mask = cv::Mat::zeros(z_size_, y_size_, CV_8UC1);
    for (int r = base; r < end; ++r) {
        for (int c = boundary_.min_y; c < boundary_.max_y; ++c) {
            int idx = r * y_size_ + c;
            if (INT_MAX != front_index_map_[idx].min_x) {
                mask.at<uint8_t>(r, c) = 255;
            }
        }
    }
    return true;
}

void Cube::generateMaskImage(cv::Mat &mask) {
    mask = cv::Mat::zeros(z_size_, y_size_, CV_8UC1);
    for (int r = 0; r < z_size_; ++r) {
        for (int c = 0; c < y_size_; ++c) {
            int idx = r * y_size_ + c;
            if (INT_MAX != front_index_map_[idx].min_x) {
                mask.at<uint8_t>(r, c) = 255;
            }
        }
    }

    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat morphology_mask = mask.clone();

    //    erode(morphology_mask, morphology_mask, kernel);  //腐蚀
    //    dilate(morphology_mask, morphology_mask, kernel);   //膨胀
    //    dilate(morphology_mask, morphology_mask, kernel);   //膨胀
    //    cv::imwrite("/home/zx/Desktop/mask.jpg", mask);
    //    cv::imwrite("/home/zx/Desktop/morphology_mask.jpg", morphology_mask);
}

void Cube::generateDeepImage(cv::Mat &deep) {
    cv::Mat mask = cv::Mat::zeros(z_size_, y_size_, CV_8UC1);
    for (int i = 0; i < z_size_; ++i) {
        for (int j = 0; j < y_size_; ++j) {
            int idx = i * y_size_ + j;
            if (INT_MAX != front_index_map_[idx].min_x) {
                mask.at<uint8_t>(i, j) = front_index_map_[idx].distance;
            }
        }
    }
//    cv::normalize(mask, deep, 0, 255, cv::NORM_MINMAX);
    mask.convertTo(deep, CV_8UC1,255/(10.0),-0.0);
}

int Cube::getAroundCellNumber(const std::unique_ptr<Cell3D[]> &grid_map, const Node &cell, const Kernel kernel) {
    // 判断扫描立方体的边界
    size_t s_x = MAX(cell.coordinate.x() - kernel.x_radius, 0);
    size_t e_x = MIN(cell.coordinate.x() + kernel.x_radius, x_size_);
    size_t s_y = MAX(cell.coordinate.y() - kernel.y_radius, 0);
    size_t e_y = MIN(cell.coordinate.y() + kernel.y_radius, y_size_);
    size_t s_z = MAX(cell.coordinate.z() - kernel.z_radius, 0);
    size_t e_z = MIN(cell.coordinate.z() + kernel.z_radius, z_size_);

    // 统计立方体内点的数量
    int count = 0;
    for (size_t x = s_x; x < e_x; ++x) {
        for (size_t y = s_y; y < e_y; ++y) {
            for (size_t z = s_z; z < e_z; ++z) {
                int idx = calcIndex(x, y, z);
                count += grid_map[idx].number;
            }
        }
    }
    return count;
}

void Cube::outlierFilter(const PointCloudConstPtr &input_cloud, const PointCloudPtr &output_cloud) {
    Kernel kernel(1, 1, 1);
    const int KERNEL_SIZE = kernel.size() / 10;
    output_cloud->reserve(occupy_vct_.size());
    int effective_count = 0;
    int index = 0;
    for (auto &cell : occupy_vct_) {
        if (false == cell.validity) {
            continue;
        }
        auto const count = getAroundCellNumber(grid_map_, cell, kernel);
        if (count > KERNEL_SIZE) {
            int index_in_point_cloud = cell.index;
            output_cloud->push_back(input_cloud->points[index_in_point_cloud], index_in_point_cloud);
            ++effective_count;
        } else {
            const_cast<Node &>(cell).validity = false;
        }
    }

    LOG(INFO) << "outlier filter: cloud->point.size()=" << input_cloud->points.size() << "->"
              << output_cloud->points.size() << " occupy_set.size()=" << occupy_vct_.size() << "->" << effective_count;
}

std::vector<Node> &Cube::getOccupyVector() { return this->occupy_vct_; }

std::unique_ptr<Cell3D[]> &Cube::getGridMap() { return this->grid_map_; }
}