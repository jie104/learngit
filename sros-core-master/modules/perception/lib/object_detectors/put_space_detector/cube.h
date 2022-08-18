/**
 * @file cube.h
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2021/5/26
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __CUBE_H__
#define __CUBE_H__

// INCLUDE
#include <set>
#include <unordered_map>

#include "../base/param_base.hpp"
#include "../../../common/point_cloud.hpp"

//CODE
namespace perception {
struct StepInfo {
    float begin;
    float end;
    int count = 0;
};

struct Cell3D {
    size_t number = 0;
    size_t idx_in_cloud;
};

struct Cell2D {
    int min_x;
    size_t idx_in_cloud;
    float distance;
};

struct Node {
    Node(const int index, const int x, const int y, const int z) : index(index), coordinate(x, y, z), validity(true) {}

    int index;
    Eigen::Vector3i coordinate;
    bool validity;
};

struct Kernel {
    Kernel(const int x, const int y, const int z) : x_radius(x), y_radius(y), z_radius(z) {}
    int size() { return (x_radius * 2 + 1) * (y_radius * 2 + 1) * (z_radius * 2 + 1); }
    int x_radius;
    int y_radius;
    int z_radius;
};

/**
 * @description : TODO
 * @author      : zhangxu
 * @date        : 2021/5/26 上午10:38
 */
class Cube {
 public:
    typedef std::shared_ptr<Cube> Ptr;
    typedef std::shared_ptr<const Cube> ConstPtr;

    Cube(const Range3D<float> range3d, const float x_step, const float y_step, const float z_step);

    ~Cube();

    void init(const PointCloudConstPtr &cloud, int &image_height, int &image_width);

    int findFirstPeak(const float rate_threshold);

    bool verticalFilterByFirstPeak(const float height, const float rate_threshold, cv::Mat &mask);

    std::vector<Node> &getOccupyVector();

    std::unique_ptr<Cell3D[]> &getGridMap();

    size_t calcIndex(const size_t x, const size_t y, const size_t z);

    size_t calcIndex(const Node &cell);

    void showVerticalFilter(const int start, const int end);

    // 打印点云投影到x,y,z三个轴在每个区间统计的数量.
    void showDimStatistic();

    // 根据点云生成生成深度图像.
    void generateDeepImage(cv::Mat &deep);

    // 根据点云生成二值图像.
    void generateMaskImage(cv::Mat &mask);

    // 访问基准格周边一定范围内的格子,如果格子中有点,就把这些点在点云中的数量.
    int getAroundCellNumber(const std::unique_ptr<Cell3D[]> &grid_map, const Node &cell, const Kernel kernel);
    // 过滤空间内的孤立点.
    void outlierFilter(const PointCloudConstPtr &input_cloud, const PointCloudPtr &output_cloud);

    // private:
    Range3D<float> range3d_;
    float x_step_;
    float y_step_;
    float z_step_;
    size_t x_size_;
    size_t y_size_;
    size_t z_size_;
    Range3D<int> boundary_;
    std::vector<StepInfo> x_dim_;
    std::vector<StepInfo> y_dim_;
    std::vector<StepInfo> z_dim_;
    std::unique_ptr<Cell3D[]> grid_map_;
    std::unique_ptr<Cell2D[]> front_index_map_;  // 记录沿x方向最前方的点索引.
    std::vector<Node> occupy_vct_;
};

using CubePtr = Cube::Ptr;
using CubeConstPtr = Cube::ConstPtr;
}
#endif //PERCEPTION_SOLUTION_CUBE_H
