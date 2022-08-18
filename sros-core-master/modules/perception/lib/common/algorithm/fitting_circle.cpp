/**
 * @file fitting_circle.cpp
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/19
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "fitting_circle.h"
#include <glog/logging.h>
#include <random>
#include <iostream>
#include <chrono>

#define MIN_VALUE 1e-8
#define IS_DOUBLE_ZERO(d)  (abs(d) < MIN_VALUE)

bool FittingCircle::circleLeastFit(const std::vector<Point2D> &points,
                                   const std::vector<int> &indices,
                                   double &center_x,
                                   double &center_y,
                                   double &radius){
    center_x = 0.0f;
    center_y = 0.0f;
    radius = 0.0f;
    if (points.size() < 3){
        return false;
    }

    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

    int N = indices.size();
    for (int i = 0; i < N; i++)
    {
        double x = points[indices[i]].x;
        double y = points[indices[i]].y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;

    if (IS_DOUBLE_ZERO(C * G - D * D)){
        return false;
    }

    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    center_x = a / (-2);
    center_y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;
    return true;
}

// CODE
void
FittingCircle::fitting(std::vector<int> &indices,
                       std::vector<float> &coefficients) {
    const int size = points_.size();
    if(size < 3) {
        return;
    }

    int N = 3;

    indices.resize(0);

    // 随机数种子生成器
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 gen(seed);  // 大随机数
    std::uniform_int_distribution<int> dis(0, size-1);  // 给定范围[0, size-1]
    int *flag = new int[size];

    auto getRandInner = [&](const int N, std::vector<int> &inner) {
        int number;
        memset(flag, 0 , size*sizeof(int));
        inner.resize(N);
        size_t i = 0, j= 0;
        while(i < N){
            number = dis(gen); // 随机生成一个点
            if (!flag[number]) {
                flag[number] = 1;
                inner[j] = number;
                j++;
                i++;
            }
        }
    };


    size_t size_threshold_ = size * this->point_rate_;
    Eigen::Vector4f xyz_centroid;
    std::vector<int> inner_indices;
    double c_x, c_y, r;
    for (size_t i = 0; i < max_iterations_; ++i) {

        // 产生一组[0～size-1]的随机数当成局内点
        getRandInner(N, inner_indices);

        // 计算这组随机点所组成的平面
        circleLeastFit(points_, inner_indices, c_x, c_y, r);

        // 把符合平面方程的局外点加入到局内点
        for (size_t j = 0; j < size; ++j){

            // 如果当前点已经是局内点则跳过,否则判断该点是否符合平面模型。
            if (flag[j]) continue;

            // 如果点到平面的距离大于阀值则放弃
            Point2D temp(c_x,c_y);
            float dist = points_[j] - temp;
            if (fabs(dist - r) > distance_threshold_){
                continue;
            }

            inner_indices.push_back(j);
            flag[j] = 1;
        }

//        std::cout << "i = " << i << " fit size=" << inner_indices.size();

        // 选择数量最多的一组局内点
        if (inner_indices.size() > indices.size()){
            indices.resize(inner_indices.size());
            copy(inner_indices.begin(), inner_indices.end(), indices.begin());
//            std::cout << " update max size=" << indices.size() << std::endl;
        } else {
//            std::cout << std::endl;
        }

        // 如果局内点的数量达到要求,则中止迭代.
        if (inner_indices.size() > size_threshold_)
            break;
    }

    delete []flag;

    // 所有局内点重新拟合
    circleLeastFit(points_, indices, c_x, c_y, r);
//    std::cout << "max fit size = " << indices.size() << std::endl;

    coefficients.resize(3);
    coefficients[0] = c_x;
    coefficients[1] = c_y;
    coefficients[2] = r;
}

// CODE