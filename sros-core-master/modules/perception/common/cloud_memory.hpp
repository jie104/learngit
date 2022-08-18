//
// Created by zx on 2020/12/9.
//

#ifndef PERCEPTION_SOLUTION_CLOUD_MEMORY_HPP
#define PERCEPTION_SOLUTION_CLOUD_MEMORY_HPP

#include <math.h>
#include "point_cloud.hpp"

/**
     * @brief copy point cloud data and point index vector.
     * @param[in] input source point cloud.
     * @param[out] output copied point clouds.
     */
static void
copyPointCloud(const PointCloudConstPtr &input,
               const PointCloudPtr &output ) {
    output->points.resize(input->size());
    output->image_indices.resize(input->size());
    copy(input->points.begin(), input->points.end(), output->points.begin());
    copy(input->image_indices.begin(), input->image_indices.end(), output->image_indices.begin());
}

/**
 * @brief copy normal cloud data and point index vector.
 * @param[in] input source normal cloud.
 * @param[out] output copied normal clouds.
 */
static void
copyNormalCloud(const NormalCloudConstPtr &input,
                const NormalCloudPtr &output) {
    output->points.resize(input->size());
    output->image_indices.resize(input->size());
    copy(input->points.begin(), input->points.end(), output->points.begin());
    copy(input->image_indices.begin(), input->image_indices.end(), output->image_indices.begin());
}

/**
 * @brief copy candidate points in point cloud.
 * @param[in] input source point cloud.
 * @param[in] indices index list of candidate points in point cloud.
 * @param[out] output candidate point cloud.
 */
static void
copyPointCloudIndices(const PointCloudConstPtr &input,
                      const std::vector<int> &indices,
                      const PointCloudPtr &output) {
    output->resize(indices.size());
    int count = 0;
    for (auto idx : indices) {
        output->points[count] = input->points.at(idx);
        output->image_indices[count] = input->image_indices.at(idx);
        count++;
    }
}

/**
 * @brief copy candidate normal in normal cloud.
 * @param[in] input source normal cloud.
 * @param[in] indices index list of candidate points in normal cloud.
 * @param[out] output candidate normal cloud.
 */
static void
copyNormalCloudIndices(const NormalCloudConstPtr &input,
                       const std::vector<int> &indices,
                       const NormalCloudPtr &output) {
    output->resize(indices.size());
    int count = 0;
    for (auto idx : indices) {
        output->points[count] = input->points.at(idx);
        output->image_indices[count] = input->image_indices.at(idx);
        count++;
    }
}

/**
 * @brief Get all valid points in the point cloud.
 * @param[in] input source point cloud.
 * @param[out] output valid point cloud.
 * @param[out] indices index of valid points in point cloud
 */
static void
getEffectivePoint(const PointCloudConstPtr &input,
                  const PointCloudPtr& output,
                  std::vector<int> &indices) {
    indices.reserve(input->points.size());
    for (size_t i = 0; i < input->points.size(); ++i) {
        if (isnan(input->points[i].x) || isnan(input->points[i].y) || isnan(input->points[i].z))
            continue;
        output->points.push_back(input->points[i]);
        indices.push_back(i);
    }
}

/**
 * @brief All candidate points are extracted from the point cloud
 * @param[in] input source point cloud.
 * @param[in] indices Index of output point cloud in input point cloud
 * @param[out] output candidate points
 */
static void
getIndicesCloud(const PointCloudConstPtr& input,
                const std::vector<int> &indices,
                PointCloudPtr output) {

    if (input == output) return;

    copyPointCloud(input, output);

    for (auto &point : output->points) {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
    }

    for (auto const &index: indices) {
        output->points[index] = input->points[index];
    }
}


#endif //PERCEPTION_SOLUTION_CLOUD_MEMORY_HPP
