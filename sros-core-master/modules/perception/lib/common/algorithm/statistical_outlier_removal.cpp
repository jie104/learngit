/**
 * @file statistical_outlier_removal.cpp
 * @brief outlier removal
 *
 * The algorithm iterates through the entire input twice:
 * During the first iteration it will compute the average distance that each point
 * has to its nearest k neighbors.
 * The value of k can be set using setMeanK().
 * Next, the mean and standard deviation of all these distances are computed in order
 * to determine a distance threshold.
 * The distance threshold will be equal to: mean + stddev_mult * stddev.
 * The multiplier for the standard deviation can be set using setStddevMulThresh().
 * During the next iteration the points will be classified as inlier or outlier if their
 * average neighbor distance is below or above this threshold respectively.
 * <br>
 * The neighbors found for each query point will be found amongst ALL points of setInputCloud(),
 * not just those indexed by setIndices().
 * The setIndices() method only indexes the points that will be iterated through as search query points.
 * <br><br>
 * For more information:
 *   - R. B. Rusu, Z. C. Marton, N. Blodow, M. Dolha, and M. Beetz.
 *     Towards 3D Point Cloud Based Object Maps for Household Environments
 *     Robotics and Autonomous Systems Journal (Special Issue on Semantic Knowledge), 2008.
 * <br><br>
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/10/28
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
// INCLUDE
#include "statistical_outlier_removal.h"


// CODE
void StatisticalOutlierRemoval::applyFilterIndices (std::vector<int> &indices) {

    // The arrays to be used
    std::vector<int> nn_indices(mean_k_);
    std::vector<float> nn_dists(mean_k_);
    std::vector<float> distances(input_->size());

    // First pass: Compute the mean distances for all points with respect to their k nearest neighbors
    int valid_distances = 0;
    for (int iii = 0; iii < input_->size(); ++iii)  // iii = input indices iterator
    {
        // Perform the nearest k search
        if (kd_tree_->nearestKSearch(input_->points[iii], mean_k_ + 1, nn_indices, nn_dists) == 0) {
            distances[iii] = 0.0;
            //LOG(INFO) << "[applyFilter] Searching for the closest "<< mean_k_ <<" neighbors failed.";
            continue;
        }

        // Calculate the mean distance to its neighbors
        double dist_sum = 0.0;
        for (int k = 1; k < mean_k_ + 1; ++k)  // k = 0 is the query point
            dist_sum += sqrt(nn_dists[k]);
        distances[iii] = static_cast<float> (dist_sum / mean_k_);
        valid_distances++;
    }

    // Estimate the mean and the standard deviation of the distance vector
    double sum = 0, sq_sum = 0;
    for (float distance : distances) {
        sum += distance;
        sq_sum += distance * distance;
    }
    double mean = sum / static_cast<double>(valid_distances);
    double variance =
        (sq_sum - sum * sum / static_cast<double>(valid_distances)) / (static_cast<double>(valid_distances) - 1);
    double stddev = sqrt(variance);

    double distance_threshold = mean + std_mul_ * stddev;

    int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator
    std::vector<int> temp_indices(input_->size());
    // Second pass: Classify the points on the computed distance threshold
    for (int iii = 0; iii < input_->size(); ++iii)  // iii = input indices iterator
    {
        // Points having a too high average distance are outliers and are passed to removed indices
        // Unless negative was set, then it's the opposite condition
        if (distances[iii] > distance_threshold) {
            continue;
        }

        // Otherwise it was a normal point for output (inlier)
        temp_indices[oii++] = iii;
    }
    indices.resize(oii);
    copy(temp_indices.begin(), temp_indices.begin()+oii, indices.begin());
}