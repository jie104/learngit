/**
 * @file euclidean_cluster_extraction.cpp
 * @brief euclidean cluster class
 *
 * Decompose a region of space into clusters based on the Euclidean distance between points
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/10/28
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "euclidean_cluster_extraction.h"

// CODE
void EuclideanClusterExtraction::extract(std::vector<Indices> &clusters){

    const size_t size = cloud_->size();

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed(size, false);

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < size; ++i)
    {
        // Skip if the point is already processed
        if (processed[i])
            continue;

        // Define a seed queue
        std::vector<unsigned int> seed_queue;
        int sq_idx = 0;

        //Add a seed
        seed_queue.push_back(static_cast<int> (i));

        processed[i] = true;

        // Traverse each seed
        while (sq_idx < static_cast<int> (seed_queue.size()))
        {
            // Search for sq_idx
            if (!kd_tree_->radiusSearch(cloud_->points[seed_queue[sq_idx]], tolerance_, nn_indices, nn_distances))//没找到近邻点就继续
            {
                sq_idx++;
                continue;
            }

            for (size_t j = 1; j < nn_indices.size(); ++j)   // nn_indices[0] should be sq_idx
            {
                if (processed[nn_indices[j]])   // Has this point been processed before ?
                    continue;

                // Perform a simple Euclidean clustering
                seed_queue.push_back (nn_indices[j]);
                processed[nn_indices[j]] = true;
            }

            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size() <= max_pts_per_cluster_) {
            Indices result;
            result.indices.resize(seed_queue.size());
            for (size_t j = 0; j < seed_queue.size(); ++j)
                result.indices[j] = seed_queue[j];

            // These two lines should not be needed: (can anyone confirm?) -FF
            std::sort(result.indices.begin(), result.indices.end());
            result.indices.erase(std::unique(result.indices.begin(), result.indices.end()), result.indices.end());

            clusters.push_back(result);   // We could avoid a copy by working directly in the vector
        }

        std::sort(clusters.rbegin (), clusters.rend (), comparePointClusters);
    }
}