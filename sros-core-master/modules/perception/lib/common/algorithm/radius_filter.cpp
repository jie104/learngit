/**
 * @file radius_filter.cpp
 * @brief outlier removal
 * @author wuchaohuo@standard-robots.com
 * @date create date：2022/04/25
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
// INCLUDE
#include "radius_filter.h"


// CODE
void RadiusFilter::applyFilterIndices (std::vector<int>& indices) {

    if (search_radius_ == 0.0)
    {
        indices.clear ();
        return;
    }
    
    // The arrays to be used
    std::vector<int> nn_indices ;
    std::vector<float> nn_dists ;
    
    int oii = 0, rii = 0;  // oii = output indices iterator, rii = removed indices iterator

    // Note: k includes the query point, so is always at least 1
    int mean_k = min_pts_radius_ + 1;
    double nn_dists_max = search_radius_ * search_radius_;
    std::vector<int> temp_indices(input_->size());

    for (int iii = 0; iii < input_->size(); ++iii)
    {
      // Perform the nearest-k search
      int k = kd_tree_->radiusSearch (input_->points[iii], search_radius_, nn_indices, nn_dists);
      //int k = kd_tree_->nearestKSearch(input_->points[iii], mean_k_, nn_indices, nn_dists);

      // Points having too few neighbors are outliers and are passed to removed indices
      // Unless negative was set, then it's the opposite condition
      if (k <= min_pts_radius_)
      {
          continue;
      }

      // Otherwise it was a normal point for output (inlier)
      temp_indices[oii++] = iii;
    }

    indices.resize(oii);
    copy(temp_indices.begin(), temp_indices.begin()+oii, indices.begin());
}