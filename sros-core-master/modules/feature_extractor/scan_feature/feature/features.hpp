#pragma once
#include <vector>
#include "geometry_feature.hpp"
#include "landmark_feature.hpp"

namespace feature{
    struct FeatureContainer{
public:
    std::vector <feature::LineSegmentFeature> line_segment_features;
    std::vector <feature::CornerFeature> articorner_features;
    std::vector <feature::CornerFeature> corner_features;
    std::vector <feature::CornerFeature> triangle_features;
    std::vector <feature::LandmarkFeature> landmark_features;
    std::vector <Eigen::Vector2f> break_points;
    void clear(){
        line_segment_features.clear();
        articorner_features.clear();
        corner_features.clear();
        triangle_features.clear();
        landmark_features.clear();
        break_points.clear();
    }
    bool empty(){
        return landmark_features.empty() && corner_features.empty() && articorner_features.empty() && triangle_features.empty();
    }
};
using FeatureContainer_Ptr = std::shared_ptr<FeatureContainer>;
}