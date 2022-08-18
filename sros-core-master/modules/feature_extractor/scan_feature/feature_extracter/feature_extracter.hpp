#pragma once

#include "geometry_feature_extracter.hpp"
#include "landmark_feature_extracter.hpp"
#include "../feature/features.hpp"
//#include <chrono>

namespace extract {
class FeatureExtracter: public GeometryFeatureExtracter, public LandmarkFeatureExtracter {
public:
    explicit FeatureExtracter(ExtractPara_ptr para)
      : BaseFeatureExtracter(para), GeometryFeatureExtracter(para), LandmarkFeatureExtracter(para) {

  }
    virtual bool extract(const sros::core::LaserScan_ptr& scan, feature::FeatureContainer_Ptr &features){//override显式声明需要重写
//        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

        LandmarkFeatureExtracter::extract(scan, features);
        GeometryFeatureExtracter::extract(scan, features);//提取几何特征，保证线段和角点（墙角和三角形）

//        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//        std::chrono::steady_clock::duration used_time = end-start;
//        std::cout << "used time: " << std::chrono::duration_cast<std::chrono::milliseconds>(used_time).count() << std::endl;
        return true;
  }
};

using FeatureExtracter_Ptr = std::shared_ptr<FeatureExtracter>;
}