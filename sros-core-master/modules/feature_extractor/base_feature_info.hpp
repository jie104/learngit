//
// Created by lfc on 2020/12/23.
//

#ifndef SROS_BASE_FEATURE_INFO_HPP
#define SROS_BASE_FEATURE_INFO_HPP
#include <core/tf/TransForm.h>
#include <memory>

namespace extractor{
struct FeatureInfo{
    int64_t time;
    slam::tf::TransForm pose_in_sensor;
    std::string feature_name;//对于二维码而言，feature_name等于code_id;
    std::string sensor_name;
    int code_type = TYPE_SCAN_LMK_CODE; //为了和定位模块对code识别做对齐。所有识别的位置信息，均需要做编码
    std::vector<sros::core::Location> points;//可能后期会有3D点云，slam融合主要识别pose信息
};

typedef std::shared_ptr<FeatureInfo> FeatureInfo_ptr;
}



#endif  // SROS_BASE_FEATURE_INFO_HPP
