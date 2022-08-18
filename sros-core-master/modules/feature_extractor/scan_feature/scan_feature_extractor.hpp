//
// Created by lfc on 2020/12/23.
//

#ifndef SROS_SCAN_FEATURE_EXTRACTOR_HPP
#define SROS_SCAN_FEATURE_EXTRACTOR_HPP
#include "../base_feature_info.hpp"
#include <core/msg/laser_scan_msg.hpp>
#include "feature_extracter/multi_type_feature_extracter.hpp"
#include "feature_extracter/gaussian_filter.hpp"
#include "feature_extracter/extract_para.hpp"
namespace extractor{
class ScanFeatureExtractor {
 public:
    ScanFeatureExtractor(){
        feature_para_.reset(new extract::ExtractPara);
        extractor_.reset(new features_extract::MultiTypeFeaturesExtracter(feature_para_));
    }

    virtual ~ScanFeatureExtractor() {

    }

    void addFeaturePara(const std::vector<double>& paras){

    }

    bool extract(sros::core::LaserScan_ptr &scan,std::vector<FeatureInfo_ptr>& infos,double max_dist = 3.0f){
        std::vector<features_extract::FeatureInfo_Ptr> recognized_features;
        infos.clear();
        sros::core::LaserScan_ptr cp_scan(new sros::core::LaserScanMsg);
        *cp_scan = *scan;
        extractor::GaussianFilter::filter(cp_scan);
        if(extractor_->extractCandidateFeatures(cp_scan,recognized_features)){
            for (auto& feature : recognized_features) {
                if (feature->distance < max_dist) {
                    FeatureInfo_ptr feature_info(new FeatureInfo);
                    Eigen::Vector3f feature_pose(feature->center[0], feature->center[1], atan2(feature->direction[1],feature->direction[0]));
                    feature_info->pose_in_sensor.buildTFfrom2DPose(feature_pose);
                    feature_info->code_type = TYPE_SCAN_LMK_CODE;
                    switch (feature->type) {
                        case features_extract::FeatureType::FEATURE_LMK:
                            feature_info->feature_name = "FEATURE_LMK";
                            break;
                        case features_extract::FeatureType::FEATURE_TRI:
                            feature_info->feature_name = "FEATURE_TRI";
                            feature_info->code_type = TYPE_SCAN_ANGLE_CODE;
                            break;
                        case features_extract::FeatureType::FEATURE_CORNER:
                            feature_info->feature_name = "FEATURE_CORNER";
                            feature_info->code_type = TYPE_SCAN_ANGLE_CODE;
                            break;
                        case features_extract::FeatureType::FEATURE_ARTICORNER:
                            feature_info->feature_name = "FEATURE_ARTICORNER";
                            feature_info->code_type = TYPE_SCAN_ANGLE_CODE;
                            break;
                        default:
                            LOG(INFO) << "cannot recog feature! continue!";
                            continue;
                    }
                    infos.push_back(feature_info);
                }
            }
        }
        return !infos.empty();
    }

 private:
    std::shared_ptr<features_extract::MultiTypeFeaturesExtracter> extractor_;
    extract::ExtractPara_ptr feature_para_;
};

}

#endif  // SROS_SCAN_FEATURE_EXTRACTOR_HPP
