//
// Created by jintongxing on 2020/2/14.
//

#ifndef STATIONCONFIRM_STATION_CONFIRM_H
#define STATIONCONFIRM_STATION_CONFIRM_H
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "feature_extracter.hpp"

namespace features_extract {
    /*特征类型：信标、三角形、墙角*/
    enum class FeatureType {
        FEATURE_LMK = 1,
        FEATURE_TRI = 2,
        FEATURE_CORNER = 3,
        FEATURE_ARTICORNER = 4,
    };
    /*特征，包括特征的类型、位置、相对于雷达的方位角和径向距离*/
    struct FeatureInfo {
        FeatureType type;
        Eigen::Vector2f center;
        float angle;//在雷达坐标系中的方位角
        float distance;//距离
        Eigen::Vector2f direction;//特征的方向向量，例如角平分线的方向

        //把不同类型特征的基本属性统一保存成该种类型
        FeatureInfo(const feature::CornerFeature& corner){
            angle = std::atan2(corner.coordinate_(1), corner.coordinate_(0));
            distance = corner.coordinate_.norm();
            center = corner.coordinate_;
            direction = corner.direction_;
        }
        FeatureInfo(const feature::LandmarkFeature& landmark){
            type = FeatureType::FEATURE_LMK;
            angle = landmark.angle_;
            distance = landmark.distance_;
            center = landmark.coordinate_;
        }
    };
//    typedef std::shared_ptr<FeatureInfo> FeatureInfo_Ptr;
    using FeatureInfo_Ptr = std::shared_ptr<FeatureInfo>;

    class MultiTypeFeaturesExtracter {
    private:
        extract::FeatureExtracter_Ptr ex_ptr_;
        feature::FeatureContainer_Ptr feature_container_ptr_;//保存提取出的各种类型特征

    public:
        /*初始化函数
         * 输入：
         * ex_para 指向特征提取所需参数结构体的智能指针
         * */
        explicit MultiTypeFeaturesExtracter(extract::ExtractPara_ptr ex_para) {
            ex_ptr_.reset(new extract::FeatureExtracter(ex_para));//智能指针的初始化
            feature_container_ptr_.reset(new feature::FeatureContainer);
        };

        /*站点初始化时提取所有类型候选特征点
        * 输入：
        * scan: 单帧激光
        * candidate_features： 返回候选特征的智能指针的数组
        * 输出：
        * 若提取不到任何候选特征，返回false，需要粘贴信标;否则，返回true.
        * */
        bool extractCandidateFeatures(const sros::core::LaserScan_ptr& scan, std::vector<FeatureInfo_Ptr> &candidate_features) {
            candidate_features.clear();
            feature_container_ptr_->clear();
            ex_ptr_->extract(scan, feature_container_ptr_);
            if (feature_container_ptr_->empty()) {
//                std::cout << "No features extracted while extract candidate features..." << std::endl;
                return false;
            } else {
                for (const auto &landmark : feature_container_ptr_->landmark_features) {
                    FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(landmark);
                    candidate_features.push_back(feature_ptr);
                }
                for (const auto &triangle : feature_container_ptr_->triangle_features) {
                    FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(triangle);
                    feature_ptr->type = FeatureType::FEATURE_TRI;
                    candidate_features.push_back(feature_ptr);
                }
                for (const auto &corner : feature_container_ptr_->corner_features) {
                    FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(corner);
                    feature_ptr->type = FeatureType::FEATURE_CORNER;;
                    candidate_features.push_back(feature_ptr);
                }
                for (const auto &articorner : feature_container_ptr_->articorner_features) {
                    FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(articorner);
                    feature_ptr->type = FeatureType::FEATURE_ARTICORNER;
                    candidate_features.push_back(feature_ptr);
                }
                return true;
            }
        }

        /*提取指定类型特征点
        * 输入：
        * scan: 单帧激光数据
        * type: 指定的需要提取的特征类型
         * extracted_features: 返回提取到的该种类型特征的智能指针的数组
         * 返回：
         * 提取不到对应类型的特征，返回false;否则返回true.
        * */
        bool extractFeaturesWithType(const sros::core::LaserScan_ptr& scan, FeatureType type,std::vector<FeatureInfo_Ptr> &extracted_features) {
            feature_container_ptr_->clear();
            ex_ptr_->extract(scan, feature_container_ptr_);
            bool is_found = false;
            switch (type) {
                case FeatureType::FEATURE_LMK:
                    if (feature_container_ptr_->landmark_features.empty()) {
                        std::cout << "No landmarks!" << std::endl;
                    } else {
                        for (const auto &landmark : feature_container_ptr_->landmark_features) {
                            FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(landmark);
                            extracted_features.push_back(feature_ptr);
                        }
                        is_found = true;
                    }
                    break;
                case FeatureType::FEATURE_TRI:
                    if (feature_container_ptr_->triangle_features.empty()) {
                        std::cout << "No triangles! " << std::endl;
                    } else {
                        for (const auto &triangle : feature_container_ptr_->triangle_features) {
                            FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(triangle);
                            feature_ptr->type = FeatureType::FEATURE_TRI;
                            extracted_features.push_back(feature_ptr);
                        }
                        is_found = true;
                    }
                    break;
                case FeatureType::FEATURE_CORNER:
                    if (feature_container_ptr_->corner_features.empty()) {
                        std::cout << "No corners!" << std::endl;
                    } else {
                        for (const auto &corner : feature_container_ptr_->corner_features) {
                            FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(corner);
                            feature_ptr->type = FeatureType::FEATURE_CORNER;
                            extracted_features.push_back(feature_ptr);
                        }
                        is_found = true;
                    }
                    break;
                case FeatureType::FEATURE_ARTICORNER:
                    if (feature_container_ptr_->articorner_features.empty()) {
                        std::cout << "No corners!" << std::endl;
                    } else {
                        for (const auto &articorner : feature_container_ptr_->articorner_features) {
                            FeatureInfo_Ptr feature_ptr = std::make_shared<FeatureInfo>(articorner);
                            feature_ptr->type = FeatureType::FEATURE_ARTICORNER;
                            extracted_features.push_back(feature_ptr);
                        }
                        is_found = true;
                    }
                    break;
                default:
                    std::cout << "wrong type!" << std::endl;
            }
            return is_found;
        }
    };
}
#endif //STATIONCONFIRM_STATION_CONFIRM_H
