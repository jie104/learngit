//
// Created by lfc on 17-9-25.
//

#ifndef PROJECT_BASE_RECOGNITION_HPP
#define PROJECT_BASE_RECOGNITION_HPP

#include <vector>
#include <memory>
#include <list>
#include <Eigen/Dense>
#include <iostream>
//#include "fitting/fitting_processor.hpp"

namespace fitting{
class FeatPara;
}
namespace recog {


struct Point {
    double x = 0;
    double y = 0;
    float intensity =0;
};
struct FeatureCluster {
    std::vector<Point> points;
    int id = 0;
};

struct FeatureClusters {
    std::vector<FeatureCluster> clusters;

    int getSize() {
        return clusters.size();
    }
};

struct RackRecogInfo {
    float intensity_thresh = 800;
    float split_dist = 0.05;
    int num_count = 5;
    float rack_length = 1.02;
    float rack_width = 0.569;
};

struct ChargingRecogInfo{
    float split_dist = 0.05;
    double charging_pile_width = 0.38;
    float intensity_thresh = 800;
    int num_count = 15;
    float reflective_stickers_length =0.05;
    float intensity_weak_threshold = 32.f; //反光强度弱阈值(小于等于此阈值被认为噪点)
    unsigned int strong_count_threshold = 15;   //特征点数量阈值（特征点数量小于此阈值的圆柱被舍弃）
    float range_close_threshold = 0.2f;   //范围差阈值（相邻点range差大于此阈值则认为该两点不连续）
    unsigned int weak_count_threshold = 10; //噪点数量阈值（连续噪点数量大于此阈值的圆柱被舍弃）


};

typedef std::shared_ptr<RackRecogInfo> RackRecogInfo_Ptr;
typedef std::shared_ptr<ChargingRecogInfo> ChargingRecogInfo_Ptr;



struct ScanPoints {

    int64_t stamp;

    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
};
typedef std::shared_ptr<ScanPoints> Scan_Ptr;
typedef std::shared_ptr<FeatureCluster> OneFeatureCluster_Ptr;
typedef std::shared_ptr<FeatureClusters> FeatureClusters_Ptr;

enum RecogType {
    TYPE_RECOG_RACK = 1,
    TYPE_RECOG_CHARGINGPILE = 2,
};

class BaseRecognition {

public:
    BaseRecognition(RecogType type_) : type(type_) {

    }

    virtual ~BaseRecognition() {

    }

    RecogType getType() {
        return type;
    }

    virtual bool getPose(Scan_Ptr scan, Eigen::Vector3f &out_pose) {
        recog::FeatureClusters_Ptr clusters(new recog::FeatureClusters);
        if(extractFeature(scan, clusters)){
            std::vector<std::shared_ptr<fitting::FeatPara>> feat_paras;
            if (getPose(clusters, feat_paras, out_pose)) {
                std::cout<<"~~~=====~~~~~"<<std::endl;
                return true;
            }
        }
        std::cout << "!!!====!!!!"<<std::endl;
        return false;
    }

    virtual bool extractFeature(Scan_Ptr scan, FeatureClusters_Ptr clusters) {
        return false;
    }

    virtual bool getPose(FeatureClusters_Ptr clusters, std::vector<std::shared_ptr<fitting::FeatPara>> &feat_paras,
                         Eigen::Vector3f &out_pose) {
        return false;
    }

    void setInfo(RackRecogInfo_Ptr info_) {
        info = info_;
    }
    void setInfo(ChargingRecogInfo_Ptr info_) {
        charging_pile_info = info_;
    }

    template<class T>
    inline void normalizeAngle(T &angle) {
        angle = fmod(angle, 2.0 * M_PI);
        if (angle >= M_PI) {
            angle -= 2.0f * M_PI;
        } else if (angle < -M_PI) {
            angle += 2.0f * M_PI;
        }
    }

protected:
    RackRecogInfo_Ptr info;
    ChargingRecogInfo_Ptr charging_pile_info;
private:
    RecogType type;

};

typedef std::shared_ptr<BaseRecognition> BaseRecognition_Ptr;

}


#endif //PROJECT_BASE_RECOGNITION_HPP
