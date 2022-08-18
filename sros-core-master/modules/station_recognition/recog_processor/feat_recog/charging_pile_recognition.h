//
// Created by liuyan on 18-3-9.
//

#ifndef SROS_CHARGING_PILE_RECOGNITION_H
#define SROS_CHARGING_PILE_RECOGNITION_H

#include "base_recognition.hpp"
#include "fitting/fitting_processor.hpp"
#include "gicp.h"
namespace recog {
    class ChargingPileRecognition : public BaseRecognition {
    public:
        ChargingPileRecognition();

        virtual ~ChargingPileRecognition();

        virtual bool extractFeature(Scan_Ptr scan, FeatureClusters_Ptr clusters);

        virtual bool getPose(FeatureClusters_Ptr clusters, std::vector<fitting::FeatPara_Ptr> &feat_paras,
                             Eigen::Vector3f &out_pose);

    private:
        fitting::FittingProcessor_Ptr fitting_proc;
        icp::Gicp icp_processor;

        bool checkTwoFeatureOnSameLine(const FeatureCluster &first_feature, const FeatureCluster &second_feature,
                                       fitting::FeatPara_Ptr &feat_para);

        void getLineDistance(const FeatureCluster &feature, double &length);

        void getLineCenterPose(const FeatureCluster &feature, Point &center_pose);

        Eigen::Vector3f computeChargingPileCenterPose(const Point &first_line_center_pose,const Point &second_line_center_pose );

        void computeNormalizedIntensities(Scan_Ptr scan) ;

        bool isNormalizedIntensityStrong(const unsigned int index);

        void getLandmarks(Scan_Ptr scan);

        bool isRangeClose(const float lrange, const float rrange) ;

        bool isStrongCountEnough(const unsigned int count);

        std::vector<float> normalized_intensities_; //归一化强度(强度/标准强度)
        std::list<std::pair<unsigned int, unsigned int>> landmarks_; //所有可能存在的信标，存储信标首尾对应的序号
        void getLandMarkFeature(Scan_Ptr scan ,FeatureClusters_Ptr clusters);
    };
}
#endif //SROS_CHARGING_PILE_RECOGNITION_H
