//
// Created by liuyan on 18-3-9.
//

#ifndef SROS_LINE_PROCESSOR_H
#define SROS_LINE_PROCESSOR_H
#include "fitting_processor.hpp"
namespace fitting {
    class LineProcessor : public FittingProcessor {
    public:
        LineProcessor();

        virtual ~LineProcessor();

        virtual void computeFeatPara(recog::FeatureCluster& cluster,FeatPara_Ptr feat_para);

        void computeLineDistance(const recog::FeatureCluster &cluster , double &length);
        void computeLineCenterPose(const recog::FeatureCluster &cluster , recog::Point &center_pose);

    private:

        virtual void computeCov(recog::FeatureCluster& cluster,FeatPara_Ptr feat_para);
        const double pow2(const double value) {
            return value * value;
        }
    };
}

#endif //SROS_LINE_PROCESSOR_H
